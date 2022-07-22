// This file is part of KWIVER, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/kwiver/blob/master/LICENSE for details.

/// \file
///
/// This tool renders a mesh into an image depth or height map

#include "texture_from_pointcloud.h"

#include <fstream>
#include <iostream>

#include <kwiversys/Directory.hxx>
#include <kwiversys/SystemTools.hxx>

#include <vital/algo/nearest_neighbors.h>
#include <vital/algo/pointcloud_io.h>
#include <vital/algo/uv_unwrap_mesh.h>
#include <vital/applets/config_validation.h>
#include <vital/config/config_block.h>
#include <vital/config/config_block_formatter.h>
#include <vital/config/config_block_io.h>
#include <vital/exceptions.h>
#include <vital/io/mesh_io.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>
#include <vital/types/pointcloud.h>
#include <vital/util/get_paths.h>

#include <vital/plugin_loader/plugin_manager.h>

#include <arrows/core/mesh_operations.h>

namespace kwiver {
namespace arrows {
namespace core {
namespace {

kwiver::vital::logger_handle_t main_logger( kwiver::vital::get_logger(
                                              "texture_from_pointcloud" ) );

// ----------------------------------------------------------------------------
bool
check_config( kwiver::vital::config_block_sptr config )
{
  using namespace kwiver::tools;

  bool config_valid = true;

#define KWIVER_CONFIG_FAIL( msg ) \
  LOG_ERROR( main_logger, "Config Check Fail: " << msg ); \
  config_valid = false

  config_valid =
    validate_required_input_dir( "mesh_directory", *config,  main_logger ) &&
    config_valid;

  config_valid =
    validate_required_input_file( "point_cloud_file", *config, main_logger ) &&
    config_valid;

#undef KWIVER_CONFIG_FAIL

  return config_valid;
}
} // end namespace

class texture_from_pointcloud::priv
{
public:

  priv() {}

  kwiver::vital::config_block_sptr config;
  kwiver::vital::algo::nearest_neighbors_sptr nn_search;
  kwiver::vital::algo::pointcloud_io_sptr point_cloud_reader;
  kwiver::vital::algo::uv_unwrap_mesh_sptr uv_unwrapper;

  kwiver::vital::path_t mesh_directory;
  kwiver::vital::path_t point_cloud_file;

  std::string mesh_extension = ".obj";

  enum commandline_mode { SUCCESS, HELP, WRITE, FAIL };

  commandline_mode
  process_command_line( cxxopts::ParseResult& cmd_args )
  {
    static std::string opt_config;
    static std::string opt_out_config;

    if ( cmd_args[ "help" ].as< bool >() )
    {
      return HELP;
    }
    if ( cmd_args.count( "config" ) > 0 )
    {
      opt_config = cmd_args[ "config" ].as< std::string >();
    }
    if ( cmd_args.count( "output-config" ) > 0 )
    {
      opt_out_config = cmd_args[ "output-config" ].as< std::string >();
    }
    if( cmd_args.count( "mesh-ext" ) > 0 )
    {
      mesh_extension = cmd_args[ "mesh-ext" ].as< std::string >();
    }

    // Set up top level configuration w/ defaults where applicable.
    config = default_config();

    // If -c/--config given, read in confg file, merge in with default just
    // generated
    if( !opt_config.empty() )
    {
      config->merge_config( kwiver::vital::read_config_file( opt_config ) );
    }

    if( cmd_args.count( "mesh-dir" ) > 0 )
    {
      mesh_directory = cmd_args[ "mesh-dir" ].as< std::string >();
      config->set_value( "mesh_directory", mesh_directory );
    }
    if( cmd_args.count( "point-cloud-file" ) > 0 )
    {
      point_cloud_file = cmd_args[ "point-cloud-file" ].as< std::string >();
      config->set_value( "point_cloud_file", point_cloud_file );
    }

    bool valid_config = check_config( config );

    if( !opt_out_config.empty() )
    {
      write_config_file( config, opt_out_config );
      if( valid_config )
      {
        LOG_INFO( main_logger,
                  "Configuration file contained valid parameters and may be "
                  "used for running" );
      }
      else
      {
        LOG_WARN( main_logger, "Configuration deemed not valid." );
      }
      config = nullptr;
      return WRITE;
    }
    else if( !valid_config )
    {
      LOG_ERROR( main_logger, "Configuration not valid." );
      config = nullptr;
      return FAIL;
    }

    return SUCCESS;
  }

  // ------------------------------------------------------------------

  static kwiver::vital::config_block_sptr
  default_config()
  {
    kwiver::vital::config_block_sptr config =
      kwiver::vital::config_block::empty_config( "texture-from-pointcloud-tool" );

    config->set_value( "nearest_neighbors:type", "vxl_kd_tree",
                       "Implementation for nearest neighbor search." );

    config->set_value( "pointcloud_io:type", "pdal",
                       "Implementation of point cloud reader.");

    config->set_value( "uv_unwrap_mesh:type", "core",
                       "Implementation of uv mesh unwrapper.");

    kwiver::vital::algo::nearest_neighbors::get_nested_algo_configuration(
                              "nearest_neighbors",
                              config,
                              kwiver::vital::algo::nearest_neighbors_sptr() );
    kwiver::vital::algo::pointcloud_io::get_nested_algo_configuration(
                              "pointcloud_io",
                              config,
                              kwiver::vital::algo::pointcloud_io_sptr() );
    kwiver::vital::algo::uv_unwrap_mesh::get_nested_algo_configuration(
                              "uv_unwrap_mesh",
                              config,
                              kwiver::vital::algo::uv_unwrap_mesh_sptr() );

    return config;
  }

  void
  initialize()
  {
    // Create algorithm from configuration
    kwiver::vital::algo::nearest_neighbors::set_nested_algo_configuration(
      "nearest_neighbors", config, nn_search );
    kwiver::vital::algo::pointcloud_io::set_nested_algo_configuration(
      "pointcloud_io", config, point_cloud_reader );
    kwiver::vital::algo::uv_unwrap_mesh::set_nested_algo_configuration(
      "uv_unwrap_mesh", config, uv_unwrapper );
  }

  void
  run_algorithm()
  {
    auto point_cloud = point_cloud_reader->load(point_cloud_file);

    kwiversys::Directory mesh_dir;
    mesh_dir.Load(mesh_directory);
    auto num_mesh_files = mesh_dir.GetNumberOfFiles();

    std::vector<double> utm_corr;
    bool has_utm_corr = false;
    for (unsigned long i = 0; i < num_mesh_files; ++i )
    {
      std::string mesh_file = mesh_dir.GetPath();
      mesh_file += "/" + std::string( mesh_dir.GetFile( i ) );
      if ( kwiversys::SystemTools::GetFilenameLastExtension( mesh_file ) == mesh_extension )
      {
        // Check first file for UTM correction
        if ( !has_utm_corr )
        {
          utm_corr = get_utm_correction( mesh_file );
          has_utm_corr = true;
        }

        std::cout << "MESH_FILE: " << mesh_file << std::endl;
        auto input_mesh = kwiver::vital::read_mesh( mesh_file );

        if ( input_mesh->faces().regularity() != 3 )
        {
          kwiver::arrows::core::mesh_triangulate(*input_mesh);
        }

        uv_unwrapper->unwrap( input_mesh );
      }
    }
  }

  std::vector<double>
  get_utm_correction( kwiver::vital::path_t mesh_file )
  {
    std::vector<double> correction = {0., 0., 0.};

    std::ifstream file;
    file.open( mesh_file );
    std::string line;
    if ( file )
    {
      for ( size_t i = 0; i < 3; ++i )
      {
        if ( std::getline( file, line ) )
        {
          std::vector< std::string > keys = { "#x", "#y", "#z "};
          for ( size_t j = 0; j < 3; ++j )
          {
            if ( line.rfind( keys[j], 0 ) == 0 )
            {
              correction[j] = std::stod( line.substr( line.rfind( " " ) ) );
            }
          }
        }
      }
    }
    file.close();

    return correction;
  }

  //kwiver::vital::image_container_sptr
  void
  texture_mesh( kwiver::vital::pointcloud_sptr point_cloud,
                kwiver::vital::mesh_sptr mesh )
  {

  }
};

// ----------------------------------------------------------------------------

void
texture_from_pointcloud
  ::add_command_options()
{
  m_cmd_options->custom_help( wrap_text(
                                "This tool texures a set of meshes using point cloud data.\n"
                                "\n"
                                "Usage: kwiver " + applet_name() +
                                " [options] mesh-dir point-cloud-file" ) );

  m_cmd_options->positional_help( "\n mesh-dir         - directory that holds "
                                  "the mesh files."
                                  "\n point-cloud-file - the file that contains "
                                  "the point cloud data." );

  m_cmd_options->add_options()
    ( "h,help",          "Display usage information" )
    ( "c,config",        "Configuration file for tool", cxxopts::value< std::string >() )
    ( "o,output-config", "Dump configuration for tool", cxxopts::value< std::string >() )
    ( "m,mesh-ext",      "Mesh file extension, defaults to *.obj", cxxopts::value< std::string >() )
    // positional parameters
    ( "mesh-dir",         "Mesh directory", cxxopts::value< std::string >() )
    ( "point-cloud-file", "Point cloud file name", cxxopts::value< std::string >() );

  m_cmd_options->parse_positional( { "mesh-dir", "point-cloud-file" } );
}

// ----------------------------------------------------------------
/** Main entry. */

int
texture_from_pointcloud
  ::run()
{
  try
  {
    switch(d->process_command_line(command_args()))
    {
      case priv::HELP:
        std::cout << m_cmd_options->help();
        return EXIT_SUCCESS;
      case priv::WRITE:
        return EXIT_SUCCESS;
      case priv::FAIL:
        return EXIT_FAILURE;
      case priv::SUCCESS:
        ;
    }

    if ( d->config == nullptr )
    {
      return EXIT_FAILURE;
    }

    if( d->nn_search == nullptr || d->point_cloud_reader == nullptr )
    {
      d->initialize();
    }

    LOG_INFO(main_logger, "Finished configuring");
    d->run_algorithm();
    LOG_INFO(main_logger, "Finished computing");

    return EXIT_SUCCESS;
  }
  catch (std::exception const& e)
  {
    LOG_ERROR(main_logger, "Exception caught: " << e.what());

    return EXIT_FAILURE;
  }
  catch (...)
  {
    LOG_ERROR(main_logger, "Unknown exception caught");

    return EXIT_FAILURE;
  }
} // run

// ----------------------------------------------------------------------------

texture_from_pointcloud
  ::texture_from_pointcloud()
  : d( new priv() )
{}

texture_from_pointcloud::
  ~texture_from_pointcloud() = default;
} // namespace core
} // namespace arrows
}     // end namespace

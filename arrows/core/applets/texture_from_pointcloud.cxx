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

#include <vital/algo/image_io.h>
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

  config_valid =
    validate_required_output_dir( "output_directory", *config,  main_logger, true ) &&
    config_valid;

#undef KWIVER_CONFIG_FAIL

  return config_valid;
}
} // end namespace

// Get the square of the area of a triangle with Heron's formula
double triangle_area( std::vector< kwiver::vital::vector_3d > tri )
{
  std::vector< double > lens;
  double s = 0.0;
  for ( size_t i = 0; i < 3; ++i )
  {
    lens.push_back( ( tri[i] - tri[(i+1)%3] ).norm() );
    s += 0.5*lens.back();
  }

  return s*(s-lens[0])*(s-lens[1])*(s-lens[2]);
}

// Get the barycentric coordinates for a point on a triangle
void barycentric(double& u, double& v,
                 double x, double y,
                 std::vector< kwiver::vital::vector_2d > pts )
{
  double denom = ( pts[1][1] - pts[2][1] )*( pts[0][0] - pts[2][0] ) +
                 ( pts[2][0] - pts[1][0] )*( pts[0][1] - pts[2][1] );

  if ( denom == 0. )
  {
    return;
  }

  u = ( ( pts[1][1] - pts[2][1] )*( x - pts[2][0]) +
        ( pts[2][0] - pts[1][0] )*( y - pts[2][1] ) ) / denom;
  v = ( ( pts[2][1] - pts[0][1] )*( x - pts[2][0]) +
        ( pts[0][0] - pts[2][0] )*( y - pts[2][1] ) ) / denom;
}

class texture_from_pointcloud::priv
{
public:

  priv() {}

  kwiver::vital::config_block_sptr config;
  kwiver::vital::algo::nearest_neighbors_sptr nn_search;
  kwiver::vital::algo::pointcloud_io_sptr point_cloud_reader;
  kwiver::vital::algo::uv_unwrap_mesh_sptr uv_unwrapper;
  kwiver::vital::algo::image_io_sptr image_writer;

  kwiver::vital::path_t mesh_directory;
  kwiver::vital::path_t point_cloud_file;
  kwiver::vital::path_t output_directory;

  // Hard code image size for now
  unsigned int img_width = 500;
  unsigned int img_height = 500;

  std::string mesh_extension = ".obj";
  std::string mtl_template = "newmtl mat\n"
                             "Ka 1.0 1.0 1.0\n"
                             "Kd 1.0 1.0 1.0\n"
                             "d 1\n"
                             "Ns 75\n"
                             "illum 1\n"
                             "map_Kd ";

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
    if( cmd_args.count( "output-dir" ) > 0 )
    {
      output_directory = cmd_args[ "output-dir" ].as< std::string >();
      config->set_value( "output_directory", output_directory );
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

    config->set_value( "image_io:type", "vxl",
                       "Implementation of the image writer.");

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
    kwiver::vital::algo::image_io::get_nested_algo_configuration(
                              "image_io",
                              config,
                              kwiver::vital::algo::image_io_sptr() );

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
    kwiver::vital::algo::image_io::set_nested_algo_configuration(
      "image_io", config, image_writer );
  }

  void
  run_algorithm()
  {
    auto point_cloud = std::make_shared< kwiver::vital::pointcloud_d >(
      point_cloud_reader->load(point_cloud_file) );

    std::vector< kwiver::vital::point_3d > points;
    for ( auto pt : point_cloud->positions() )
    {
      points.push_back( kwiver::vital::point_3d ( pt ) );
    }
    nn_search->build( points );

    kwiversys::Directory mesh_dir;
    mesh_dir.Load(mesh_directory);
    auto num_mesh_files = mesh_dir.GetNumberOfFiles();

    for (unsigned long i = 0; i < num_mesh_files; ++i )
    {
      std::string mesh_path = mesh_dir.GetPath();
      std::string mesh_file = std::string( mesh_dir.GetFile( i ) );
      if ( kwiversys::SystemTools::GetFilenameLastExtension( mesh_file ) == mesh_extension )
      {
        auto input_mesh = kwiver::vital::read_mesh( mesh_path + "/" + mesh_file );

        if ( input_mesh->faces().regularity() != 3 )
        {
          kwiver::arrows::core::mesh_triangulate(*input_mesh);
        }

        uv_unwrapper->unwrap( input_mesh );

        auto tex_image = texture_mesh( point_cloud, input_mesh );

        // Write out the texture image file
        std::string image_file =
          kwiversys::SystemTools::GetFilenameWithoutExtension( mesh_file ) + ".png";

        image_writer->save( output_directory + "/" + image_file, tex_image );

        // Write out the mtl file
        std::string mtl_filename =
          kwiversys::SystemTools::GetFilenameWithoutExtension( mesh_file ) + ".mtl";
        std::ofstream mtl_file( output_directory + "/" + mtl_filename );
        mtl_file << mtl_template << image_file << std::endl;
        mtl_file.close();

        // Write out the new mesh file
        std::string mesh_filename =
          kwiversys::SystemTools::GetFilenameWithoutExtension( mesh_file ) + ".obj";
        input_mesh->set_tex_source( mtl_filename );
        kwiver::vital::write_obj( output_directory + "/" + mesh_filename, *input_mesh );
      }
    }
  }

  kwiver::vital::image_container_sptr
  texture_mesh( kwiver::vital::pointcloud_sptr point_cloud,
                kwiver::vital::mesh_sptr mesh )
  {
    kwiver::vital::image texture_image( img_width, img_height, 3 );

    std::shared_ptr< kwiver::vital::mesh_face_array > faces =
      std::make_shared< kwiver::vital::mesh_face_array >( mesh->faces() );
    auto vertices = mesh->vertices<3>();

    double img_dx = double(1./img_width);
    double img_dy = double(1./img_height);

    for ( size_t i = 0; i < mesh->num_faces(); ++i )
    {
      double x_min = 1.;
      double y_min = 1.;
      double x_max = 0.;
      double y_max = 0.;

      auto pc_data = point_cloud->colors();

      std::vector< kwiver::vital::vector_2d > tx_coords;
      std::map < int, int > x_t = { { 0, 0 }, { 1, 1 }, { 2, 0 } };
      std::map < int, int > y_t = { { 0, 1 }, { 1, 0 }, { 2, 0 } };
      for ( size_t j = 0; j < 3; ++j )
      {
        auto tmp_crd = mesh->texture_map(i, x_t[j], y_t[j]);

        x_min = std::min(tmp_crd[0], x_min);
        y_min = std::min(tmp_crd[1], y_min);
        x_max = std::max(tmp_crd[0], x_max);
        y_max = std::max(tmp_crd[1], y_max);

        tx_coords.push_back( tmp_crd );
      }

      std::vector< kwiver::vital::vector_3d > corners;
      for ( auto idx : (*faces)[i] )
      {
        corners.push_back( vertices[idx] );
      }

      std::vector< kwiver::vital::point_3d > pixel_pts;
      std::vector< kwiver::vital::point_2i > pixel_indices;
      for ( double x = x_min; x < x_max; x += img_dx )
      {
        for ( double y = y_min; y < y_max; y += img_dy )
        {
          double u = -1.;
          double v = -1.;

          barycentric(u, v, x, y, tx_coords);

          if ( ( 0. <= u ) && ( u <= 1.) && ( 0. <= v ) &&
               ( v <= 1. ) && ( u + v <= 1. ) )
          {
            pixel_pts.push_back(
              kwiver::vital::point_3d ( (1. - u - v)*corners[0] +
                                        v*corners[1] +
                                        u*corners[2]) );

            pixel_indices.push_back(
              kwiver::vital::point_2i( int( x*img_width ),
                                      int( (1. - y)*img_height ) ) );
          }
        }
      }

      std::vector< std::vector< int > > closest_indices;
      std::vector< std::vector< double > > closest_dists;
      nn_search->find_nearest_points(pixel_pts, 1, closest_indices, closest_dists);

      for ( size_t j = 0; j < pixel_indices.size(); ++j )
      {
        auto x_idx = pixel_indices[j][0];
        auto y_idx = pixel_indices[j][1];
        auto px_color = pc_data[ closest_indices[j][0] ];
        texture_image.at<uint8_t>( x_idx, y_idx, 0 ) = px_color.r;
        texture_image.at<uint8_t>( x_idx, y_idx, 1 ) = px_color.g;
        texture_image.at<uint8_t>( x_idx, y_idx, 2 ) = px_color.b;
      }
    }

    return std::make_shared< kwiver::vital::simple_image_container >(
      kwiver::vital::simple_image_container( texture_image ) );
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
                                " [options] mesh-dir point-cloud-file output-dir" ) );

  m_cmd_options->positional_help( "\n mesh-dir         - directory that holds "
                                  "the mesh files."
                                  "\n point-cloud-file - the file that contains "
                                  "the point cloud data."
                                  "\n output-dir       - directory where the new"
                                  " the new files will be written.");

  m_cmd_options->add_options()
    ( "h,help",          "Display usage information" )
    ( "c,config",        "Configuration file for tool", cxxopts::value< std::string >() )
    ( "o,output-config", "Dump configuration for tool", cxxopts::value< std::string >() )
    ( "m,mesh-ext",      "Mesh file extension, defaults to *.obj", cxxopts::value< std::string >() )
    // positional parameters
    ( "mesh-dir",         "Mesh directory", cxxopts::value< std::string >() )
    ( "point-cloud-file", "Point cloud file name", cxxopts::value< std::string >() )
    ( "output-dir",       "Directory to write new files too", cxxopts::value< std::string >() );

  m_cmd_options->parse_positional( { "mesh-dir", "point-cloud-file", "output-dir" } );
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

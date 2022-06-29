// This file is part of KWIVER, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/kwiver/blob/master/LICENSE for details.

/// \file
///
/// This tool renders a mesh into an image depth or height map

#include "texture_from_pointcloud.h"

#include <iostream>
#include <fstream>

#include <vital/algo/nearest_neighbors.h>
#include <vital/config/config_block.h>
#include <vital/config/config_block_io.h>
#include <vital/config/config_block_formatter.h>
#include <vital/exceptions.h>
#include <vital/io/mesh_io.h>
#include <vital/util/get_paths.h>

#include <vital/plugin_loader/plugin_manager.h>

#include <arrows/core/mesh_operations.h>

namespace kwiver {
namespace arrows {
namespace core {

namespace {

// Global options
std::string opt_out_config;     // output config file name
int opt_width;
int opt_height;

// ------------------------------------------------------------------
static kwiver::vital::config_block_sptr default_config()
{
  kwiver::vital::config_block_sptr config =
    kwiver::vital::config_block::empty_config( "texture-from-pointcloud-tool" );

  config->set_value( "nearest_neighbors:type", "vxl_kd_tree",
                     "Implementation for nearest neighbor search." );

  kwiver::vital::algo::nearest_neighbors::get_nested_algo_configuration(
    "nearest_neighbors", config, kwiver::vital::algo::nearest_neighbors_sptr() );

  return config;
}

} // end namespace

class texture_from_pointcloud::priv
{
public:
  priv() {}
};

// ----------------------------------------------------------------------------
void
texture_from_pointcloud::
add_command_options()
{
  m_cmd_options->custom_help( wrap_text(
       "This tool texures a set of meshes using point cloud data.\n"
       "\n"
       "Usage: kwiver " + applet_name() + " [options] mesh-dir point-cloud-file"
          ) );

  m_cmd_options->positional_help( "\n mesh-dir         - directory that holds "
                                  "the mesh files."
                                  "\n point-cloud-file - the file that contains "
                                  "the point cloud data.");

  m_cmd_options->add_options()
    ( "h,help",        "Display usage information" )
    ( "c",             "Configuration file for tool" )
    ( "output-config", "Dump configuration for tool", cxxopts::value<std::string>() )

    // positional parameters
    ( "mesh-dir",          "Mesh directory", cxxopts::value<std::string>() )
    ( "point-cloud-file",  "Point cloud file name", cxxopts::value<std::string>() );

    m_cmd_options->parse_positional({"mesh-dir", "point-cloud-file"});
}

// ----------------------------------------------------------------
/** Main entry. */
int
texture_from_pointcloud::
run()
{
  auto& cmd_args = command_args();

  if ( cmd_args["help"].as<bool>() )
  {
    std::cout << m_cmd_options->help();
    return EXIT_SUCCESS;
  }

  // If we are not writing out the config, then all positional file
  // names are required.
  if ( cmd_args.count("output-config") == 0 )
  {
    if ( ( cmd_args.count("mesh-dir") == 0 ) ||
         ( cmd_args.count("point-cloud-file") == 0 ) )
    {
      std::cout << "Missing file name.\n"
                << "Usage: " << applet_name()
                << " mesh-dir point-cloud-file\n"
                << std::endl;

      return EXIT_FAILURE;
    }
  }

  kwiver::vital::algo::nearest_neighbors_sptr nn_search;
  kwiver::vital::config_block_sptr config = default_config();
  // If --config given, read in config file, merge in with default just generated
  if ( cmd_args.count("c") > 0 )
  {
    config->merge_config( kwiver::vital::read_config_file( cmd_args["c"].as<std::string>() ) );
  }

  kwiver::vital::algo::nearest_neighbors::set_nested_algo_configuration( "nearest_neighbors", config, nn_search );
  kwiver::vital::algo::nearest_neighbors::get_nested_algo_configuration( "nearest_neighbors", config, nn_search );

  // Check to see if we are to dump config
  if ( cmd_args.count("output-config") > 0 )
  {
    opt_out_config = cmd_args["output-config"].as<std::string>();
    std::ofstream fout( opt_out_config );
    if( ! fout )
    {
      std::cout << "Couldn't open \"" << opt_out_config << "\" for writing.\n";
      return EXIT_FAILURE;
    }

    kwiver::vital::config_block_formatter fmt( config );
    fmt.print( fout );
    std::cout << "Wrote config to \"" << opt_out_config << "\". Exiting.\n";
    return EXIT_SUCCESS;
  }

  if( !kwiver::vital::algo::nearest_neighbors::check_nested_algo_configuration( "nearest_neighbors", config ) )
  {
    std::cerr << "Invalid nearest_neighbors config" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string mesh_dir = cmd_args["mesh-dir"].as<std::string>();
  const std::string point_cloud_file = cmd_args["point-cloud-file"].as<std::string>();

  opt_width = cmd_args["x"].as<int>();
  opt_height = cmd_args["y"].as<int>();

  // std::cout << "Reading Mesh" << std::endl;
  // auto mesh = kwiver::vital::read_mesh(mesh_file);

  // if ( mesh->faces().regularity() != 3 )
  // {
  //   std::cout << "Triangulating Mesh" << std::endl;
  //   kwiver::arrows::core::mesh_triangulate(*mesh);
  // }

  return EXIT_SUCCESS;
}

// ----------------------------------------------------------------------------
texture_from_pointcloud::
texture_from_pointcloud()
 : d(new priv())
{ }

texture_from_pointcloud::
~texture_from_pointcloud() = default;

} } } // end namespace

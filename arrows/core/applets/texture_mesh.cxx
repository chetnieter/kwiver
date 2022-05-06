// This file is part of KWIVER, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/kwiver/blob/master/LICENSE for details.

/// \file
///
/// This tool renders a mesh into an image depth or height map

#include "texture_mesh.h"

#include <iostream>
#include <fstream>

#include <kwiversys/Glob.hxx>
#include <kwiversys/SystemTools.hxx>

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
    kwiver::vital::config_block::empty_config( "texture-mesh-tool" );

  return config;
}

} // end namespace

// ----------------------------------------------------------------------------
void
texture_mesh::
add_command_options()
{
  m_cmd_options->custom_help( wrap_text(
       "This tool creates a texture map for a set of meshes with point cloud data\n"
       "\n"
       "Usage: kwiver " + applet_name() + " [options] mesh-dir point-cloud-file output-dir"
          ) );

  m_cmd_options->positional_help( "\n   mesh-dir - Mesh directory name.\n"
                                  "   pcd - point cloud file name.\n"
                                  "   output-dir - output file name.");

  m_cmd_options->add_options()
    ( "h,help",        "Display usage information" )
    ( "c",             "Configuration file for tool" )
    ( "output-config", "Dump configuration for tool", cxxopts::value<std::string>() )

    // positional parameters
    ( "mesh-dir",    "Mesh directory name", cxxopts::value<std::string>() )
    ( "pcd-file",    "Point cloud file name", cxxopts::value<std::string>() )
    ( "output-dir", "Output image file name", cxxopts::value<std::string>() )
    ;

    m_cmd_options->parse_positional({"mesh-dir", "pcd-file", "output-dir"});
}

// ----------------------------------------------------------------
/** Main entry. */
int
texture_mesh::
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
         ( cmd_args.count("pcd-file") == 0 ) ||
         ( cmd_args.count("output-dir") == 0 ) )
    {
      std::cout << "Missing file name.\n"
                << "Usage: " << applet_name()
                << " mesh-dir pcd-file output-dir\n"
                << std::endl;

      return EXIT_FAILURE;
    }
  }

  kwiver::vital::config_block_sptr config = default_config();
  // If --config given, read in config file, merge in with default just generated
  if ( cmd_args.count("c") > 0 )
  {
    config->merge_config( kwiver::vital::read_config_file( cmd_args["c"].as<std::string>() ) );
  }

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

  const std::string mesh_dir = cmd_args["mesh-dir"].as<std::string>();
  const std::string pcd_file = cmd_args["pcd-file"].as<std::string>();
  const std::string output_dir = cmd_args["output-dir"].as<std::string>();

  kwiversys::Glob meshGlob;
  std::vector< std::string > globStr = {mesh_dir, "*.obj"};
  meshGlob.FindFiles(kwiversys::SystemTools::JoinPath(globStr));
  auto mesh_files = meshGlob.GetFiles();

  std::cout << "Reading Meshes" << std::endl;
  std::vector< kwiver::vital::mesh_sptr > meshes;
  for ( auto f : mesh_files)
  {
    auto mesh = kwiver::vital::read_mesh(f);
    if ( mesh->faces().regularity() != 3 )
    {
      std::cout << "Triangulating Mesh " << f << std::endl;
      kwiver::arrows::core::mesh_triangulate(*mesh);
    }
    meshes.push_back( mesh );
  }

  return EXIT_SUCCESS;
}

} } } // end namespace

// This file is part of KWIVER, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/kwiver/blob/master/LICENSE for details.

#ifndef KWIVER_ARROWS_CORE_TOOLS_TEXTURE_MESH_H
#define KWIVER_ARROWS_CORE_TOOLS_TEXTURE_MESH_H

#include <vital/applets/kwiver_applet.h>

#include <arrows/core/applets/kwiver_algo_core_applets_export.h>

#include <string>
#include <vector>

namespace kwiver {
namespace arrows {
namespace core {

class KWIVER_ALGO_CORE_APPLETS_EXPORT texture_mesh
  : public kwiver::tools::kwiver_applet
{
public:
  texture_mesh(){}
  virtual ~texture_mesh() = default;

  PLUGIN_INFO( "texture-mesh",
               "Texture map a set of meshes with point cloud data.\n\n"
               "This tool reads in a group of mesh files and a point cloud file "
               "and generates texture map images from the color data in the "
               "point cloud.");

  int run() override;
  void add_command_options() override;

protected:

private:

}; // end of class

} } } // end namespace

#endif /* KWIVER_ARROWS_CORE_TOOLS_TEXTURE_MESH_H */

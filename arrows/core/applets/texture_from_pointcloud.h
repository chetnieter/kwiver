// This file is part of KWIVER, and is distributed under the
// OSI-approved BSD 3-Clause License. See top-level LICENSE file or
// https://github.com/Kitware/kwiver/blob/master/LICENSE for details.

#ifndef KWIVER_ARROWS_CORE_TOOLS_TEXTURE_FROM_POINTCLOUD_H
#define KWIVER_ARROWS_CORE_TOOLS_TEXTURE_FROM_POINTCLOUD_H

#include <vital/applets/kwiver_applet.h>

#include <arrows/core/applets/kwiver_algo_core_applets_export.h>

#include <string>
#include <vector>

namespace kwiver {
namespace arrows {
namespace core {

class KWIVER_ALGO_CORE_APPLETS_EXPORT texture_from_pointcloud
  : public kwiver::tools::kwiver_applet
{
public:
  texture_from_pointcloud();
  virtual ~texture_from_pointcloud();

  PLUGIN_INFO( "texture-from-pointcloud",
               "Texture a set of meshes from point cloud data.\n\n"
               "This tool reads in a set of meshes and a point cloud "
               "and creates texture maps from the color data in the point cloud.");

  int run() override;
  void add_command_options() override;

protected:

private:
  class priv;
  std::unique_ptr<priv> d;


}; // end of class

} } } // end namespace

#endif /* KWIVER_ARROWS_CORE_TOOLS_TEXTURE_FROM_POINTCLOUD_H */

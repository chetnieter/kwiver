/*ckwg +29
 * Copyright 2018 by Kitware, SAS.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither name of Kitware, Inc. nor the names of any contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Implementation of kwiver::arrows::render_mesh_depth_map function
 */

#ifndef KWIVER_ARROWS_CORE_RENDER_MESH_DEPTH_MAP_H
#define KWIVER_ARROWS_CORE_RENDER_MESH_DEPTH_MAP_H

#include <arrows/core/kwiver_algo_core_export.h>
#include <arrows/core/triangle_scan_iterator.h>

#include <vital/types/camera_perspective.h>
#include <vital/types/image_container.h>
#include <vital/types/mesh.h>


namespace kwiver {
namespace arrows {
namespace core {

/// This function renders a depth map of a triangular mesh seen by a camera
/**
 * \param mesh [in]
 * \param camera [in]
 * \return a depth map
 */
KWIVER_ALGO_CORE_EXPORT
vital::image_container_sptr render_mesh_depth_map(kwiver::vital::mesh_sptr mesh,
                                                  kwiver::vital::camera_perspective_sptr camera);


/// This function renders a height map of a triangular mesh
/**
 * \param mesh [in]
 * \param camera [in]
 * \return height map
 */
KWIVER_ALGO_CORE_EXPORT
vital::image_container_sptr render_mesh_height_map(kwiver::vital::mesh_sptr mesh,
                                                   kwiver::vital::camera_sptr camera);


/// This function converts a depth map into a height map obtained with a perspective camera
/**
 * \param camera [in]
 * \param depth_map [in]
 * \param height_map [out]
 */
KWIVER_ALGO_CORE_EXPORT
void depth_map_to_height_map(vital::camera_perspective_sptr const& camera,
                             vital::image_of<double>& depth_map,
                             vital::image_of<double>& height_map);


/// This functions renders a triangle and fills it with depth
/**
 * \param v1 [in] 2D triangle point
 * \param v2 [in] 2D triangle point
 * \param v3 [in] 2D triangle point
 * \param depth_v1 [in] corresponding depth
 * \param depth_v2 [in] corresponding depth
 * \param depth_v3 [in] corresponding depth
 * \param depth_img [in/out] depth map used and updated
 */
KWIVER_ALGO_CORE_EXPORT
void render_triangle(const vital::vector_2d& v1, const vital::vector_2d& v2, const vital::vector_2d& v3,
                     double depth_v1, double depth_v2, double depth_v3,
                     vital::image_of<double>& depth_img);


/// This function renders a triangle and linearly interpolating attributes
/**
 * \param v1 [in] 2D triangle point
 * \param v2 [in] 2D triangle point
 * \param v3 [in] 2D triangle point
 * \param depth_v1 [in] corresponding depth
 * \param depth_v2 [in] corresponding depth
 * \param depth_v3 [in] corresponding depth
 * \param attrib_v1 [in] attribute which is interpolated
 * \param attrib_v2 [in] attribute which is interpolated
 * \param attrib_v3 [in] attribute which is interpolated
 * \param depth_img [in/out] depth map used and updated during depth test
 * \param img [out] image on which the triangle is rendered
 */
template<class T>
void render_triangle(const vital::vector_2d& v1, const vital::vector_2d& v2, const vital::vector_2d& v3,
                     double depth_v1, double depth_v2, double depth_v3,
                     T attrib_v1, T attrib_v2, T attrib_v3,
                     vital::image_of<double>& depth_img,
                     vital::image_of<T>& img)
{
  triangle_scan_iterator tsi(v1, v2, v3);
  double attrib_v1_d = static_cast<double>(attrib_v1);
  double attrib_v2_d = static_cast<double>(attrib_v2);
  double attrib_v3_d = static_cast<double>(attrib_v3);

  // Linear interpolation attributes
  vital::vector_3d b1(v2.x()-v1.x(), v2.y()-v1.y(), attrib_v2_d - attrib_v1_d);
  vital::vector_3d b2(v3.x()-v1.x(), v3.y()-v1.y(), attrib_v3_d - attrib_v1_d);
  vital::vector_3d n = b1.cross(b2);
  double A = -n.x()/n.z();
  double B = -n.y()/n.z();
  double C = (v1.x() * n.x() + v1.y() * n.y() + attrib_v1_d * n.z()) / n.z();
  // Linear interpolation depth
  vital::vector_3d b1_d(v2.x()-v1.x(), v2.y()-v1.y(), depth_v2 - depth_v1);
  vital::vector_3d b2_d(v3.x()-v1.x(), v3.y()-v1.y(), depth_v3 - depth_v1);
  vital::vector_3d n_d = b1_d.cross(b2_d);
  double A_d = -n_d.x()/n_d.z();
  double B_d = -n_d.y()/n_d.z();
  double C_d = (v1.x() * n_d.x() + v1.y() * n_d.y() + depth_v1 * n_d.z()) / n_d.z();

  for (tsi.reset(); tsi.next(); )
  {
    int y = tsi.scan_y();
    if (y < 0 || y >= static_cast<int>(img.height()))
      continue;
    int min_x = std::max(0, tsi.start_x());
    int max_x = std::min(static_cast<int>(img.width()) - 1, tsi.end_x());

    double new_i = B * y + C;
    double new_i_d = B_d * y + C_d;
    for (int x = min_x; x <= max_x; ++x)
    {
      double attrib = new_i + A * x;
      double depth = new_i_d + A_d * x;
      if (depth < depth_img(x, y))
      {
        img(x, y) =  static_cast<T>(attrib);
        depth_img(x, y) = depth;
      }
    }
  }
}


/// This functions renders a triangle and fills every pixel with value where depth_img is updated
/**
 * \param v1 [in] 2D triangle point
 * \param v2 [in] 2D triangle point
 * \param v3 [in] 2D triangle point
 * \param depth_v1 [in] corresponding depth
 * \param depth_v2 [in] corresponding depth
 * \param depth_v3 [in] corresponding depth
 * \param value [in] value used to fill the triangle
 * \param depth_img [in/out] depth map used and updated during depth test
 */
template<class T>
void render_triangle(const vital::vector_2d& v1, const vital::vector_2d& v2, const vital::vector_2d& v3,
                     double depth_v1, double depth_v2, double depth_v3,
                     T const& value,
                     vital::image_of<double>& depth_img,
                     vital::image_of<T>& img)
{
  triangle_scan_iterator tsi(v1, v2, v3);

  // Linear interpolation depth
  vital::vector_3d b1(v2.x()-v1.x(), v2.y()-v1.y(), depth_v2 - depth_v1);
  vital::vector_3d b2(v3.x()-v1.x(), v3.y()-v1.y(), depth_v3 - depth_v1);
  vital::vector_3d n = b1.cross(b2);
  double A = -n.x()/n.z();
  double B = -n.y()/n.z();
  double C = (v1.x() * n.x() + v1.y() * n.y() + depth_v1 * n.z()) / n.z();

  for (tsi.reset(); tsi.next(); )
  {
    int y = tsi.scan_y();
    if (y < 0 || y >= static_cast<int>(img.height()))
      continue;
    int min_x = std::max(0, tsi.start_x());
    int max_x = std::min(static_cast<int>(img.width()) - 1, tsi.end_x());

    double new_i = B * y + C;
    for (int x = min_x; x <= max_x; ++x)
    {
      double depth = new_i + A * x;
      if (depth < depth_img(x, y))
      {
        depth_img(x, y) = depth;
        img(x, y) = value;
      }
    }
  }
}


}
}
}
#endif // KWIVER_ARROWS_CORE_RENDER_MESH_DEPTH_MAP_H
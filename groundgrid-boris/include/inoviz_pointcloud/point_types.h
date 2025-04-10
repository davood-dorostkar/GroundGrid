/**
 *
 *  Point Cloud Library point structures for Inoviz data.
 *
 *  @author Dorian Guyot
 */

#ifndef __INOVIZ_POINTCLOUD_POINT_TYPES_H
#define __INOVIZ_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace inoviz_pointcloud {
/** Euclidean Inoviz coordinate, including intensity. */
struct PointXYZI {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float intensity;                 ///< laser intensity reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

};  // namespace inoviz_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(
    inoviz_pointcloud::PointXYZI,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity))

#endif  // __INOVIZ_POINTCLOUD_POINT_TYPES_H

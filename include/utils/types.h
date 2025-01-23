#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointXYZRGBA PointTypeRGBA;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>> PointVector;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<PointTypeRGBA> PointCloudXYZRGBA;

typedef Eigen::Vector2f V2F;
typedef Eigen::Vector2d V2D;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

#define MD(a, b) Eigen::Matrix<double, (a), (b)>
#define VD(a) Eigen::Matrix<double, (a), 1>
#define MF(a, b) Eigen::Matrix<float, (a), (b)>
#define VF(a) Eigen::Matrix<float, (a), 1>

struct Pose6D
{
  /*** the preintegrated Lidar states at the time of IMU measurements in a frame ***/
  double offset_time; // the offset time of IMU measurement w.r.t the first lidar point
  double acc[3];      // the preintegrated total acceleration (global frame) at the Lidar origin
  double gyr[3];      // the unbiased angular velocity (body frame) at the Lidar origin
  double vel[3];      // the preintegrated velocity (global frame) at the Lidar origin
  double pos[3];      // the preintegrated position (global frame) at the Lidar origin
  double rot[9];      // the preintegrated rotation (global frame) at the Lidar origin
};

#endif
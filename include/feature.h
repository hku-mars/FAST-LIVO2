/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIVO_FEATURE_H_
#define LIVO_FEATURE_H_

#include "visual_point.h"

// A salient image region that is tracked across frames.
struct Feature
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum FeatureType
  {
    CORNER,
    EDGELET
  };
  int id_;
  FeatureType type_;     //!< Type can be corner or edgelet.
  cv::Mat img_;          //!< Image associated with the patch feature
  Vector2d px_;          //!< Coordinates in pixels on pyramid level 0.
  Vector3d f_;           //!< Unit-bearing vector of the patch feature.
  int level_;            //!< Image pyramid level where patch feature was extracted.
  VisualPoint *point_;   //!< Pointer to 3D point which corresponds to the patch feature.
  Vector2d grad_;        //!< Dominant gradient direction for edglets, normalized.
  SE3 T_f_w_;            //!< Pose of the frame where the patch feature was extracted.
  float *patch_;         //!< Pointer to the image patch data.
  float score_;          //!< Score of the patch feature.
  float mean_;           //!< Mean intensity of the image patch feature, used for normalization.
  double inv_expo_time_; //!< Inverse exposure time of the image where the patch feature was extracted.
  
  Feature(VisualPoint *_point, float *_patch, const Vector2d &_px, const Vector3d &_f, const SE3 &_T_f_w, int _level)
      : type_(CORNER), px_(_px), f_(_f), T_f_w_(_T_f_w), mean_(0), score_(0), level_(_level), patch_(_patch), point_(_point)
  {
  }

  inline Vector3d pos() const { return T_f_w_.inverse().translation(); }
  
  ~Feature()
  {
    // ROS_WARN("The feature %d has been destructed.", id_);
    delete[] patch_;
  }
};

#endif // LIVO_FEATURE_H_

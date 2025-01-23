/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "visual_point.h"
#include "feature.h"
#include <stdexcept>
#include <vikit/math_utils.h>

VisualPoint::VisualPoint(const Vector3d &pos)
    : pos_(pos), previous_normal_(Vector3d::Zero()), normal_(Vector3d::Zero()),
      is_converged_(false), is_normal_initialized_(false), has_ref_patch_(false)
{
}

VisualPoint::~VisualPoint() 
{
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    delete(*it);
  }
  obs_.clear();
  ref_patch = nullptr;
}

void VisualPoint::addFrameRef(Feature *ftr)
{
  obs_.push_front(ftr);
}

void VisualPoint::deleteFeatureRef(Feature *ftr)
{
  if (ref_patch == ftr)
  {
    ref_patch = nullptr;
    has_ref_patch_ = false;
  }
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    if ((*it) == ftr)
    {
      delete((*it));
      obs_.erase(it);
      return;
    }
  }
}

bool VisualPoint::getCloseViewObs(const Vector3d &framepos, Feature *&ftr, const Vector2d &cur_px) const
{
  // TODO: get frame with same point of view AND same pyramid level!
  if (obs_.size() <= 0) return false;

  Vector3d obs_dir(framepos - pos_);
  obs_dir.normalize();
  auto min_it = obs_.begin();
  double min_cos_angle = 0;
  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    Vector3d dir((*it)->T_f_w_.inverse().translation() - pos_);
    dir.normalize();
    double cos_angle = obs_dir.dot(dir);
    if (cos_angle > min_cos_angle)
    {
      min_cos_angle = cos_angle;
      min_it = it;
    }
  }
  ftr = *min_it;

  // Vector2d ftr_px = ftr->px_;
  // double pixel_dist = (cur_px-ftr_px).norm();

  // if(pixel_dist > 200)
  // {
  //   ROS_ERROR("The pixel dist exceeds 200.");
  //   return false;
  // }

  if (min_cos_angle < 0.5) // assume that observations larger than 60° are useless 0.5
  {
    // ROS_ERROR("The obseved angle is larger than 60°.");
    return false;
  }

  return true;
}

void VisualPoint::findMinScoreFeature(const Vector3d &framepos, Feature *&ftr) const
{
  auto min_it = obs_.begin();
  float min_score = std::numeric_limits<float>::max();

  for (auto it = obs_.begin(), ite = obs_.end(); it != ite; ++it)
  {
    if ((*it)->score_ < min_score)
    {
      min_score = (*it)->score_;
      min_it = it;
    }
  }
  ftr = *min_it;
}

void VisualPoint::deleteNonRefPatchFeatures()
{
  for (auto it = obs_.begin(); it != obs_.end();)
  {
    if (*it != ref_patch)
    {
      delete *it;
      it = obs_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}
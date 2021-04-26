/** @file
 *
 * Definitions of data structures.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_DEFINITIONS_H
#define MULTI_HYPOTHESIS_TRACKING_DEFINITIONS_H

#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace MultiHypothesisTracker
{

struct Detection
{
  Eigen::Vector3f position;             ///< Position of detection - e.g. center point.
  Eigen::Matrix3f covariance;           ///< Covariance of detection.
  std::vector<Eigen::Vector3f> points;  ///< Point cloud corresponding to detection.
};

struct Detections
{
  std::vector<Detection> detections;    ///< Vector of detection at current time_stamp in frame frame_id.

  std::string frame_id;                 ///< Frame ID of detection - e.g. world or sensor frame.
  double time_stamp;                    ///< Time stamp when the detection was created.
};

struct AxisAlignedBox
{
  AxisAlignedBox(){};
  AxisAlignedBox(Eigen::Array3f min, Eigen::Array3f  max) : min_corner(std::move(min)), max_corner(std::move(max)){};
  Eigen::Array3f min_corner;           ///< min corner of axis aligned bounding box.
  Eigen::Array3f max_corner;           ///< max corner of axis aligned bounding box.

  Eigen::Array3f getCenterPosition(){ return (max_corner + min_corner) / 2.f; };
  Eigen::Array3f getSideLengths(){ return (max_corner - min_corner).eval(); };
  
  void moveBox(const Eigen::Array3f& offset)
  {
    min_corner = (min_corner + offset).eval();
    max_corner = (max_corner + offset).eval();
  };
};

}

#endif //MULTI_HYPOTHESIS_TRACKING_DEFINITIONS_H

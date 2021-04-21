/** @file
 *
 * Definitions of data structures.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_DEFINITIONS_H
#define MULTI_HYPOTHESIS_TRACKING_DEFINITIONS_H

#include <vector>

#include <Eigen/Dense>

namespace MultiHypothesisTracker
{

/**
 * @brief Description of detection.
 */
struct Detection
{
  Eigen::Vector3f position;             ///< Position of detection - e.g. center point.
  Eigen::Matrix3f covariance;           ///< Covariance of detection.
  std::vector<Eigen::Vector3f> points;  ///< Point cloud corresponding to detection.

  std::string frame_id;                 ///< Frame ID of detection - e.g. world or sensor frame.
  double time_stamp;                    ///< Time stamp when the detection was created.
};

struct Box
{
  Box(){};
  Box(const Eigen::Array3f& min, const Eigen::Array3f& max) : min_corner(min), max_corner(max){};
  Eigen::Array3f min_corner;           ///< min corner of axis aligned bounding box.
  Eigen::Array3f max_corner;           ///< max corner of axis aligned bounding box.
};

}

#endif //MULTI_HYPOTHESIS_TRACKING_DEFINITIONS_H

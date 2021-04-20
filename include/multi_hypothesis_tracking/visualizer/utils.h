/** @file
 *
 * Helper functions for visualizations publisher.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_UTILS_H
#define MULTI_HYPOTHESIS_TRACKING_UTILS_H

#include <Eigen/Dense>

#include <geometry_msgs/Point.h>

namespace MultiHypothesisTracker
{

/** @breif Converts an Eigen::Vector3f to a geometry_msgs::Point. */
inline void eigenToGeometryMsgs(const Eigen::Vector3f& eigen_vector,
                                geometry_msgs::Point& point)
{
  point.x = eigen_vector(0);
  point.y = eigen_vector(1);
  point.z = eigen_vector(2);
}

}

#endif //MULTI_HYPOTHESIS_TRACKING_UTILS_H

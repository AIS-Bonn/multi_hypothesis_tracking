/** @file
 *
 * Helper functions for multi object tracker.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_OBJECT_TRACKING_UTILS_H
#define MULTI_OBJECT_TRACKING_UTILS_H

#include <sys/time.h>
#include <Eigen/Dense>
#include <math.h>

namespace MultiHypothesisTracker
{

/**
 * @breif Returns current time of the day as a double value
 *
 * @return current time of the day
 */
inline double getTimeHighRes()
{
  timeval curr_time;
  gettimeofday(&curr_time, NULL);
  double time_high_res = ((double)curr_time.tv_sec) + ((double)curr_time.tv_usec) * 1e-6;

  return time_high_res;
}

/// See: https://en.wikipedia.org/wiki/Mahalanobis_distance
inline float mahalanobis(const Eigen::Vector3f& dist,
                         const Eigen::Matrix3f& cov)
{
  return (dist.transpose() * cov.inverse() * dist).eval()(0);
}

/// See: https://en.wikipedia.org/wiki/Bhattacharyya_distance
inline float bhattacharyya(const Eigen::Vector3f& dist,
                           const Eigen::Matrix3f& cov1,
                           const Eigen::Matrix3f& cov2)
{
  const Eigen::Matrix3f cov = (cov1 + cov2) / 2;
  const float d1 = mahalanobis(dist, cov) / 8;
  const float d2 = log(cov.determinant() / sqrt(cov1.determinant() * cov2.determinant())) / 2;
  return d1 + d2;
}

inline float bhattacharyya(const Eigen::Vector3f& mean_1,
                           const Eigen::Vector3f& mean_2,
                           const Eigen::Matrix3f& cov_1,
                           const Eigen::Matrix3f& cov_2)
{
  Eigen::Vector3f means_diff = mean_1 - mean_2;
  return bhattacharyya(means_diff, cov_1, cov_2);
}

}

#endif //MULTI_OBJECT_TRACKING_UTILS_H

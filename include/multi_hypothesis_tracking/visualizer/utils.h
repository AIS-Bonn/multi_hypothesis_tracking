/** @file
 *
 * Helper functions for visualizations publisher.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_UTILS_H
#define MULTI_HYPOTHESIS_TRACKING_UTILS_H

#include <stdlib.h>     /* srand, rand */

#include <Eigen/Dense>

#include <geometry_msgs/Point.h>

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypothesis.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <std_msgs/ColorRGBA.h>

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

/** @breif Computes the total number of points associated with the detections. */
inline int computeTotalNumberOfPoints(const std::vector<Detection>& detections)
{
  int total_number_of_points = 0;
  for(const auto& detection : detections)
    total_number_of_points += (int)detection.points.size();
  return total_number_of_points;
}

/** @breif Converts and accumulates the points associated with the detections into one pcl point cloud. */
inline void convertDetectionsPointsToCloud(const std::vector<Detection>& detections,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  int total_points_count = computeTotalNumberOfPoints(detections);

  cloud->points.resize(total_points_count);
  int point_counter = 0;
  for(const auto& detection : detections)
    for(size_t point_id = 0; point_id < detection.points.size(); point_id++, point_counter++)
      cloud->points[point_counter].getVector3fMap() = detection.points.at(point_id);
}

/** @breif Computes the total number of points associated with the hypotheses. */
inline int computeTotalNumberOfPoints(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  int total_number_of_points = 0;
  for(const auto& hypothesis : hypotheses)
    total_number_of_points += (int)hypothesis->getPointCloud().size();
  return total_number_of_points;
}

/** @breif Converts and accumulates the points associated with the hypotheses into one pcl point cloud. */
inline void convertHypothesesPointsToCloud(const std::vector<std::shared_ptr<Hypothesis>>& hypotheses,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  int total_number_of_points = computeTotalNumberOfPoints(hypotheses);

  cloud->points.resize(total_number_of_points);
  int point_counter = 0;
  for(const auto& hypothesis : hypotheses)
    for(size_t point_id = 0; point_id < hypothesis->getPointCloud().size(); point_id++, point_counter++)
      cloud->points[point_counter].getVector3fMap() = hypothesis->getPointCloud().at(point_id);
}

inline void getColorByID(const unsigned int id,
                         std_msgs::ColorRGBA& color)
{
  srand(id);
  color.r = (rand() % 1000) / 1000.f;
  color.g = (rand() % 1000) / 1000.f;
  color.b = (rand() % 1000) / 1000.f;
}

}

#endif //MULTI_HYPOTHESIS_TRACKING_UTILS_H

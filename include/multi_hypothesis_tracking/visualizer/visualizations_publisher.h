/** @file
 *
 * Class to publish data from the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#ifndef __VISUALIZATIONS_PUBLISHER_H__
#define __VISUALIZATIONS_PUBLISHER_H__

#include <ros/ros.h>

#include <std_msgs/Float32.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/point_cloud.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <multi_hypothesis_tracking_msgs/ObjectDetections.h>
#include <multi_hypothesis_tracking_msgs/Box.h>
#include <multi_hypothesis_tracking_msgs/HypothesesBoxes.h>
#include <multi_hypothesis_tracking_msgs/HypothesesEvaluationBoxes.h>
#include <multi_hypothesis_tracking_msgs/HypothesesBoxesArray.h>
#include <multi_hypothesis_tracking_msgs/HypothesesFull.h>

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypothesis.h>
#include <multi_hypothesis_tracking/utils.h>
#include <multi_hypothesis_tracking/visualizer/utils.h>


namespace MultiHypothesisTracker
{

typedef multi_hypothesis_tracking_msgs::HypothesesEvaluationBoxes HypothesesEvaluationBoxesMsg;
typedef multi_hypothesis_tracking_msgs::HypothesesFull HypothesesFullMsg;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef std::vector<std::shared_ptr<Hypothesis>> Hypotheses;

typedef visualization_msgs::Marker MarkerMsg;
typedef visualization_msgs::MarkerArray MarkerArrayMsg;

class VisualizationsPublisher
{
public:
  /** @brief Constructor */
  VisualizationsPublisher();
  /** @brief Destructor */
  ~VisualizationsPublisher(){};

  void initializePublishers(ros::NodeHandle& node_handle);
  void getRosParameters(ros::NodeHandle& node_handle);

  /** @brief Calls all publishers. */
  void publishAll(const Hypotheses& hypotheses,
                  const ros::Time& stamp);

  /**
   * @brief Return a rather specific marker.
   *
   * @param[in] r           red color value.
   * @param[in] g           green color value.
   * @param[in] b           blue color value.
   * @param[in] name_space  namespace of marker.
   *
   * @return marker.
   */
  MarkerMsg createMarker(float r = 0.0,
                         float g = 1.0,
                         float b = 0.0,
                         const std::string& name_space = "multi_object_tracker");

  /** @brief Publishes detections' positions as markers. */
  void publishDetectionsPositions(const std::vector<Detection>& detections,
                                  const ros::Time& stamp);
  /** @brief Publishes detections' covariances as markers. */
  void publishDetectionsCovariances(const std::vector<Detection>& detections,
                                    const ros::Time& stamp);
  /** @brief Publishes point clouds corresponding to detections. */
  void publishDetectionsPoints(const std::vector<Detection>& detections,
                               const ros::Time& stamp);

  /** @brief Publishes positions of valid hypotheses. */
  void publishHypothesesPositions(const Hypotheses& hypotheses,
                                  const ros::Time& stamp);
  bool isValid(std::shared_ptr<Hypothesis>& hypothesis,
               double current_time) const;
  bool isOldEnough(std::shared_ptr<Hypothesis>& hypothesis,
                   double current_time) const;
  bool wasAssignedOftenEnough(std::shared_ptr<Hypothesis>& hypothesis) const;

  /** @brief Publishes covariances of valid hypotheses. */
  void publishHypothesesCovariances(const Hypotheses& hypotheses,
                                    const ros::Time& stamp);
  /** @brief Publishes points corresponding to valid hypotheses. */
  void publishHypothesesPoints(const Hypotheses& hypotheses,
                               const ros::Time& stamp);

  /** @brief Publishes positions of static and valid hypotheses. */
  void publishStaticHypothesesPositions(const Hypotheses& hypotheses,
                                        const ros::Time& stamp);
  /** @brief Publishes positions of dynamic and valid hypotheses. */
  void publishDynamicHypothesesPositions(const Hypotheses& hypotheses,
                                         const ros::Time& stamp);

  /** @brief Publishes paths of valid hypotheses. */
  void publishHypothesesPaths(const Hypotheses& hypotheses,
                              const ros::Time& stamp);
  /** @brief Publishes bounding boxes of valid hypotheses. */
  void publishHypothesesBoundingBoxes(const Hypotheses& hypotheses,
                                      const ros::Time& stamp);
  /** @brief Publishes predicted positions of valid hypotheses. */
  void publishHypothesesPredictedPositions(const Hypotheses& hypotheses,
                                           const ros::Time& stamp);

  /** @brief Publishes full hypotheses messages. */
  void publishHypothesesFull(const Hypotheses& hypotheses,
                             const ros::Time& stamp);
  /** @brief Publish the bounding boxes of all hypotheses. */
  void publishHypothesesBoxesEvaluation(const Hypotheses& hypotheses,
                                        const ros::Time& stamp);

  /** @brief Publish the likelihood. */
  void publishLikelihood(float likelihood);

private:
  // Parameters
  /** @brief The ID of a fixed frame in the world. */
  std::string m_world_frame;
  /** @brief The duration a hypothesis has to exist to be considered for publishing.
   *
   * If the hypothesis is too young it may be unreliable.
   */
  double m_born_time_threshold;
  /** @brief Number of times a hypotheses has to be assigned to detections to be considered for publishing. */
  int m_number_of_assignments_threshold;
  /** @brief Time offset for predictions. */
  double m_future_time;

  ros::Publisher m_detection_positions_publisher;
  ros::Publisher m_detections_covariances_publisher;
  ros::Publisher m_detections_points_publisher;

  ros::Publisher m_hypotheses_positions_publisher;
  ros::Publisher m_hypotheses_covariance_publisher;
  ros::Publisher m_hypotheses_points_publisher;

  ros::Publisher m_static_hypotheses_positions_publisher;
  ros::Publisher m_dynamic_hypotheses_positions_publisher;

  ros::Publisher m_hypotheses_paths_publisher;
  ros::Publisher m_hypotheses_bounding_boxes_publisher;
  ros::Publisher m_hypotheses_predicted_positions_publisher;

  ros::Publisher m_hypotheses_full_publisher;
  ros::Publisher m_hypotheses_box_evaluation_publisher;

  ros::Publisher m_likelihood_publisher;
};

}

#endif
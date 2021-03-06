/** @file
 *
 * Class to publish data from the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_VISUALIZATIONS_PUBLISHER_H
#define MULTI_HYPOTHESIS_TRACKING_VISUALIZATIONS_PUBLISHER_H

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
#include <multi_hypothesis_tracking/hypotheses/hypothesis_with_bounding_box.h>
#include <multi_hypothesis_tracking/utils.h>
#include <multi_hypothesis_tracking/visualizer/utils.h>


namespace MultiHypothesisTracker
{

typedef multi_hypothesis_tracking_msgs::HypothesesEvaluationBoxes HypothesesEvaluationBoxesMsg;
typedef multi_hypothesis_tracking_msgs::HypothesesFull HypothesesFullMsg;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef std::vector<std::shared_ptr<HypothesisInterface>> Hypotheses;

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

  /** @brief Calls all detections publishers. */
  void publishDetectionsVisualizations(const Detections& detections);
  /** @brief Calls all hypotheses publishers. */
  void publishHypothesesVisualizations(const Hypotheses& hypotheses,
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
  void publishDetectionsPositions(const Detections& detections);
  /** @brief Publishes detections' covariances as markers. */
  void publishDetectionsCovariances(const Detections& detections);
  /** @brief Publishes point clouds corresponding to detections. */
  void publishDetectionsPoints(const Detections& detections);

  /** @brief Publishes positions of valid hypotheses. */
  void publishHypothesesPositions(const Hypotheses& hypotheses,
                                  const ros::Time& stamp);


  template<typename HypothesisType>
  bool isValid(const std::shared_ptr<HypothesisType>& hypothesis,
               double current_time);
  template<typename HypothesisType>
  bool isOldEnough(const std::shared_ptr<HypothesisType>& hypothesis,
                   double current_time);
  template<typename HypothesisType>
  bool wasAssignedOftenEnough(const std::shared_ptr<HypothesisType>& hypothesis);

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

private:
  // Parameters
  /** @brief The ID of a fixed frame in the world. */
  std::string m_world_frame_id;
  /** @brief The duration a hypothesis has to exist to be considered for publishing.
   *
   * If the hypothesis is too young it may be unreliable.
   */
  double m_hypothesis_age_threshold_in_seconds;
  /** @brief Number of times a hypotheses has to be assigned to detections to be considered for publishing. */
  int m_number_of_assignments_threshold;
  /** @brief Time offset for predictions. */
  double m_time_offset_for_predictions;

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
};

}

#endif
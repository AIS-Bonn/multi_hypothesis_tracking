/** @file
 *
 * Class to publish data from the multi object tracker.
 *
 * @author Jan Razlaw
 */

#ifndef __MOT_PUBLISHER_H__
#define __MOT_PUBLISHER_H__

#include <stdlib.h>     /* srand, rand */

#include <queue>

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

#include <multi_hypothesis_tracking/hypothesis.h>
#include <multi_hypothesis_tracking/utils.h>


namespace MultiHypothesisTracker
{

typedef multi_hypothesis_tracking_msgs::HypothesesEvaluationBoxes HypothesesEvaluationBoxesMsg;
typedef multi_hypothesis_tracking_msgs::HypothesesFull HypothesesFullMsg;
typedef multi_hypothesis_tracking_msgs::ObjectDetections ObjectDetectionsMsg;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef std::vector<std::shared_ptr<Hypothesis>> Hypotheses;

typedef visualization_msgs::Marker MarkerMsg;
typedef visualization_msgs::MarkerArray MarkerArrayMsg;

class MOTPublisher
{
public:
  /** @brief Constructor */
  MOTPublisher();
  /** @brief Destructor */
  ~MOTPublisher(){};

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

  void eigenToGeometryMsgs(const Eigen::Vector3f& eigen_vector,
                           geometry_msgs::Point& point) const;
  
  /** @brief Publishes detections' positions as markers. */
  void publishDetectionPositions(const std::vector<Detection>& detections,
                                 const ros::Time& stamp);
  /** @brief Publishes detections' covariances as markers. */
  void publishDetectionsCovariances(const std::vector<Detection>& detections,
                                    const ros::Time& stamp);
  /** @brief Publishes point clouds corresponding to detections. */
  void publishDetectionsPoints(const std::vector<Detection>& detections,
                               const ros::Time& stamp);
  
  /** @brief Publish positions of hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesPositions(const Hypotheses& hypotheses,
                                  const ros::Time& stamp);
  /** @brief Publish covariances of hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesCovariances(const Hypotheses& hypotheses,
                                    const ros::Time& stamp);
  /** @brief Publish positions of hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesPoints(const Hypotheses& hypotheses,
                               const ros::Time& stamp);

  /** @brief Publish positions of static hypotheses that are tracked longer than #m_born_time_threshold */
  void publishStaticHypothesesPositions(const Hypotheses& hypotheses,
                                        const ros::Time& stamp);
  /** @brief Publish positions of dynamic hypotheses that are tracked longer than #m_born_time_threshold */
  void publishDynamicHypothesesPositions(const Hypotheses& hypotheses,
                                         const ros::Time& stamp);

  /** @brief Publish paths of hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesPaths(const Hypotheses& hypotheses,
                              const ros::Time& stamp);
  /** @brief Publish bounding boxes of all hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesBoundingBoxes(const Hypotheses& hypotheses,
                                      const ros::Time& stamp);
  /** @brief Publish predicted positions of hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesPredictedPositions(const Hypotheses& hypotheses,
                                           const ros::Time& stamp);
  
  /** @brief Publish hypotheses that are tracked longer than #m_born_time_threshold */
  void publishHypothesesFull(const Hypotheses& hypotheses,
                             const ros::Time& stamp);
  /** @brief Publish hypotheses' predictions that are tracked longer than #m_born_time_threshold */
  void publishHypothesesPredictions(const Hypotheses& hypotheses,
                                    const ros::Time& stamp);
  /** @brief Publish the bounding boxes of dynamic hypotheses that were assigned in the current step. */
  void publishHypothesesBoxesEvaluation(const Hypotheses& hypotheses,
                                        const ros::Time& stamp);

  /** @brief Publish the likelihood. */
  void publishLikelihood(float likelihood);

private:
  // Parameters
  /** @brief A fixed frame in the world. */
  std::string m_world_frame;
  /** @brief Time after which we start publishing a new hypothesis.
   *
   * If the hypothesis is too young it may be unreliable and therefore it
   * will be removed by the isSpurious.
   */
  double m_born_time_threshold;
  /** @brief Number of times a hypotheses has to be assigned to detections to be considered valid. */
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
  ros::Publisher m_hypotheses_predictions_publisher;
  ros::Publisher m_hypotheses_box_evaluation_publisher;

  ros::Publisher m_likelihood_publisher;
};

}

#endif
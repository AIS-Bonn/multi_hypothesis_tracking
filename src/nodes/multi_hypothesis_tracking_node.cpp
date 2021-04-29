/** @file
 *
 * Multi hypothesis tracking node receiving detection messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/nodes/multi_hypothesis_tracking_node.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTrackingNode::MultiHypothesisTrackingNode()
{
  ros::NodeHandle private_node_handle("~");
  initializeHypothesisFactory(private_node_handle);
  
  m_object_detection_subscriber = private_node_handle.subscribe<multi_hypothesis_tracking_msgs::ObjectDetections>(
    m_input_topic, 1,
    &MultiHypothesisTrackingNode::detectionCallback,
    this);
}

void MultiHypothesisTrackingNode::initializeHypothesisFactory(const ros::NodeHandle& private_node_handle)
{
  auto hypothesis_factory = std::make_shared<HypothesisFactory>();

  float kalman_process_noise_covariance_per_second;
  private_node_handle.param<float>("kalman_process_noise_covariance_per_second", kalman_process_noise_covariance_per_second, 0.5f);
  hypothesis_factory->setKalmanProcessNoiseCovariancePerSecond(kalman_process_noise_covariance_per_second);

  float maximally_allowed_hypothesis_covariance;
  private_node_handle.param<float>("maximally_allowed_hypothesis_covariance", maximally_allowed_hypothesis_covariance, 5.f);
  hypothesis_factory->setMaxAllowedHypothesisCovariance(maximally_allowed_hypothesis_covariance);

  m_multi_hypothesis_tracker.setHypothesisFactory(hypothesis_factory);
}

void MultiHypothesisTrackingNode::detectionCallback(const DetectionsMsg::ConstPtr& detections_message)
{
  ROS_DEBUG_STREAM("Tracker::detectionCallback.");

  auto callback_start_time = std::chrono::high_resolution_clock::now();

  Detections detections;
  convert(detections_message, detections);

  if(!transformToFrame(detections, m_world_frame_id))
    return;

  processDetections(detections);

  updateProcessingTimeMeasurements(callback_start_time);
  publishVisualizations(detections);
}

void MultiHypothesisTrackingNode::convert(const DetectionsMsg::ConstPtr& detections_message,
                                          Detections& detections)
{
  detections.frame_id = detections_message->header.frame_id;
  detections.time_stamp = detections_message->header.stamp.toSec();

  for(size_t i = 0; i < detections_message->object_detections.size(); i++)
  {
    Detection detection;
    detection.position(0) = static_cast<float>(detections_message->object_detections[i].centroid.x);
    detection.position(1) = static_cast<float>(detections_message->object_detections[i].centroid.y);
    detection.position(2) = static_cast<float>(detections_message->object_detections[i].centroid.z);

    detection.covariance(0, 0) = detections_message->object_detections[i].position_covariance_xx;
    detection.covariance(0, 1) = detections_message->object_detections[i].position_covariance_xy;
    detection.covariance(0, 2) = detections_message->object_detections[i].position_covariance_xz;
    detection.covariance(1, 0) = detections_message->object_detections[i].position_covariance_xy;
    detection.covariance(1, 1) = detections_message->object_detections[i].position_covariance_yy;
    detection.covariance(1, 2) = detections_message->object_detections[i].position_covariance_yz;
    detection.covariance(2, 0) = detections_message->object_detections[i].position_covariance_xz;
    detection.covariance(2, 1) = detections_message->object_detections[i].position_covariance_yz;
    detection.covariance(2, 2) = detections_message->object_detections[i].position_covariance_zz;

    detection.points.clear();
    detection.points.reserve(
      detections_message->object_detections[i].cloud.height * detections_message->object_detections[i].cloud.width);
    for(sensor_msgs::PointCloud2ConstIterator<float> it(detections_message->object_detections[i].cloud, "x");
        it != it.end(); ++it)
      detection.points.push_back(Eigen::Vector3f(it[0], it[1], it[2]));

    detections.detections.push_back(detection);
  }
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_hypothesis_tracking");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  MultiHypothesisTracker::MultiHypothesisTrackingNode tracker;

  ros::spin();

  return 0;
}

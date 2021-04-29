/** @file
 *
 * Multi hypothesis tracking node receiving pose array messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/nodes/multi_hypothesis_tracking_node_for_pose_arrays.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTrackingNodeForPoseArrays::MultiHypothesisTrackingNodeForPoseArrays()
{
  ros::NodeHandle private_node_handle("~");
  initializeHypothesisFactory(private_node_handle);
  
  m_pose_array_subscriber = private_node_handle.subscribe<PoseArrayMsg>(m_input_topic, 
                                                                        1,
                                                                        &MultiHypothesisTrackingNodeForPoseArrays::detectionPosesCallback,
                                                                        this);
}

void MultiHypothesisTrackingNodeForPoseArrays::initializeHypothesisFactory(const ros::NodeHandle& private_node_handle)
{
  auto hypothesis_factory = std::make_shared<HypothesisFactory>();

  float kalman_process_noise_covariance_per_second;
  private_node_handle.param<float>("kalman_process_noise_covariance_per_second", kalman_process_noise_covariance_per_second, 0.5f);
  hypothesis_factory->setKalmanProcessNoiseCovariancePerSecond(kalman_process_noise_covariance_per_second);

  m_multi_hypothesis_tracker.setHypothesisFactory(hypothesis_factory);
}

void MultiHypothesisTrackingNodeForPoseArrays::detectionPosesCallback(const PoseArrayMsg::ConstPtr& detections_message)
{
  auto callback_start_time = std::chrono::high_resolution_clock::now();

  Detections detections;
  convert(detections_message, detections);

  if(!transformToFrame(detections, m_world_frame_id))
    return;

  processDetections(detections);

  updateProcessingTimeMeasurements(callback_start_time);
  publishVisualizations(detections);
}

void MultiHypothesisTrackingNodeForPoseArrays::convert(const PoseArrayMsg::ConstPtr& detections_message,
                                                       Detections& detections)
{
  detections.frame_id = detections_message->header.frame_id;
  detections.time_stamp = detections_message->header.stamp.toSec();

  for(size_t i = 0; i < detections_message->poses.size(); i++)
  {
    Detection detection;
    detection.position(0) = static_cast<float>(detections_message->poses[i].position.x);
    detection.position(1) = static_cast<float>(detections_message->poses[i].position.y);
    detection.position(2) = static_cast<float>(detections_message->poses[i].position.z);

    float detection_std = 0.03f;
    detection.covariance.setIdentity();
    detection.covariance(0, 0) = detection_std * detection_std;
    detection.covariance(1, 1) = detection_std * detection_std;
    detection.covariance(2, 2) = detection_std * detection_std;

    detection.points.clear();

    detections.detections.push_back(detection);
  }
}

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_hypothesis_tracking_for_pose_arrays");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  MultiHypothesisTracker::MultiHypothesisTrackingNodeForPoseArrays tracker;

  ros::spin();

  return 0;
}

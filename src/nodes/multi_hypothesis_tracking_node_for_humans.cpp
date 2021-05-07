/** @file
 *
 * Multi human tracking node receiving human messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/nodes/multi_hypothesis_tracking_node_for_humans.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTrackingNodeForHumans::MultiHypothesisTrackingNodeForHumans()
{
  ros::NodeHandle private_node_handle("~");
  initializeHypothesisFactory(private_node_handle);
  
  m_human_detection_subscriber = private_node_handle.subscribe<HumanMsg>(m_input_topic,
                                                                         1,
                                                                         &MultiHypothesisTrackingNodeForHumans::detectionCallback,
                                                                         this);
}

void MultiHypothesisTrackingNodeForHumans::initializeHypothesisFactory(const ros::NodeHandle& private_node_handle)
{
  auto hypothesis_factory = std::make_shared<HypothesisForHumanPoseFactory>();

  float kalman_process_noise_covariance_per_second;
  private_node_handle.param<float>("kalman_process_noise_covariance_per_second", kalman_process_noise_covariance_per_second, 0.5f);
  hypothesis_factory->setKalmanProcessNoiseCovariancePerSecond(kalman_process_noise_covariance_per_second);

  float maximally_allowed_hypothesis_covariance;
  private_node_handle.param<float>("maximally_allowed_hypothesis_covariance", maximally_allowed_hypothesis_covariance, 5.f);
  hypothesis_factory->setMaxAllowedHypothesisCovariance(maximally_allowed_hypothesis_covariance);
  
  m_multi_hypothesis_tracker.setHypothesisFactory(hypothesis_factory);
}

void MultiHypothesisTrackingNodeForHumans::detectionCallback(const HumanMsg::ConstPtr& detections_message)
{
  ROS_DEBUG_STREAM("MultiHumanTrackingNode::detectionCallback.");

  auto callback_start_time = std::chrono::high_resolution_clock::now();

  Detections detections;
  convert(detections_message, detections);

  if(!transformToFrame(detections, m_world_frame_id))
    return;

  processDetections(detections);
  
  updateProcessingTimeMeasurements(callback_start_time);
  publishVisualizations(detections);
}

void MultiHypothesisTrackingNodeForHumans::convert(const HumanMsg::ConstPtr& detections_message,
                                                   Detections& detections)
{
  detections.frame_id = detections_message->header.frame_id;
  detections.time_stamp = detections_message->header.stamp.toSec();

  float score_threshold = 0.1f;
  for(const auto& person_detection : detections_message->persons)
  {
    if((int)person_detection.keypoints.size() != 21)
      ROS_INFO_STREAM(
        "Person detection has " << (int)person_detection.keypoints.size() << " keypoints, but should have 21.");

    if(person_detection.keypoints[8].score < score_threshold)
    {
      ROS_INFO_STREAM(
        "Mid hip score is " << person_detection.keypoints[8].score << ", which is below the threshold of 0.1");
      continue;
    }

    Detection detection;
    detection.position(0) = static_cast<float>(person_detection.keypoints[8].joint.x);
    detection.position(1) = static_cast<float>(person_detection.keypoints[8].joint.y);
    detection.position(2) = static_cast<float>(person_detection.keypoints[8].joint.z);

    convert(person_detection.keypoints[8].cov, detection.covariance);

    detection.points.clear();
    for(const auto& joint : person_detection.keypoints)
      if(joint.score > 0.0)
      {
        detection.points.emplace_back(Eigen::Vector3f(static_cast<float>(joint.joint.x),
                                                      static_cast<float>(joint.joint.y),
                                                      static_cast<float>(joint.joint.z)));
        Eigen::Matrix3f joint_covariance;
        convert(joint.cov, joint_covariance);
        detection.points_covariances.push_back(joint_covariance);
      }
      else
      {
        detection.points.emplace_back(Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(),
                                                      std::numeric_limits<float>::quiet_NaN(),
                                                      std::numeric_limits<float>::quiet_NaN()));
        detection.points_covariances.emplace_back(Eigen::Matrix3f::Identity());
      }

    detections.detections.push_back(detection);
  }
}

void MultiHypothesisTrackingNodeForHumans::convert(const boost::array<double, 6>& covariance_msg,
                                                   Eigen::Matrix3f& covariance)
{
  covariance(0, 0) = static_cast<float>(covariance_msg[0]);
  covariance(0, 1) = static_cast<float>(covariance_msg[1]);
  covariance(0, 2) = static_cast<float>(covariance_msg[2]);
  covariance(1, 1) = static_cast<float>(covariance_msg[3]);
  covariance(1, 2) = static_cast<float>(covariance_msg[4]);
  covariance(2, 2) = static_cast<float>(covariance_msg[5]);
  // Mirror covariances to fill rest of matrix 
  covariance(1, 0) = covariance(0, 1);
  covariance(2, 0) = covariance(0, 2);
  covariance(2, 1) = covariance(1, 2);
}

}


int main(int argc,
         char** argv)
{
  ros::init(argc, argv, "multi_hypothesis_tracking_node_for_humans");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  MultiHypothesisTracker::MultiHypothesisTrackingNodeForHumans tracker;

  ros::spin();

  return 0;
}

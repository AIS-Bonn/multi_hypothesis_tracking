/** @file
 *
 * Multi human tracking node receiving human messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/multi_hypothesis_tracking_node_for_humans.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTrackingNodeForHumans::MultiHypothesisTrackingNodeForHumans()
{
  ros::NodeHandle private_node_handle("~");
  m_human_detection_subscriber = private_node_handle.subscribe<HumanMsg>(m_input_topic,
                                                                         1,
                                                                         &MultiHypothesisTrackingNodeForHumans::detectionCallback,
                                                                         this);
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

    float detection_std = 0.03f;
    detection.covariance.setIdentity();
    detection.covariance(0, 0) = detection_std * detection_std;
    detection.covariance(1, 1) = detection_std * detection_std;
    detection.covariance(2, 2) = detection_std * detection_std;

    detection.points.clear();
    for(const auto& joint : person_detection.keypoints)
      if(joint.score > 0.0)
        detection.points.emplace_back(Eigen::Vector3f(joint.joint.x,
                                                      joint.joint.y,
                                                      joint.joint.z));

    detections.detections.push_back(detection);
  }
}

}


int main(int argc,
         char** argv)
{
  ros::init(argc, argv, "multi_hypothesis_tracking_for_humans");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  MultiHypothesisTracker::MultiHypothesisTrackingNodeForHumans tracker;

  ros::spin();

  return 0;
}

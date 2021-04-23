/** @file
 *
 * Multi hypothesis tracking node receiving detection messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/multi_hypothesis_tracking_node.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTrackingNode::MultiHypothesisTrackingNode()
{
  ros::NodeHandle private_node_handle("~");
  bool subscribe_to_poses_only;
  private_node_handle.param<bool>("subscribe_to_poses_only", subscribe_to_poses_only, false);
  if(subscribe_to_poses_only)
    m_laser_detection_subscriber = private_node_handle.subscribe<geometry_msgs::PoseArray>(m_input_topic, 1,
                                                                                           &MultiHypothesisTrackingNode::detectionPosesCallback,
                                                                                           this);
  else
    m_laser_detection_subscriber = private_node_handle.subscribe<multi_hypothesis_tracking_msgs::ObjectDetections>(
      m_input_topic, 1,
      &MultiHypothesisTrackingNode::detectionCallback,
      this);
}

// TODO: merge with other callback after converting to detections
void MultiHypothesisTrackingNode::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Laser detection callback.");

//  double start = getTimeHighRes();

  Detections detections;
  convert(msg, detections);

  if(!transformToFrame(detections, m_world_frame_id))
    return;

  processDetections(detections);

//  std::cout << std::setprecision(10) << "\n####time for one callback " << (getTimeHighRes() - start) << " " << std::endl;
  publishVisualizations(detections);
}

void MultiHypothesisTrackingNode::detectionCallback(const multi_hypothesis_tracking_msgs::ObjectDetections::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Tracker::detectionCallback.");

  auto callback_start_time = std::chrono::high_resolution_clock::now();

  Detections detections;
  convert(msg, detections);

  if(!transformToFrame(detections, m_world_frame_id))
    return;

  processDetections(detections);

  updateProcessingTimeMeasurements(callback_start_time);
  publishVisualizations(detections);
}

void MultiHypothesisTrackingNode::convert(const geometry_msgs::PoseArray::ConstPtr& msg,
                                          Detections& detections)
{
  detections.frame_id = msg->header.frame_id;
  detections.time_stamp = msg->header.stamp.toSec();

  for(size_t i = 0; i < msg->poses.size(); i++)
  {
    Detection detection;
    detection.position(0) = static_cast<float>(msg->poses[i].position.x);
    detection.position(1) = static_cast<float>(msg->poses[i].position.y);
    detection.position(2) = static_cast<float>(msg->poses[i].position.z);

    float detection_std = 0.03f;
    detection.covariance.setIdentity();
    detection.covariance(0, 0) = detection_std * detection_std;
    detection.covariance(1, 1) = detection_std * detection_std;
    detection.covariance(2, 2) = detection_std * detection_std;

    detection.points.clear();

    detections.detections.push_back(detection);
  }
}

void MultiHypothesisTrackingNode::convert(const multi_hypothesis_tracking_msgs::ObjectDetections::ConstPtr& msg,
                                          Detections& detections)
{
  detections.frame_id = msg->header.frame_id;
  detections.time_stamp = msg->header.stamp.toSec();

  for(size_t i = 0; i < msg->object_detections.size(); i++)
  {
    Detection detection;
    detection.position(0) = static_cast<float>(msg->object_detections[i].centroid.x);
    detection.position(1) = static_cast<float>(msg->object_detections[i].centroid.y);
    detection.position(2) = static_cast<float>(msg->object_detections[i].centroid.z);

    detection.covariance(0, 0) = msg->object_detections[i].position_covariance_xx;
    detection.covariance(0, 1) = msg->object_detections[i].position_covariance_xy;
    detection.covariance(0, 2) = msg->object_detections[i].position_covariance_xz;
    detection.covariance(1, 0) = msg->object_detections[i].position_covariance_xy;
    detection.covariance(1, 1) = msg->object_detections[i].position_covariance_yy;
    detection.covariance(1, 2) = msg->object_detections[i].position_covariance_yz;
    detection.covariance(2, 0) = msg->object_detections[i].position_covariance_xz;
    detection.covariance(2, 1) = msg->object_detections[i].position_covariance_yz;
    detection.covariance(2, 2) = msg->object_detections[i].position_covariance_zz;

    detection.points.clear();
    detection.points.reserve(msg->object_detections[i].cloud.height * msg->object_detections[i].cloud.width);
    for(sensor_msgs::PointCloud2ConstIterator<float> it(msg->object_detections[i].cloud, "x"); it != it.end(); ++it)
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

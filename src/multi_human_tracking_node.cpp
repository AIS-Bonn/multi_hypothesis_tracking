/** @file
 *
 * Multi human tracking node receiving human messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/multi_human_tracking_node.h>

namespace MultiHypothesisTracker
{

Tracker::Tracker()
  : m_multi_hypothesis_tracker(std::make_shared<HypothesisFactory>())
    , m_last_prediction_time(0)
    , m_measure_time(false)
    , m_number_of_callbacks(0)
    , m_got_first_detections(false)
{
  ros::NodeHandle private_node_handle("~");

  m_transform_listener = std::make_shared<tf::TransformListener>();

  private_node_handle.param<std::string>("world_frame", m_world_frame, "world");

  private_node_handle.param<double>("merge_close_hypotheses_distance", m_merge_distance, 0.1);
  private_node_handle.param<float>("max_covariance", m_max_covariance, 5.f);

  double max_correspondence_distance;
  private_node_handle.param<double>("max_correspondence_distance", max_correspondence_distance, 3.75);
  m_multi_hypothesis_tracker.setMaxCorrespondenceDistance(max_correspondence_distance);

  double kalman_covariance_per_second;
  private_node_handle.param<double>("kalman_covariance_per_second", kalman_covariance_per_second, 0.5);
  m_multi_hypothesis_tracker.setKalmanCovariancePerSecond(kalman_covariance_per_second);

  private_node_handle.param<bool>("compute_likelihood", m_compute_likelihood, false);
  m_multi_hypothesis_tracker.setComputeLikelihood(m_compute_likelihood);

  private_node_handle.getParam("measure_time", m_measure_time);
  if(m_measure_time)
  {
    std::string path_to_results_file = "/tmp/times_multi_hypothesis_tracking";
    m_time_file.open(path_to_results_file);
  }
  m_summed_time_for_callbacks = std::chrono::microseconds::zero();

  std::string input_topic;
  private_node_handle.param<std::string>("input_topic", input_topic, "/object_poses");
  m_human_detection_subscriber = private_node_handle.subscribe<HumanMsg>(input_topic, 1,
                                                                         &Tracker::detectionCallback,
                                                                         this);
}

void Tracker::publish(const ros::Time& stamp)
{
  m_mot_publisher.publishAll(getHypotheses(), stamp);
  m_mot_publisher.publishDeletedHypotheses(getDeletedHypotheses());
  if(m_compute_likelihood)
    m_mot_publisher.publishLikelihood(m_multi_hypothesis_tracker.getAverageLikelihood());
}

void Tracker::detectionCallback(const HumanMsg::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Tracker::detectionCallback.");

  auto callback_start_time = std::chrono::high_resolution_clock::now();

  std::vector <Detection> detections;
  convert(msg, detections);

  if(!transformToFrame(detections, msg->header, m_world_frame))
    return;

  m_mot_publisher.publishDetectionPositions(detections, msg->header.stamp);
  m_mot_publisher.publishDetectionsCovariances(detections, msg->header.stamp);

  processDetections(detections);

  if(!m_got_first_detections && !msg->persons.empty())
    m_got_first_detections = true;

  if(m_measure_time && m_got_first_detections)
  {
    std::chrono::microseconds time_for_one_callback = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - callback_start_time);
    ROS_DEBUG_STREAM("Time for tracking for one cloud: " << time_for_one_callback.count() << " microseconds.");
    m_time_file << (double)time_for_one_callback.count() / 1000.0 << std::endl;
    m_summed_time_for_callbacks += time_for_one_callback;
    m_number_of_callbacks++;
    ROS_DEBUG_STREAM(
      "Mean time for tracking for one cloud: " << m_summed_time_for_callbacks.count() / m_number_of_callbacks
                                               << " microseconds.");
  }

  publish(msg->header.stamp);
}

void Tracker::convert(const HumanMsg::ConstPtr& msg,
                      std::vector <Detection>& detections)
{
  Detection detection;
  detection.frame = msg->header.frame_id;
  detection.time = msg->header.stamp.toSec();

  float score_threshold = 0.1f;
  for(const auto& person_detection : msg->persons)
  {
    if((int)person_detection.keypoints.size() != 21)
      ROS_INFO_STREAM("Person detection has " << (int)person_detection.keypoints.size() << " keypoints, but should have 21.");

    if(person_detection.keypoints[8].score < score_threshold)
    {
      ROS_INFO_STREAM("Mid hip score is " << person_detection.keypoints[8].score << ", which is below the threshold of 0.1");
      continue;
    }
    
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
    
    detection.class_a_detection = true;

    detections.push_back(detection);
  }
}

bool Tracker::transformToFrame(std::vector <Detection>& detections,
                               const std_msgs::Header& header,
                               const std::string& target_frame)
{
  if(target_frame == header.frame_id)
    return true;

  if(!m_transform_listener->waitForTransform(target_frame, header.frame_id, header.stamp, ros::Duration(1.0)))
  {
    ROS_ERROR_STREAM("Could not wait for transform at time " << header.stamp);
    return false;
  }

  tf::StampedTransform transform;
  try
  {
    m_transform_listener->lookupTransform(target_frame, header.frame_id, header.stamp, transform);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\"", header.frame_id.c_str(),
              target_frame.c_str());
    return false;
  }

  for(auto& detection : detections)
  {
    Eigen::Affine3d transform_eigen;
    tf::transformTFToEigen(transform, transform_eigen);
    Eigen::Affine3f transform_eigenf = transform_eigen.cast<float>();

    detection.position = transform_eigenf * detection.position;

    for(auto& point : detection.points)
      point = transform_eigenf * point;
    
    detection.frame = target_frame;
  }

  return true;
}

void Tracker::processDetections(const std::vector <Detection>& detections)
{
  if(detections.empty())
    return;

  // Prediction step of kalman filter for all hypotheses
  if(m_last_prediction_time > 0)
    m_multi_hypothesis_tracker.predict(detections.at(0).time - m_last_prediction_time);

  m_last_prediction_time = detections.at(0).time;

  // Correction step of kalman filter for all hypotheses
  m_multi_hypothesis_tracker.correct(detections);

  // Filter out weak hypotheses
  m_multi_hypothesis_tracker.clearDeletedHypotheses();
  m_multi_hypothesis_tracker.deleteSpuriousHypotheses(m_max_covariance);
  m_multi_hypothesis_tracker.mergeCloseHypotheses(m_merge_distance);
}

const std::vector <std::shared_ptr<Hypothesis>>& Tracker::getHypotheses()
{
  return m_multi_hypothesis_tracker.getHypotheses();
}

std::queue <Hypothesis>& Tracker::getDeletedHypotheses()
{
  return m_multi_hypothesis_tracker.getDeletedHypotheses();
}

}


int main(int argc,
         char** argv)
{
  ros::init(argc, argv, "multi_human_tracking");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  MultiHypothesisTracker::Tracker tracker;

  ros::spin();

  return 0;
}

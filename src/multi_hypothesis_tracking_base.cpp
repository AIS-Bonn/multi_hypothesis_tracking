/** @file
 *
 * Multi hypothesis tracking node receiving detection messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/multi_hypothesis_tracking_base.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTrackingBase::MultiHypothesisTrackingBase()
  : m_multi_hypothesis_tracker(std::make_shared<HypothesisFactory>())
    , m_last_prediction_time(0)
{
  getRosParameters();
  m_transform_listener = std::make_shared<tf::TransformListener>();

  if(m_measure_time)
    prepareMeasuringProcessingTime();
}

void MultiHypothesisTrackingBase::getRosParameters()
{
  ros::NodeHandle private_node_handle("~");

  private_node_handle.param<std::string>("input_topic", m_input_topic, "/object_poses");
  private_node_handle.param<std::string>("world_frame_id", m_world_frame, "world");

  private_node_handle.param<double>("merge_close_hypotheses_distance", m_merge_distance, 0.1);
  private_node_handle.param<float>("max_covariance", m_max_covariance, 5.f);
  
  private_node_handle.param<bool>("measure_time", m_measure_time, false);

  bool use_bhattacharyya_distance;
  private_node_handle.param<bool>("use_bhattacharyya_distance", use_bhattacharyya_distance, true);
  m_multi_hypothesis_tracker.setUseBhattacharyyaDistance(use_bhattacharyya_distance);

  double max_correspondence_distance;
  private_node_handle.param<double>("max_correspondence_distance", max_correspondence_distance, 3.75);
  m_multi_hypothesis_tracker.setMaxCorrespondenceDistance(max_correspondence_distance);

  double kalman_covariance_per_second;
  private_node_handle.param<double>("kalman_covariance_per_second", kalman_covariance_per_second, 0.5);
  m_multi_hypothesis_tracker.setKalmanCovariancePerSecond(kalman_covariance_per_second);

  private_node_handle.param<bool>("compute_likelihood", m_compute_likelihood, false);
  m_multi_hypothesis_tracker.setComputeLikelihood(m_compute_likelihood);
}

void MultiHypothesisTrackingBase::prepareMeasuringProcessingTime()
{
  std::string path_to_results_file = "/tmp/times_multi_hypothesis_tracking";
  m_time_file.open(path_to_results_file);
  m_number_of_callbacks = 0;
  m_summed_time_for_callbacks = std::chrono::microseconds::zero();
  m_got_first_detections = false;
}

void MultiHypothesisTrackingBase::publish(const ros::Time& stamp)
{
  m_visualizations_publisher.publishAll(getHypotheses(), stamp);
  if(m_compute_likelihood)
    m_visualizations_publisher.publishLikelihood(m_multi_hypothesis_tracker.getAverageLikelihood());
}

bool MultiHypothesisTrackingBase::transformToFrame(std::vector<Detection>& detections,
                                                   const std_msgs::Header& header,
                                                   const std::string& target_frame)
{
  if(target_frame == header.frame_id)
    return true;

  tf::StampedTransform transform;
  if(!getTransform(header.frame_id, 
                   target_frame, 
                   header.stamp, 
                   transform))
    return false;

  transformDetections(detections, transform, target_frame);
  return true;
}

bool MultiHypothesisTrackingBase::getTransform(const std::string& source_frame,
                                               const std::string& target_frame,
                                               const ros::Time& time_stamp,
                                               tf::StampedTransform& transform)
{
  if(!m_transform_listener->waitForTransform(target_frame, 
                                             source_frame, 
                                             time_stamp, 
                                             ros::Duration(1.0)))
  {
    ROS_ERROR_STREAM("Could not wait for transform at time " << time_stamp);
    return false;
  }

  try
  {
    m_transform_listener->lookupTransform(target_frame, 
                                          source_frame, 
                                          time_stamp, 
                                          transform);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\"", 
              source_frame.c_str(),
              target_frame.c_str());
    return false;
  }
  return true;
}

void MultiHypothesisTrackingBase::transformDetections(std::vector<Detection>& detections,
                                                      const tf::StampedTransform& transform,
                                                      const std::string& target_frame)
{
  Eigen::Affine3d transform_eigen;
  tf::transformTFToEigen(transform, transform_eigen);
  Eigen::Affine3f transform_eigenf = transform_eigen.cast<float>();
  
  for(auto& detection : detections)
  {
    detection.position = transform_eigenf * detection.position;

    for(auto& point : detection.points)
      point = transform_eigenf * point;

    detection.frame_id = target_frame;
  }
}

void MultiHypothesisTrackingBase::processDetections(const std::vector<Detection>& detections)
{
  if(detections.empty())
    return;

  double time_since_last_detections = detections.at(0).time_stamp - m_last_prediction_time;
  if(time_since_last_detections <= 0.0)
    return;

  // Prediction step of kalman filter for all hypotheses
  if(m_last_prediction_time > 0.0)
    m_multi_hypothesis_tracker.predict(time_since_last_detections);

  m_last_prediction_time = detections.at(0).time_stamp;

  // Correction step of kalman filter for all hypotheses
  m_multi_hypothesis_tracker.correct(detections);

  // Filter out weak hypotheses
  m_multi_hypothesis_tracker.deleteSpuriousHypotheses(m_max_covariance);
  m_multi_hypothesis_tracker.mergeCloseHypotheses(m_merge_distance);
}

const std::vector<std::shared_ptr<Hypothesis>>& MultiHypothesisTrackingBase::getHypotheses()
{
  return m_multi_hypothesis_tracker.getHypotheses();
}

}




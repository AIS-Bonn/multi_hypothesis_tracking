/** @file
 *
 * Multi object tracker implementation.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/multi_hypothesis_tracking_node.h>

namespace MultiHypothesisTracker
{

Tracker::Tracker()
  : m_multi_hypothesis_tracker(std::make_shared<HypothesisFactory>())
    , m_last_prediction_time(0)
    , m_measure_time(false)
    , m_number_of_callbacks(0)
    , m_got_first_detections(false)
{
  ros::NodeHandle n("~");
  ros::NodeHandle pub_n;

  m_transform_listener = std::make_shared<tf::TransformListener>();

  std::string input_topic;
  n.param<std::string>("input_topic", input_topic, "/object_poses");

  n.param<std::string>("world_frame", m_world_frame, "world");

  n.param<double>("merge_close_hypotheses_distance", m_merge_distance, 0.1);
  n.param<float>("max_covariance", m_max_covariance, 5.f);

  double max_correspondence_distance;
  n.param<double>("max_correspondence_distance", max_correspondence_distance, 3.75);
  m_multi_hypothesis_tracker.setMaxCorrespondenceDistance(max_correspondence_distance);

  double kalman_covariance_per_second;
  n.param<double>("kalman_covariance_per_second", kalman_covariance_per_second, 0.5);
  m_multi_hypothesis_tracker.setKalmanCovariancePerSecond(kalman_covariance_per_second);

  n.param<bool>("compute_likelihood", m_compute_likelihood, false);
  m_multi_hypothesis_tracker.setComputeLikelihood(m_compute_likelihood);

  bool subscribe_to_poses_only;
  n.param<bool>("subscribe_to_poses_only", subscribe_to_poses_only, false);

  n.getParam("measure_time", m_measure_time);
  if(m_measure_time)
  {
    std::string path_to_results_file = "/tmp/times_multi_hypothesis_tracking";
    m_time_file.open(path_to_results_file);
  }
  m_summed_time_for_callbacks = std::chrono::microseconds::zero();

  if(subscribe_to_poses_only)
    m_laser_detection_subscriber = n.subscribe<geometry_msgs::PoseArray>(input_topic, 1,
                                                                         &Tracker::detectionPosesCallback, this);
  else
    m_laser_detection_subscriber = n.subscribe<multi_hypothesis_tracking_msgs::ObjectDetections>(input_topic, 1,
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

// TODO: merge with other callback after converting to measurements
void Tracker::detectionPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Laser detection callback.");

//  double start = getTimeHighRes();

  std::vector <Measurement> measurements;
  convert(msg, measurements);

  if(!transformToFrame(measurements, msg->header, m_world_frame))
    return;

  m_mot_publisher.publishMeasurementPositions(measurements, msg->header.stamp);
  m_mot_publisher.publishMeasurementsCovariances(measurements, msg->header.stamp);

  processMeasurements(measurements);

//  std::cout << std::setprecision(10) << "\n####time for one callback " << (getTimeHighRes() - start) << " " << std::endl;
  publish(msg->header.stamp);
}

void Tracker::detectionCallback(const multi_hypothesis_tracking_msgs::ObjectDetections::ConstPtr& msg)
{
  ROS_DEBUG_STREAM("Tracker::detectionCallback.");

  auto callback_start_time = std::chrono::high_resolution_clock::now();

  std::vector <Measurement> measurements;
  convert(msg, measurements);

  if(!transformToFrame(measurements, msg->header, m_world_frame))
    return;

  m_mot_publisher.publishMeasurementPositions(measurements, msg->header.stamp);
  m_mot_publisher.publishMeasurementsCovariances(measurements, msg->header.stamp);
  m_mot_publisher.publishMeasurementsPoints(measurements, msg->header.stamp);

  processMeasurements(measurements);

  if(!m_got_first_detections && !msg->object_detections.empty())
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

void Tracker::convert(const geometry_msgs::PoseArray::ConstPtr& msg,
                      std::vector <Measurement>& measurements)
{
  Measurement measurement;
  measurement.frame = msg->header.frame_id;
  measurement.time = msg->header.stamp.toSec();

  for(size_t i = 0; i < msg->poses.size(); i++)
  {
    measurement.pos(0) = static_cast<float>(msg->poses[i].position.x);
    measurement.pos(1) = static_cast<float>(msg->poses[i].position.y);
    measurement.pos(2) = static_cast<float>(msg->poses[i].position.z);

    float measurement_std = 0.03f;
    measurement.cov.setIdentity();
    measurement.cov(0, 0) = measurement_std * measurement_std;
    measurement.cov(1, 1) = measurement_std * measurement_std;
    measurement.cov(2, 2) = measurement_std * measurement_std;

    measurement.points.clear();
    measurement.point_ids.clear();
    measurement.class_a_detection = true;

    measurements.push_back(measurement);
  }
}

void Tracker::convert(const multi_hypothesis_tracking_msgs::ObjectDetections::ConstPtr& msg,
                      std::vector <Measurement>& measurements)
{
  Measurement measurement;
  measurement.frame = msg->header.frame_id;
  measurement.time = msg->header.stamp.toSec();

  for(size_t i = 0; i < msg->object_detections.size(); i++)
  {
    measurement.pos(0) = static_cast<float>(msg->object_detections[i].centroid.x);
    measurement.pos(1) = static_cast<float>(msg->object_detections[i].centroid.y);
    measurement.pos(2) = static_cast<float>(msg->object_detections[i].centroid.z);

    measurement.cov(0, 0) = msg->object_detections[i].position_covariance_xx;
    measurement.cov(0, 1) = msg->object_detections[i].position_covariance_xy;
    measurement.cov(0, 2) = msg->object_detections[i].position_covariance_xz;
    measurement.cov(1, 0) = msg->object_detections[i].position_covariance_xy;
    measurement.cov(1, 1) = msg->object_detections[i].position_covariance_yy;
    measurement.cov(1, 2) = msg->object_detections[i].position_covariance_yz;
    measurement.cov(2, 0) = msg->object_detections[i].position_covariance_xz;
    measurement.cov(2, 1) = msg->object_detections[i].position_covariance_yz;
    measurement.cov(2, 2) = msg->object_detections[i].position_covariance_zz;

    measurement.points.clear();
    measurement.points.reserve(msg->object_detections[i].cloud.height * msg->object_detections[i].cloud.width);
    for(sensor_msgs::PointCloud2ConstIterator<float> it(msg->object_detections[i].cloud, "x"); it != it.end(); ++it)
      measurement.points.push_back(Eigen::Vector3f(it[0], it[1], it[2]));

    measurement.point_ids.clear();
    measurement.point_ids.reserve(msg->object_detections[i].point_ids.size());
    for(const auto& idx : msg->object_detections[i].point_ids)
      measurement.point_ids.push_back(idx);

    measurement.class_a_detection = msg->object_detections[i].class_a_detection;

    measurements.push_back(measurement);
  }
}

bool Tracker::transformToFrame(std::vector <Measurement>& measurements,
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

  for(auto& measurement : measurements)
  {
    Eigen::Affine3d transform_eigen;
    tf::transformTFToEigen(transform, transform_eigen);
    Eigen::Affine3f transform_eigenf = transform_eigen.cast<float>();

    measurement.pos = transform_eigenf * measurement.pos;

    for(auto& point : measurement.points)
      point = transform_eigenf * point;

    measurement.frame = target_frame;
  }

  return true;
}

void Tracker::processMeasurements(const std::vector <Measurement>& measurements)
{
  if(measurements.empty())
    return;

  // Prediction step of kalman filter for all hypotheses
  if(m_last_prediction_time > 0)
    m_multi_hypothesis_tracker.predict(measurements.at(0).time - m_last_prediction_time);

  m_last_prediction_time = measurements.at(0).time;

  // Correction step of kalman filter for all hypotheses
  m_multi_hypothesis_tracker.correct(measurements);

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
  ros::init(argc, argv, "multi_hypothesis_tracking");

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    ros::console::notifyLoggerLevelsChanged();

  MultiHypothesisTracker::Tracker tracker;

  ros::spin();

  return 0;
}

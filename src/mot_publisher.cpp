/** @file
 *
 * Class to publish data from the multi object tracker.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/mot_publisher.h>

namespace MultiHypothesisTracker
{

MOTPublisher::MOTPublisher()
  : m_debug_counter(0)
{
  ros::NodeHandle n("~");
  ros::NodeHandle pub_n;

  m_hypotheses_full_publisher = pub_n.advertise<multi_hypothesis_tracking_msgs::HypothesesFull>("hypotheses_full", 1);
  m_hypotheses_predictions_publisher = pub_n.advertise<multi_hypothesis_tracking_msgs::ObjectDetections>("hypotheses_predictions", 1);

  m_measurement_positions_publisher = n.advertise<visualization_msgs::Marker>(
    n.getNamespace() + "/measurements_positions", 1);
  m_measurements_covariances_publisher = n.advertise<visualization_msgs::Marker>(
    n.getNamespace() + "/measurements_covariances", 1);
  m_measurements_points_publisher =
    n.advertise < pcl::PointCloud < pcl::PointXYZ >> (n.getNamespace() + "/measurements_points", 1);
  m_hypotheses_positions_publisher = n.advertise<visualization_msgs::Marker>(n.getNamespace() + "/hypotheses_positions",
                                                                             1);
  m_hypotheses_points_publisher =
    n.advertise < pcl::PointCloud < pcl::PointXYZ >> (n.getNamespace() + "/hypotheses_points", 1);
  m_track_line_publisher = n.advertise<visualization_msgs::Marker>(n.getNamespace() + "/track_line", 1);
  m_hypotheses_covariance_publisher = n.advertise<visualization_msgs::Marker>(
    n.getNamespace() + "/hypotheses_covariances", 1);
  m_static_hypotheses_positions_publisher = n.advertise<visualization_msgs::Marker>(
    n.getNamespace() + "/static_hypotheses_positions", 1);
  m_dynamic_hypotheses_positions_publisher = n.advertise<visualization_msgs::Marker>(
    n.getNamespace() + "/dynamic_hypotheses_positions", 1);
  m_hypotheses_predicted_positions_publisher = n.advertise<visualization_msgs::Marker>(
    n.getNamespace() + "/hypotheses_predicted_positions", 1);
  m_hypotheses_point_indices_publisher = n.advertise<multi_hypothesis_tracking_msgs::HypothesesPointIndices>(
    n.getNamespace() + "/hypotheses_point_indices", 1, true);
  m_hypotheses_box_evaluation_publisher = n.advertise<multi_hypothesis_tracking_msgs::HypothesesEvaluationBoxes>(
    n.getNamespace() + "/hypotheses_boxes_evaluation", 1, true);
  m_hypotheses_boxes_history_publisher = n.advertise<multi_hypothesis_tracking_msgs::HypothesesBoxesArray>(
    n.getNamespace() + "/hypotheses_boxes_history", 1, true);
  m_likelihood_publisher = n.advertise<std_msgs::Float32>(n.getNamespace() + "/likelihood", 1);

  n.param<std::string>("world_frame", m_world_frame, "world");
  n.param<double>("born_time_threshold", m_born_time_threshold, 0.5);
  n.param<double>("future_time", m_future_time, 0.0);
}

void MOTPublisher::publishAll(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                              const ros::Time& stamp)
{
  if(hypotheses.empty())
    ROS_DEBUG_STREAM("Publishing empty hypotheses.");

  publishHypothesesPositions(hypotheses, stamp);
  publishHypothesesPoints(hypotheses, stamp);
  publishHypothesesFull(hypotheses, stamp);
  publishHypothesesPredictions(hypotheses, stamp);
  publishHypothesesPredictedPositions(hypotheses, stamp);
  publishHypothesesCovariances(hypotheses, stamp);
  publishStaticHypothesesPositions(hypotheses, stamp);
  publishDynamicHypothesesPositions(hypotheses, stamp);
  publishHypothesesBoxesEvaluation(hypotheses, stamp);
  publishHypothesesPointIndices(hypotheses, stamp);

  publishHypothesesBoxesHistory(hypotheses, stamp);

//  publishFullTracks(hypotheses);
//  publishDebug(hypotheses);
}

visualization_msgs::Marker MOTPublisher::createMarker(float r, float g, float b, std::string ns)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = m_world_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.lifetime = ros::Duration(1.0);
  return marker;
}

void MOTPublisher::publishMeasurementPositions(const std::vector <Measurement>& measurements,
                                               const ros::Time& stamp)
{
  if(m_measurement_positions_publisher.getNumSubscribers() == 0 || measurements.empty())
    return;

  visualization_msgs::Marker measurement_positions_marker = createMarker(1.0, 0.0, 0.0,
                                                                         "mot_measurements_markers"); //red marker
  measurement_positions_marker.header.frame_id = measurements.at(0).frame;
  measurement_positions_marker.header.stamp = stamp;

  measurement_positions_marker.points.resize(measurements.size());
  for(size_t i = 0; i < measurements.size(); i++)
  {
    measurement_positions_marker.points[i].x = measurements[i].pos(0);
    measurement_positions_marker.points[i].y = measurements[i].pos(1);
    measurement_positions_marker.points[i].z = measurements[i].pos(2);
  }
  m_measurement_positions_publisher.publish(measurement_positions_marker);
}

void MOTPublisher::publishMeasurementsCovariances(const std::vector <Measurement>& measurements,
                                                  const ros::Time& stamp)
{
  if(m_measurements_covariances_publisher.getNumSubscribers() == 0 || measurements.empty())
    return;

  visualization_msgs::Marker measurement_cov_marker = createMarker(1.0, 0.0, 0.0, "mot_measurement_covariance_marker");;
  measurement_cov_marker.type = visualization_msgs::Marker::SPHERE;
  measurement_cov_marker.color.a = 0.5f;
  measurement_cov_marker.header.stamp = stamp;

  for(size_t i = 0; i < measurements.size(); i++)
  {
    measurement_cov_marker.header.frame_id = measurements.at(i).frame;
    measurement_cov_marker.id = (int)i;
    measurement_cov_marker.pose.position.x = measurements[i].pos(0);
    measurement_cov_marker.pose.position.y = measurements[i].pos(1);
    measurement_cov_marker.pose.position.z = measurements[i].pos(2);

    measurement_cov_marker.scale.x = sqrt(4.204) * sqrt(measurements[i].cov(0, 0));
    measurement_cov_marker.scale.y = sqrt(4.204) * sqrt(measurements[i].cov(1, 1));
    measurement_cov_marker.scale.z = sqrt(4.204) * sqrt(measurements[i].cov(2, 2));

    m_measurements_covariances_publisher.publish(measurement_cov_marker);
  }
}

void MOTPublisher::publishMeasurementsPoints(const std::vector <Measurement>& measurements,
                                             const ros::Time& stamp)
{
  if(m_measurements_points_publisher.getNumSubscribers() == 0 || measurements.empty())
    return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
  cloud->header.frame_id = measurements.at(0).frame;
  cloud->header.stamp = pcl_conversions::toPCL(stamp);

  int total_points_count = 0;
  for(size_t i = 0; i < measurements.size(); i++)
    total_points_count += measurements.at(i).points.size();

  cloud->points.resize(total_points_count);
  int point_counter = 0;
  for(size_t i = 0; i < measurements.size(); i++)
    for(size_t point_id = 0; point_id < measurements.at(i).points.size(); point_id++, point_counter++)
      cloud->points[point_counter].getVector3fMap() = measurements.at(i).points.at(point_id);

  m_measurements_points_publisher.publish(cloud);
}

void MOTPublisher::publishHypothesesCovariances(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                                const ros::Time& stamp)
{
  if(m_hypotheses_covariance_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  double current_time = getTimeHighRes();

  visualization_msgs::Marker hyp_covariance_marker = createMarker(1.0, 0.0, 0.0, "mot_hypotheses_covariance_marker");;
  hyp_covariance_marker.type = visualization_msgs::Marker::SPHERE;
  hyp_covariance_marker.color.a = 0.5f;
  hyp_covariance_marker.header.stamp = stamp;

  for(size_t i = 0; i < hypotheses.size(); i++)
  {
    hyp_covariance_marker.id = (int)i;
    hyp_covariance_marker.pose.position.x = hypotheses[i]->getPosition()(0);
    hyp_covariance_marker.pose.position.y = hypotheses[i]->getPosition()(1);
    hyp_covariance_marker.pose.position.z = hypotheses[i]->getPosition()(2);

    hyp_covariance_marker.scale.x = sqrt(4.204) * sqrt(hypotheses[i]->getCovariance()(0, 0));
    hyp_covariance_marker.scale.y = sqrt(4.204) * sqrt(hypotheses[i]->getCovariance()(1, 1));
    hyp_covariance_marker.scale.z = sqrt(4.204) * sqrt(hypotheses[i]->getCovariance()(2, 2));

    if(current_time - hypotheses[i]->getBornTime() >= m_born_time_threshold)
      m_hypotheses_covariance_publisher.publish(hyp_covariance_marker);
  }
}


void MOTPublisher::publishHypothesesPositions(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                              const ros::Time& stamp)
{
  if(m_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  visualization_msgs::Marker hypothesis_marker = createMarker(0.0, 1.0, 0.0, "mot_hypotheses_positions_markers");
  hypothesis_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr <Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    const Eigen::Vector3f& mean = hypothesis->getPosition();
    geometry_msgs::Point p;
    p.x = mean(0);
    p.y = mean(1);
    p.z = mean(2);

    if(current_time - hypothesis->getBornTime() >= m_born_time_threshold)
      hypothesis_marker.points.push_back(p);
  }
  m_hypotheses_positions_publisher.publish(hypothesis_marker);
}

void MOTPublisher::publishHypothesesPoints(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                           const ros::Time& stamp)
{
  if(m_hypotheses_points_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
  cloud->header.frame_id = m_world_frame;
  cloud->header.stamp = pcl_conversions::toPCL(stamp);

  int total_points_count = 0;
  int min_cloud_size = std::numeric_limits<int>::max();
  int max_cloud_size = 0;
  for(size_t i = 0; i < hypotheses.size(); i++)
  {
    total_points_count += hypotheses.at(i)->getPointCloud().size();
    min_cloud_size = std::min(min_cloud_size, (int)hypotheses.at(i)->getPointCloud().size());
    max_cloud_size = std::max(max_cloud_size, (int)hypotheses.at(i)->getPointCloud().size());
  }

  std::cout << " number of hypotheses " << (int)hypotheses.size() << std::endl;
  std::cout << " total points count " << total_points_count << std::endl;
  std::cout << " min_cloud_size " << min_cloud_size << std::endl;
  std::cout << " max_cloud_size " << max_cloud_size << std::endl;

  cloud->points.resize(total_points_count);
  int point_counter = 0;
  for(size_t i = 0; i < hypotheses.size(); i++)
    for(size_t point_id = 0; point_id < hypotheses.at(i)->getPointCloud().size(); point_id++, point_counter++)
      cloud->points[point_counter].getVector3fMap() = hypotheses.at(i)->getPointCloud().at(point_id);

  m_hypotheses_points_publisher.publish(cloud);
}

void MOTPublisher::publishHypothesesFull(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                         const ros::Time& stamp)
{
  if(m_hypotheses_full_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  multi_hypothesis_tracking_msgs::HypothesesFullPtr hypotheses_msg(
    new multi_hypothesis_tracking_msgs::HypothesesFull());
  hypotheses_msg->header.frame_id = m_world_frame;
  hypotheses_msg->header.stamp = stamp;

  multi_hypothesis_tracking_msgs::State state;
  multi_hypothesis_tracking_msgs::Box box;

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr <Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    // TODO: add something like confidence to message to be able to seperate hypotheses that were just assigned from those that are tracked for a while

    // fill state
    state.id = hypothesis->getID();

    const Eigen::Vector3f pos = hypothesis->getPosition();
    state.position.x = pos.x();
    state.position.y = pos.y();
    state.position.z = pos.z();

    const Eigen::Vector3f vel = hypothesis->getVelocity();
    state.velocity.x = vel.x();
    state.velocity.y = vel.y();
    state.velocity.z = vel.z();

    hypotheses_msg->states.push_back(state);

    // fill box
    box.id = hypothesis->getID();

    const Eigen::Array3f& min_box = hypothesis->getMinBoxDetection();
    box.min_corner.x = min_box(0);
    box.min_corner.y = min_box(1);
    box.min_corner.z = min_box(2);

    const Eigen::Array3f& max_box = hypothesis->getMaxBoxDetection();
    box.max_corner.x = max_box(0);
    box.max_corner.y = max_box(1);
    box.max_corner.z = max_box(2);

    hypotheses_msg->boxes.push_back(box);
  }
  m_hypotheses_full_publisher.publish(hypotheses_msg);
}

void MOTPublisher::publishHypothesesPredictions(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                                const ros::Time& stamp)
{
  if(m_hypotheses_predictions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  double current_time = getTimeHighRes();
  multi_hypothesis_tracking_msgs::ObjectDetections object_detecions;
  multi_hypothesis_tracking_msgs::ObjectDetection object;
  object_detecions.header.stamp = stamp;
  object_detecions.header.frame_id = m_world_frame;

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr <Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);
    Eigen::Vector3f mean = hypothesis->getPosition();

    //Predict a little bit into the future
    mean += hypothesis->getVelocity() * m_future_time;

    object.centroid.x = mean(0);
    object.centroid.y = mean(1);
    object.centroid.z = mean(2);

    if(current_time - hypothesis->getBornTime() >= m_born_time_threshold)
      object_detecions.object_detections.push_back(object);
  }
  m_hypotheses_predictions_publisher.publish(object_detecions);
}

void MOTPublisher::publishHypothesesPredictedPositions(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                                       const ros::Time& stamp)
{
  if(m_hypotheses_predicted_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  visualization_msgs::Marker hypothesis_marker = createMarker(0.0, 0.0, 1.0,
                                                              "mot_hypotheses_predicted_positions_markers");
  hypothesis_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr <Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    //Predict a little bit into the future
    Eigen::Vector3f mean = hypothesis->getPosition();
    mean += hypothesis->getVelocity() * m_future_time;

    geometry_msgs::Point p;
    p.x = mean(0);
    p.y = mean(1);
    p.z = mean(2);

    if(current_time - hypothesis->getBornTime() >= m_born_time_threshold)
      hypothesis_marker.points.push_back(p);
  }
  m_hypotheses_predicted_positions_publisher.publish(hypothesis_marker);
}

void MOTPublisher::publishStaticHypothesesPositions(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                                    const ros::Time& stamp)
{
  if(m_static_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  visualization_msgs::Marker static_objects_marker = createMarker(0.0, 1.0, 0.0, "mot_static_hypotheses_positions");
  static_objects_marker.type = visualization_msgs::Marker::LINE_LIST;
  static_objects_marker.color.a = 1;
  static_objects_marker.scale.x = 1.0;
  static_objects_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr <Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(hypothesis->isStatic() && current_time - hypothesis->getBornTime() >= m_born_time_threshold)
    {
      srand(hypothesis->getID());
      std_msgs::ColorRGBA color;
      color.a = 1.f;
      color.r = (rand() % 1000) / 1000.f;
      color.g = (rand() % 1000) / 1000.f;
      color.b = (rand() % 1000) / 1000.f;

      const Eigen::Vector3f& mean = hypothesis->getPosition();
      geometry_msgs::Point p;
      p.x = mean(0);
      p.y = mean(1);
      p.z = mean(2);
      static_objects_marker.points.push_back(p);
      static_objects_marker.colors.push_back(color);

      //push another point for the second point of the line
      p.z += 10;
      static_objects_marker.points.push_back(p);
      static_objects_marker.colors.push_back(color);
    }
  }
  m_static_hypotheses_positions_publisher.publish(static_objects_marker);
}

void MOTPublisher::publishDynamicHypothesesPositions(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                                     const ros::Time& stamp)
{
  if(m_dynamic_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  visualization_msgs::Marker dynamic_objects_marker = createMarker(0.0, 0.5, 0.5, "mot_dynamic_hypotheses_positions");
  dynamic_objects_marker.type = visualization_msgs::Marker::LINE_LIST;
  dynamic_objects_marker.color.a = 1.0;
  dynamic_objects_marker.scale.x = 0.5;
  dynamic_objects_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr <Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(!hypothesis->isStatic() && current_time - hypothesis->getBornTime() >= m_born_time_threshold)
    {
      srand(hypothesis->getID());
      std_msgs::ColorRGBA color;
      color.a = 1.f;
      color.r = (rand() % 1000) / 1000.f;
      color.g = (rand() % 1000) / 1000.f;
      color.b = (rand() % 1000) / 1000.f;

      const Eigen::Vector3f& mean = hypothesis->getPosition();
      geometry_msgs::Point p;
      p.x = mean(0);
      p.y = mean(1);
      p.z = mean(2);
      dynamic_objects_marker.points.push_back(p);
      dynamic_objects_marker.colors.push_back(color);

      //push another point for the second point of the line
      p.z += 20;
      dynamic_objects_marker.points.push_back(p);
      dynamic_objects_marker.colors.push_back(color);
    }
  }
  m_dynamic_hypotheses_positions_publisher.publish(dynamic_objects_marker);
}

// TODO: implement
void MOTPublisher::publishFullTracks(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                     const ros::Time& stamp)
{
  if(m_track_line_publisher.getNumSubscribers() == 0)
    return;

  //Full track for first hypothesis
  // std::vector<Hypothesis*> hypotheses = m_algorithm->getHypotheses();
  // Hypothesis *hypothesis = (Hypothesis *) hypotheses[0];
  // const Eigen::Vector3f& mean = hypothesis->getPosition();
  // geometry_msgs::Point p;
  // p.x=mean(0);
  // p.y=mean(1);
  // p.z=mean(2);
  // full_track.points.push_back(p);
  // m_track_line_publisher.publish(full_track);

//  m_track_line_publisher.publish(dynamic_objects_marker);
}

void MOTPublisher::publishHypothesesPointIndices(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                                 const ros::Time& stamp)
{
  if(m_hypotheses_point_indices_publisher.getNumSubscribers() == 0)
    return;

  multi_hypothesis_tracking_msgs::HypothesesPointIndicesPtr hypotheses_msg(
    new multi_hypothesis_tracking_msgs::HypothesesPointIndices());
  hypotheses_msg->header.frame_id = m_world_frame;
  hypotheses_msg->header.stamp = stamp;

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr <Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(!hypothesis->isStatic() && stamp.toSec() == hypothesis->getLastCorrectionTime())
    {
      hypotheses_msg->point_ids.reserve(hypotheses_msg->point_ids.size() + hypothesis->getPointIds().size());
      hypotheses_msg->point_ids.insert(hypotheses_msg->point_ids.end(), hypothesis->getPointIds().begin(),
                                       hypothesis->getPointIds().end());
    }
  }
  m_hypotheses_point_indices_publisher.publish(hypotheses_msg);
}

void MOTPublisher::publishHypothesesBoxesEvaluation(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                                    const ros::Time& stamp)
{
  if(m_hypotheses_box_evaluation_publisher.getNumSubscribers() == 0)
    return;

  multi_hypothesis_tracking_msgs::HypothesesEvaluationBoxesPtr hypotheses_msg(
    new multi_hypothesis_tracking_msgs::HypothesesEvaluationBoxes());
  hypotheses_msg->header.frame_id = m_world_frame;
  hypotheses_msg->header.stamp = stamp;

  multi_hypothesis_tracking_msgs::EvaluationBox box;

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr <Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    // TODO: dont check for last correction time and use box hypothesis below
    // publish if hypothesis was assigned to a detection in the current step
//    if(stamp.toSec() == hypothesis->getLastCorrectionTime())
//    {
    box.id = hypothesis->getID();

    box.dynamic = !hypothesis->isStatic();
    const Eigen::Array3f& min_box = hypothesis->getMinBoxHypothesis();
//      const Eigen::Array3f& min_box = hypothesis->getMinBoxDetection();
    box.min_corner.x = min_box(0);
    box.min_corner.y = min_box(1);
    box.min_corner.z = min_box(2);

    const Eigen::Array3f& max_box = hypothesis->getMaxBoxHypothesis();
//      const Eigen::Array3f& max_box = hypothesis->getMaxBoxDetection();
    box.max_corner.x = max_box(0);
    box.max_corner.y = max_box(1);
    box.max_corner.z = max_box(2);

    hypotheses_msg->boxes.push_back(box);
//    }
  }
  m_hypotheses_box_evaluation_publisher.publish(hypotheses_msg);
}

void MOTPublisher::publishHypothesesBoxesHistory(const std::vector <std::shared_ptr<Hypothesis>>& hypotheses,
                                                 const ros::Time& stamp)
{
  if(m_hypotheses_boxes_history_publisher.getNumSubscribers() == 0)
    return;

  multi_hypothesis_tracking_msgs::HypothesesBoxesArrayPtr hypotheses_array_msg(
    new multi_hypothesis_tracking_msgs::HypothesesBoxesArray());

  multi_hypothesis_tracking_msgs::HypothesesBoxesPtr hypotheses_msg(
    new multi_hypothesis_tracking_msgs::HypothesesBoxes());
  hypotheses_msg->header.frame_id = m_world_frame;

  multi_hypothesis_tracking_msgs::Box box;

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr <Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(hypothesis->turnedDynamicNow() || hypothesis->recoveredTrack())
    {
      std::vector <StampedBox> boxes;
      if(hypothesis->turnedDynamicNow())
        boxes = hypothesis->getBoxHistory();
      else
        boxes = hypothesis->getInterpolatedBoxes();

      //std::cout << "publishDeletedHypotheses: number of boxes for this hypothesis " << (int)boxes.size() << " " << std::endl;

      // for each box, test if hypothesis_array has already an entry with this stamp
      for(int box_id = 0; box_id < (int)boxes.size(); box_id++)
      {
        box.id = hypothesis->getID();

        box.min_corner.x = boxes[box_id].box.min_corner(0);
        box.min_corner.y = boxes[box_id].box.min_corner(1);
        box.min_corner.z = boxes[box_id].box.min_corner(2);

        box.max_corner.x = boxes[box_id].box.max_corner(0);
        box.max_corner.y = boxes[box_id].box.max_corner(1);
        box.max_corner.z = boxes[box_id].box.max_corner(2);

        bool stamp_already_exists = false;
        for(auto& msg_boxes : hypotheses_array_msg->hypotheses)
        {
          // if for this stamp there are already boxes, add current box to them
          if(fabs(msg_boxes.header.stamp.toSec() - boxes[box_id].time) < 0.01)
          {
            msg_boxes.boxes.push_back(box);

            stamp_already_exists = true;
            break;
          }
        }

        // if for this stamp no boxes exist, add new entry with stamp and box
        if(!stamp_already_exists)
        {
          hypotheses_msg->header.stamp = ros::Time(boxes[box_id].time);
          hypotheses_msg->boxes.clear();
          hypotheses_msg->boxes.push_back(box);
          hypotheses_array_msg->hypotheses.push_back(*hypotheses_msg);
        }
      }
    }
  }

  if(hypotheses_array_msg->hypotheses.size() > 0)
    m_hypotheses_boxes_history_publisher.publish(hypotheses_array_msg);
}

void MOTPublisher::publishDeletedHypotheses(std::queue <Hypothesis>& hypotheses)
{
  if(hypotheses.empty())
    return;

  if(m_hypotheses_boxes_history_publisher.getNumSubscribers() == 0)
    return;

  multi_hypothesis_tracking_msgs::HypothesesBoxesArrayPtr hypotheses_array_msg(
    new multi_hypothesis_tracking_msgs::HypothesesBoxesArray());

  multi_hypothesis_tracking_msgs::HypothesesBoxesPtr hypotheses_msg(
    new multi_hypothesis_tracking_msgs::HypothesesBoxes());
  hypotheses_msg->header.frame_id = m_world_frame;

  multi_hypothesis_tracking_msgs::Box box;

  while(!hypotheses.empty())
  {
    Hypothesis& hypothesis = hypotheses.front();

    const std::vector <StampedBox>& boxes = hypothesis.getBoxHistory();

    //std::cout << "publishDeletedHypotheses: number of boxes for this hypothesis " << (int)boxes.size() << " " << std::endl;

    // for each box, test if hypothesis_array has already an entry with this stamp
    for(int box_id = 0; box_id < (int)boxes.size(); box_id++)
    {
      box.id = hypothesis.getID();

      box.min_corner.x = boxes[box_id].box.min_corner(0);
      box.min_corner.y = boxes[box_id].box.min_corner(1);
      box.min_corner.z = boxes[box_id].box.min_corner(2);

      box.max_corner.x = boxes[box_id].box.max_corner(0);
      box.max_corner.y = boxes[box_id].box.max_corner(1);
      box.max_corner.z = boxes[box_id].box.max_corner(2);

      bool stamp_already_exists = false;
      for(auto& msg_boxes : hypotheses_array_msg->hypotheses)
      {
        // if for this stamp there are already boxes, add current box to them
        if(fabs(msg_boxes.header.stamp.toSec() - boxes[box_id].time) < 0.01)
        {
          msg_boxes.boxes.push_back(box);

          stamp_already_exists = true;
          break;
        }
      }

      // if for this stamp no boxes exist, add new entry with stamp and box
      if(!stamp_already_exists)
      {
        hypotheses_msg->header.stamp = ros::Time(boxes[box_id].time);
        hypotheses_msg->boxes.clear();
        hypotheses_msg->boxes.push_back(box);
        hypotheses_array_msg->hypotheses.push_back(*hypotheses_msg);
      }
    }

    hypotheses.pop();
  }

  if(hypotheses_array_msg->hypotheses.size() > 0)
    m_hypotheses_boxes_history_publisher.publish(hypotheses_array_msg);
}

void MOTPublisher::publishLikelihood(float likelihood)
{
  std_msgs::Float32 likelihood_msg;
  likelihood_msg.data = likelihood;
  m_likelihood_publisher.publish(likelihood_msg);
}

}

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
{
  ros::NodeHandle private_node_handle("~");
  initializePublishers(private_node_handle);
  getRosParameters(private_node_handle);
}

void MOTPublisher::initializePublishers(ros::NodeHandle& node_handle)
{
  m_hypotheses_full_publisher = node_handle.advertise<HypothesesFullMsg>("hypotheses_full", 1);
  m_hypotheses_predictions_publisher = node_handle.advertise<ObjectDetectionsMsg>("hypotheses_predictions", 1);

  m_detection_positions_publisher = node_handle.advertise<MarkerMsg>("detections_positions", 1);
  m_detections_covariances_publisher = node_handle.advertise<MarkerMsg>("detections_covariances", 1);
  m_detections_points_publisher = node_handle.advertise<PointCloud>("detections_points", 1);

  m_hypotheses_positions_publisher = node_handle.advertise<MarkerMsg>("hypotheses_positions", 1);
  m_hypotheses_covariance_publisher = node_handle.advertise<MarkerMsg>("hypotheses_covariances", 1);
  m_hypotheses_points_publisher = node_handle.advertise<PointCloud>("hypotheses_points", 1);

  m_static_hypotheses_positions_publisher = node_handle.advertise<MarkerMsg>("static_hypotheses_positions", 1);
  m_dynamic_hypotheses_positions_publisher = node_handle.advertise<MarkerMsg>("dynamic_hypotheses_positions", 1);

  m_hypotheses_paths_publisher = node_handle.advertise<MarkerMsg>("hypotheses_paths", 1);
  m_hypotheses_bounding_boxes_publisher = node_handle.advertise<MarkerArrayMsg>("hypotheses_bounding_boxes", 1);
  m_hypotheses_predicted_positions_publisher = node_handle.advertise<MarkerMsg>("hypotheses_predicted_positions", 1);

  m_hypotheses_box_evaluation_publisher = node_handle.advertise<HypothesesEvaluationBoxesMsg>(
    "hypotheses_boxes_evaluation", 1, true);
  m_likelihood_publisher = node_handle.advertise<std_msgs::Float32>("likelihood", 1);
}

void MOTPublisher::getRosParameters(ros::NodeHandle& node_handle)
{
  node_handle.param<std::string>("world_frame", m_world_frame, "world");
  node_handle.param<double>("born_time_threshold", m_born_time_threshold, 0.5);
  node_handle.param<int>("number_of_assignments_threshold", m_number_of_assignments_threshold, 3);
  node_handle.param<double>("future_time", m_future_time, 0.0);
}

void MOTPublisher::publishAll(const Hypotheses& hypotheses,
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
  publishHypothesesBoundingBoxes(hypotheses, stamp);
  publishHypothesesBoxesEvaluation(hypotheses, stamp);

  publishFullTracks(hypotheses, stamp);
//  publishDebug(hypotheses);
}

MarkerMsg MOTPublisher::createMarker(float r, float g, float b, std::string ns)
{
  MarkerMsg marker;
  marker.header.frame_id = m_world_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = MarkerMsg::POINTS;
  marker.action = MarkerMsg::ADD;
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

void MOTPublisher::publishDetectionPositions(const std::vector<Detection>& detections,
                                             const ros::Time& stamp)
{
  if(m_detection_positions_publisher.getNumSubscribers() == 0 || detections.empty())
    return;

  MarkerMsg detection_positions_marker = createMarker(1.0, 0.0, 0.0,
                                                      "mot_detections_markers"); //red marker
  detection_positions_marker.header.frame_id = detections.at(0).frame_id;
  detection_positions_marker.header.stamp = stamp;

  detection_positions_marker.points.resize(detections.size());
  for(size_t i = 0; i < detections.size(); i++)
  {
    detection_positions_marker.points[i].x = detections[i].position(0);
    detection_positions_marker.points[i].y = detections[i].position(1);
    detection_positions_marker.points[i].z = detections[i].position(2);
  }
  m_detection_positions_publisher.publish(detection_positions_marker);
}

void MOTPublisher::publishDetectionsCovariances(const std::vector<Detection>& detections,
                                                const ros::Time& stamp)
{
  if(m_detections_covariances_publisher.getNumSubscribers() == 0 || detections.empty())
    return;

  MarkerMsg detection_cov_marker = createMarker(1.0, 0.0, 0.0, "mot_detection_covariance_marker");;
  detection_cov_marker.type = MarkerMsg::SPHERE;
  detection_cov_marker.color.a = 0.5f;
  detection_cov_marker.header.stamp = stamp;

  for(size_t i = 0; i < detections.size(); i++)
  {
    detection_cov_marker.header.frame_id = detections.at(i).frame_id;
    detection_cov_marker.id = (int)i;
    detection_cov_marker.pose.position.x = detections[i].position(0);
    detection_cov_marker.pose.position.y = detections[i].position(1);
    detection_cov_marker.pose.position.z = detections[i].position(2);

    detection_cov_marker.scale.x = sqrt(4.204) * sqrt(detections[i].covariance(0, 0));
    detection_cov_marker.scale.y = sqrt(4.204) * sqrt(detections[i].covariance(1, 1));
    detection_cov_marker.scale.z = sqrt(4.204) * sqrt(detections[i].covariance(2, 2));

    m_detections_covariances_publisher.publish(detection_cov_marker);
  }
}

void MOTPublisher::publishDetectionsPoints(const std::vector<Detection>& detections,
                                           const ros::Time& stamp)
{
  if(m_detections_points_publisher.getNumSubscribers() == 0 || detections.empty())
    return;

  PointCloud::Ptr cloud(new PointCloud);
  cloud->header.frame_id = detections.at(0).frame_id;
  cloud->header.stamp = pcl_conversions::toPCL(stamp);

  int total_points_count = 0;
  for(size_t i = 0; i < detections.size(); i++)
    total_points_count += detections.at(i).points.size();

  cloud->points.resize(total_points_count);
  int point_counter = 0;
  for(size_t i = 0; i < detections.size(); i++)
    for(size_t point_id = 0; point_id < detections.at(i).points.size(); point_id++, point_counter++)
      cloud->points[point_counter].getVector3fMap() = detections.at(i).points.at(point_id);

  m_detections_points_publisher.publish(cloud);
}

void MOTPublisher::publishHypothesesCovariances(const Hypotheses& hypotheses,
                                                const ros::Time& stamp)
{
  if(m_hypotheses_covariance_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  double current_time = getTimeHighRes();

  MarkerMsg hyp_covariance_marker = createMarker(1.0, 0.0, 0.0, "mot_hypotheses_covariance_marker");;
  hyp_covariance_marker.type = MarkerMsg::SPHERE;
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

    if(current_time - hypotheses[i]->getBornTime() >= m_born_time_threshold
       && hypotheses[i]->getNumberOfAssignments() >= m_number_of_assignments_threshold)
      m_hypotheses_covariance_publisher.publish(hyp_covariance_marker);
  }
}


void MOTPublisher::publishHypothesesPositions(const Hypotheses& hypotheses,
                                              const ros::Time& stamp)
{
  if(m_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  MarkerMsg hypothesis_marker = createMarker(0.0, 1.0, 0.0, "mot_hypotheses_positions_markers");
  hypothesis_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    const Eigen::Vector3f& mean = hypothesis->getPosition();
    geometry_msgs::Point p;
    p.x = mean(0);
    p.y = mean(1);
    p.z = mean(2);

    if(current_time - hypothesis->getBornTime() >= m_born_time_threshold
       && hypothesis->getNumberOfAssignments() >= m_number_of_assignments_threshold)
      hypothesis_marker.points.push_back(p);
  }
  m_hypotheses_positions_publisher.publish(hypothesis_marker);
}

void MOTPublisher::publishHypothesesPoints(const Hypotheses& hypotheses,
                                           const ros::Time& stamp)
{
  if(m_hypotheses_points_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  PointCloud::Ptr cloud(new PointCloud);
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

void MOTPublisher::publishHypothesesFull(const Hypotheses& hypotheses,
                                         const ros::Time& stamp)
{
  if(m_hypotheses_full_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  HypothesesFullMsg::Ptr hypotheses_msg(new HypothesesFullMsg());
  hypotheses_msg->header.frame_id = m_world_frame;
  hypotheses_msg->header.stamp = stamp;

  multi_hypothesis_tracking_msgs::State state;
  multi_hypothesis_tracking_msgs::Box box;

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

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

void MOTPublisher::publishHypothesesPredictions(const Hypotheses& hypotheses,
                                                const ros::Time& stamp)
{
  if(m_hypotheses_predictions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  double current_time = getTimeHighRes();
  ObjectDetectionsMsg object_detecions;
  multi_hypothesis_tracking_msgs::ObjectDetection object;
  object_detecions.header.stamp = stamp;
  object_detecions.header.frame_id = m_world_frame;

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);
    Eigen::Vector3f mean = hypothesis->getPosition();

    //Predict a little bit into the future
    mean += hypothesis->getVelocity() * m_future_time;

    object.centroid.x = mean(0);
    object.centroid.y = mean(1);
    object.centroid.z = mean(2);

    if(current_time - hypothesis->getBornTime() >= m_born_time_threshold
       && hypothesis->getNumberOfAssignments() >= m_number_of_assignments_threshold)
      object_detecions.object_detections.push_back(object);
  }
  m_hypotheses_predictions_publisher.publish(object_detecions);
}

void MOTPublisher::publishHypothesesPredictedPositions(const Hypotheses& hypotheses,
                                                       const ros::Time& stamp)
{
  if(m_hypotheses_predicted_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  MarkerMsg hypothesis_marker = createMarker(0.0, 0.0, 1.0,
                                             "mot_hypotheses_predicted_positions_markers");
  hypothesis_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    //Predict a little bit into the future
    Eigen::Vector3f mean = hypothesis->getPosition();
    mean += hypothesis->getVelocity() * m_future_time;

    geometry_msgs::Point p;
    p.x = mean(0);
    p.y = mean(1);
    p.z = mean(2);

    if(current_time - hypothesis->getBornTime() >= m_born_time_threshold
       && hypothesis->getNumberOfAssignments() >= m_number_of_assignments_threshold)
      hypothesis_marker.points.push_back(p);
  }
  m_hypotheses_predicted_positions_publisher.publish(hypothesis_marker);
}

void MOTPublisher::publishStaticHypothesesPositions(const Hypotheses& hypotheses,
                                                    const ros::Time& stamp)
{
  if(m_static_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  double color_alpha = 0.5;
  MarkerMsg static_objects_marker = createMarker(0.0, 1.0, 0.0, "mot_static_hypotheses_positions");
  static_objects_marker.type = MarkerMsg::LINE_LIST;
  static_objects_marker.color.a = color_alpha;
  static_objects_marker.scale.x = 0.4;
  static_objects_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(hypothesis->isStatic()
       && current_time - hypothesis->getBornTime() >= m_born_time_threshold
       && hypothesis->getNumberOfAssignments() >= m_number_of_assignments_threshold)
    {
      srand(hypothesis->getID());
      std_msgs::ColorRGBA color;
      color.a = color_alpha;
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
      p.z += 2;
      static_objects_marker.points.push_back(p);
      static_objects_marker.colors.push_back(color);
    }
  }
  m_static_hypotheses_positions_publisher.publish(static_objects_marker);
}

void MOTPublisher::publishDynamicHypothesesPositions(const Hypotheses& hypotheses,
                                                     const ros::Time& stamp)
{
  if(m_dynamic_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  double color_alpha = 0.5;
  MarkerMsg dynamic_objects_marker = createMarker(0.0, 0.5, 0.5, "mot_dynamic_hypotheses_positions");
  dynamic_objects_marker.type = MarkerMsg::LINE_LIST;
  dynamic_objects_marker.color.a = color_alpha;
  dynamic_objects_marker.scale.x = 0.2;
  dynamic_objects_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  // Publish tracks
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(!hypothesis->isStatic()
       && current_time - hypothesis->getBornTime() >= m_born_time_threshold
       && hypothesis->getNumberOfAssignments() >= m_number_of_assignments_threshold)
    {
      srand(hypothesis->getID());
      std_msgs::ColorRGBA color;
      color.a = color_alpha;
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
      p.z += 4;
      dynamic_objects_marker.points.push_back(p);
      dynamic_objects_marker.colors.push_back(color);
    }
  }
  m_dynamic_hypotheses_positions_publisher.publish(dynamic_objects_marker);
}

void MOTPublisher::publishHypothesesBoundingBoxes(const Hypotheses& hypotheses,
                                                  const ros::Time& stamp)
{
  if(m_hypotheses_bounding_boxes_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  MarkerArrayMsg bounding_boxes_markers;

  double color_alpha = 0.5;
  MarkerMsg hypotheses_boxes_marker = createMarker(0.0, 0.5, 0.5, "mot_hypotheses_bounding_boxes");
  hypotheses_boxes_marker.type = MarkerMsg::CUBE;
  hypotheses_boxes_marker.action = MarkerMsg::ADD;
  hypotheses_boxes_marker.color.a = color_alpha;
  hypotheses_boxes_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  // Publish bounding boxes
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(current_time - hypothesis->getBornTime() >= m_born_time_threshold
       && hypothesis->getNumberOfAssignments() >= m_number_of_assignments_threshold)
    {
      hypotheses_boxes_marker.id = hypothesis->getID();
      srand(hypothesis->getID());
      hypotheses_boxes_marker.color.a = color_alpha;
      hypotheses_boxes_marker.color.r = (rand() % 1000) / 1000.f;
      hypotheses_boxes_marker.color.g = (rand() % 1000) / 1000.f;
      hypotheses_boxes_marker.color.b = (rand() % 1000) / 1000.f;

      const Eigen::Vector3f& mean = hypothesis->getPosition();
      geometry_msgs::Point p;
      p.x = mean(0);
      p.y = mean(1);
      p.z = mean(2);
      hypotheses_boxes_marker.pose.position = p;

      hypotheses_boxes_marker.pose.orientation.x = 0.0;
      hypotheses_boxes_marker.pose.orientation.y = 0.0;
      hypotheses_boxes_marker.pose.orientation.z = 0.0;
      hypotheses_boxes_marker.pose.orientation.w = 1.0;

      auto box_size = hypothesis->getHypothesisBoxSize();
      hypotheses_boxes_marker.scale.x = std::max(static_cast<float>(box_size.x()), 0.1f);
      hypotheses_boxes_marker.scale.y = std::max(static_cast<float>(box_size.y()), 0.1f);
      hypotheses_boxes_marker.scale.z = std::max(static_cast<float>(box_size.z()), 0.1f);

      hypotheses_boxes_marker.lifetime = ros::Duration(0, 100000000); // 0.1 seconds

      bounding_boxes_markers.markers.push_back(hypotheses_boxes_marker);
    }
  }
  m_hypotheses_bounding_boxes_publisher.publish(bounding_boxes_markers);
}

void MOTPublisher::publishFullTracks(const Hypotheses& hypotheses,
                                     const ros::Time& stamp)
{
  if(m_hypotheses_paths_publisher.getNumSubscribers() == 0)
    return;

  double color_alpha = 1.0;
  MarkerMsg hypotheses_paths_marker = createMarker(0.0, 0.5, 0.5, "mot_hypotheses_paths");
  hypotheses_paths_marker.type = MarkerMsg::LINE_LIST;
  hypotheses_paths_marker.color.a = color_alpha;
  hypotheses_paths_marker.scale.x = 0.325;
  hypotheses_paths_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  std_msgs::ColorRGBA predictions_color;
  predictions_color.a = color_alpha;
  predictions_color.r = 0.f;
  predictions_color.g = 0.f;
  predictions_color.b = 0.f;

  // Publish paths
  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(current_time - hypothesis->getBornTime() >= m_born_time_threshold
       && hypothesis->getNumberOfAssignments() >= m_number_of_assignments_threshold)
    {
      const std::vector<Eigen::Vector3f>& positions = hypothesis->getPositionHistory();
      const std::vector<bool>& was_assigned = hypothesis->getWasAssignedHistory();

      if(positions.size() <= 1)
        continue;

      srand(hypothesis->getID());
      std_msgs::ColorRGBA assigned_color;
      assigned_color.a = color_alpha;
      assigned_color.r = (rand() % 1000) / 1000.f;
      assigned_color.g = (rand() % 1000) / 1000.f;
      assigned_color.b = (rand() % 1000) / 1000.f;

      size_t last_index = positions.size() - 1;
      for(size_t j = 0; j < positions.size(); j++)
      {
        const auto& position = positions[j];

        geometry_msgs::Point p;
        p.x = position(0);
        p.y = position(1);
        p.z = position(2);
        hypotheses_paths_marker.points.push_back(p);
        if(was_assigned[j])
          hypotheses_paths_marker.colors.push_back(assigned_color);
        else
          hypotheses_paths_marker.colors.push_back(predictions_color);

        if(j == 0 || j == last_index)
          continue;

        // Add current position again, as the start of the next line segment
        hypotheses_paths_marker.points.push_back(p);
        if(was_assigned[j])
          hypotheses_paths_marker.colors.push_back(assigned_color);
        else
          hypotheses_paths_marker.colors.push_back(predictions_color);
      }
    }
  }
  m_hypotheses_paths_publisher.publish(hypotheses_paths_marker);
}

void MOTPublisher::publishHypothesesBoxesEvaluation(const Hypotheses& hypotheses,
                                                    const ros::Time& stamp)
{
  if(m_hypotheses_box_evaluation_publisher.getNumSubscribers() == 0)
    return;

  HypothesesEvaluationBoxesMsg::Ptr hypotheses_msg(new HypothesesEvaluationBoxesMsg());
  hypotheses_msg->header.frame_id = m_world_frame;
  hypotheses_msg->header.stamp = stamp;

  multi_hypothesis_tracking_msgs::EvaluationBox box;

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

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

void MOTPublisher::publishLikelihood(float likelihood)
{
  std_msgs::Float32 likelihood_msg;
  likelihood_msg.data = likelihood;
  m_likelihood_publisher.publish(likelihood_msg);
}

}

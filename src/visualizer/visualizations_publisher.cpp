/** @file
 *
 * Class to publish data from the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/visualizer/visualizations_publisher.h>

namespace MultiHypothesisTracker
{

VisualizationsPublisher::VisualizationsPublisher()
{
  ros::NodeHandle private_node_handle("~");
  initializePublishers(private_node_handle);
  getRosParameters(private_node_handle);
}

void VisualizationsPublisher::initializePublishers(ros::NodeHandle& node_handle)
{
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

  m_hypotheses_full_publisher = node_handle.advertise<HypothesesFullMsg>("hypotheses_full", 1);
  m_hypotheses_box_evaluation_publisher = node_handle.advertise<HypothesesEvaluationBoxesMsg>(
    "hypotheses_boxes_evaluation", 1, true);
}

void VisualizationsPublisher::getRosParameters(ros::NodeHandle& node_handle)
{
  node_handle.param<std::string>("world_frame_id", m_world_frame_id, "world");
  node_handle.param<double>("hypothesis_age_threshold_in_seconds", m_hypothesis_age_threshold_in_seconds, 0.5);
  node_handle.param<int>("number_of_assignments_threshold", m_number_of_assignments_threshold, 3);
  node_handle.param<double>("time_offset_for_predictions", m_time_offset_for_predictions, 0.0);
}

void VisualizationsPublisher::publishDetectionsVisualizations(const Detections& detections)
{
  publishDetectionsPositions(detections);
  publishDetectionsCovariances(detections);
  publishDetectionsPoints(detections);
}

void VisualizationsPublisher::publishHypothesesVisualizations(const Hypotheses& hypotheses,
                                                              const ros::Time& stamp)
{
  if(hypotheses.empty())
    ROS_DEBUG_STREAM("Publishing empty hypotheses.");

  publishHypothesesPositions(hypotheses, stamp);
  publishHypothesesCovariances(hypotheses, stamp);
  publishHypothesesPoints(hypotheses, stamp);

  publishStaticHypothesesPositions(hypotheses, stamp);
  publishDynamicHypothesesPositions(hypotheses, stamp);

  publishHypothesesPaths(hypotheses, stamp);
  publishHypothesesBoundingBoxes(hypotheses, stamp);
  publishHypothesesPredictedPositions(hypotheses, stamp);

  publishHypothesesFull(hypotheses, stamp);
  publishHypothesesBoxesEvaluation(hypotheses, stamp);
}

MarkerMsg VisualizationsPublisher::createMarker(float r,
                                                float g,
                                                float b,
                                                const std::string& name_space)
{
  MarkerMsg marker;
  marker.header.frame_id = m_world_frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = name_space;
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
  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.lifetime = ros::Duration(1.0);
  return marker;
}

void VisualizationsPublisher::publishDetectionsPositions(const Detections& detections)
{
  if(m_detection_positions_publisher.getNumSubscribers() == 0 || detections.detections.empty())
    return;

  MarkerMsg detection_positions_marker = createMarker(1.0, 0.0, 0.0, "mot_detections_markers");
  detection_positions_marker.header.frame_id = detections.frame_id;
  detection_positions_marker.header.stamp = ros::Time(detections.time_stamp);

  detection_positions_marker.points.resize(detections.detections.size());
  for(size_t i = 0; i < detections.detections.size(); i++)
    eigenToGeometryMsgs(detections.detections[i].position, detection_positions_marker.points[i]);

  m_detection_positions_publisher.publish(detection_positions_marker);
}

void VisualizationsPublisher::publishDetectionsCovariances(const Detections& detections)
{
  if(m_detections_covariances_publisher.getNumSubscribers() == 0 || detections.detections.empty())
    return;

  MarkerMsg detection_cov_marker = createMarker(1.0, 0.0, 0.0, "mot_detection_covariance_marker");;
  detection_cov_marker.type = MarkerMsg::SPHERE;
  detection_cov_marker.color.a = 0.5f;
  detection_cov_marker.header.frame_id = detections.frame_id;
  detection_cov_marker.header.stamp = ros::Time(detections.time_stamp);

  for(size_t i = 0; i < detections.detections.size(); i++)
  {
    detection_cov_marker.id = (int)i;
    eigenToGeometryMsgs(detections.detections[i].position, detection_cov_marker.pose.position);

    detection_cov_marker.scale.x = sqrt(4.204) * sqrt(detections.detections[i].covariance(0, 0));
    detection_cov_marker.scale.y = sqrt(4.204) * sqrt(detections.detections[i].covariance(1, 1));
    detection_cov_marker.scale.z = sqrt(4.204) * sqrt(detections.detections[i].covariance(2, 2));

    m_detections_covariances_publisher.publish(detection_cov_marker);
  }
}

void VisualizationsPublisher::publishDetectionsPoints(const Detections& detections)
{
  if(m_detections_points_publisher.getNumSubscribers() == 0 || detections.detections.empty())
    return;

  PointCloud::Ptr cloud(new PointCloud);
  convertDetectionsPointsToCloud(detections, cloud);
  m_detections_points_publisher.publish(cloud);
}

void VisualizationsPublisher::publishHypothesesPositions(const Hypotheses& hypotheses,
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

    if(isValid(hypothesis, current_time))
    {
      geometry_msgs::Point position;
      eigenToGeometryMsgs(hypothesis->getPosition(), position);
      hypothesis_marker.points.push_back(position);
    }
  }
  m_hypotheses_positions_publisher.publish(hypothesis_marker);
}

bool VisualizationsPublisher::isValid(std::shared_ptr<Hypothesis>& hypothesis,
                                      double current_time) const
{
  return isOldEnough(hypothesis, current_time) && wasAssignedOftenEnough(hypothesis);
}

bool VisualizationsPublisher::isOldEnough(std::shared_ptr<Hypothesis>& hypothesis,
                                          double current_time) const
{
  return current_time - hypothesis->getTimeStampOfBirth() >= m_hypothesis_age_threshold_in_seconds;
}

bool VisualizationsPublisher::wasAssignedOftenEnough(std::shared_ptr<Hypothesis>& hypothesis) const
{
  return hypothesis->getNumberOfAssignments() >= m_number_of_assignments_threshold;
}

void VisualizationsPublisher::publishHypothesesCovariances(const Hypotheses& hypotheses,
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
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);
    if(isValid(hypothesis, current_time))
    {
      hyp_covariance_marker.id = (int)i;
      eigenToGeometryMsgs(hypothesis->getPosition(), hyp_covariance_marker.pose.position);

      hyp_covariance_marker.scale.x = sqrt(4.204) * sqrt(hypothesis->getCovariance()(0, 0));
      hyp_covariance_marker.scale.y = sqrt(4.204) * sqrt(hypothesis->getCovariance()(1, 1));
      hyp_covariance_marker.scale.z = sqrt(4.204) * sqrt(hypothesis->getCovariance()(2, 2));

      m_hypotheses_covariance_publisher.publish(hyp_covariance_marker);
    }
  }
}

void VisualizationsPublisher::publishHypothesesPoints(const Hypotheses& hypotheses,
                                                      const ros::Time& stamp)
{
  if(m_hypotheses_points_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  PointCloud::Ptr cloud(new PointCloud);
  cloud->header.frame_id = m_world_frame_id;
  cloud->header.stamp = pcl_conversions::toPCL(stamp);

  convertHypothesesPointsToCloud(hypotheses, cloud);

  m_hypotheses_points_publisher.publish(cloud);
}

void VisualizationsPublisher::publishStaticHypothesesPositions(const Hypotheses& hypotheses,
                                                               const ros::Time& stamp)
{
  if(m_static_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  float color_alpha = 0.5f;
  MarkerMsg static_objects_marker = createMarker(0.0, 1.0, 0.0, "mot_static_hypotheses_positions");
  static_objects_marker.type = MarkerMsg::LINE_LIST;
  static_objects_marker.color.a = color_alpha;
  static_objects_marker.scale.x = 0.4;
  static_objects_marker.header.stamp = stamp;
  double current_time = getTimeHighRes();

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    if(isValid(hypothesis, current_time) && hypothesis->isStatic())
    {
      std_msgs::ColorRGBA color;
      getColorByID(hypothesis->getID(), color);
      color.a = color_alpha;

      geometry_msgs::Point position;
      eigenToGeometryMsgs(hypothesis->getPosition(), position);

      static_objects_marker.points.push_back(position);
      static_objects_marker.colors.push_back(color);

      //push another point for the second point of the line
      position.z += 2;
      static_objects_marker.points.push_back(position);
      static_objects_marker.colors.push_back(color);
    }
  }
  m_static_hypotheses_positions_publisher.publish(static_objects_marker);
}

void VisualizationsPublisher::publishDynamicHypothesesPositions(const Hypotheses& hypotheses,
                                                                const ros::Time& stamp)
{
  if(m_dynamic_hypotheses_positions_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  float color_alpha = 0.5;
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

    if(isValid(hypothesis, current_time) && !hypothesis->isStatic())
    {
      std_msgs::ColorRGBA color;
      getColorByID(hypothesis->getID(), color);
      color.a = color_alpha;

      geometry_msgs::Point position;
      eigenToGeometryMsgs(hypothesis->getPosition(), position);

      dynamic_objects_marker.points.push_back(position);
      dynamic_objects_marker.colors.push_back(color);

      //push another point for the second point of the line
      position.z += 4;
      dynamic_objects_marker.points.push_back(position);
      dynamic_objects_marker.colors.push_back(color);
    }
  }
  m_dynamic_hypotheses_positions_publisher.publish(dynamic_objects_marker);
}

void VisualizationsPublisher::publishHypothesesPaths(const Hypotheses& hypotheses,
                                                     const ros::Time& stamp)
{
  if(m_hypotheses_paths_publisher.getNumSubscribers() == 0)
    return;

  float color_alpha = 1.0;
  MarkerMsg hypotheses_paths_marker = createMarker(0.0, 0.5, 0.5, "mot_hypotheses_paths");
  hypotheses_paths_marker.type = MarkerMsg::LINE_LIST;
  hypotheses_paths_marker.color.a = color_alpha;
  hypotheses_paths_marker.scale.x = 0.075;
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

    if(isValid(hypothesis, current_time))
    {
      const std::vector<Eigen::Vector3f>& positions = hypothesis->getPositionHistory();
      const std::vector<bool>& was_assigned = hypothesis->getWasAssignedHistory();

      if(positions.size() <= 1)
        continue;

      std_msgs::ColorRGBA assigned_color;
      getColorByID(hypothesis->getID(), assigned_color);
      assigned_color.a = color_alpha;

      size_t last_index = positions.size() - 1;
      for(size_t j = 0; j < positions.size(); j++)
      {
        geometry_msgs::Point position;
        eigenToGeometryMsgs(positions[j], position);

        hypotheses_paths_marker.points.push_back(position);
        if(was_assigned[j])
          hypotheses_paths_marker.colors.push_back(assigned_color);
        else
          hypotheses_paths_marker.colors.push_back(predictions_color);

        if(j == 0 || j == last_index)
          continue;

        // Add current position again, as the start of the next line segment
        hypotheses_paths_marker.points.push_back(position);
        if(was_assigned[j])
          hypotheses_paths_marker.colors.push_back(assigned_color);
        else
          hypotheses_paths_marker.colors.push_back(predictions_color);
      }
    }
  }
  m_hypotheses_paths_publisher.publish(hypotheses_paths_marker);
}

void VisualizationsPublisher::publishHypothesesBoundingBoxes(const Hypotheses& hypotheses,
                                                             const ros::Time& stamp)
{
  if(m_hypotheses_bounding_boxes_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  MarkerArrayMsg bounding_boxes_markers;

  float color_alpha = 0.5;
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

    if(isValid(hypothesis, current_time))
    {
      hypotheses_boxes_marker.id = hypothesis->getID();
      getColorByID(hypothesis->getID(), hypotheses_boxes_marker.color);
      hypotheses_boxes_marker.color.a = color_alpha;

      eigenToGeometryMsgs(hypothesis->getPosition(), hypotheses_boxes_marker.pose.position);

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

void VisualizationsPublisher::publishHypothesesPredictedPositions(const Hypotheses& hypotheses,
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

    if(isValid(hypothesis, current_time))
    {
      //Predict a little bit into the future
      Eigen::Vector3f mean = hypothesis->getPosition();
      mean += hypothesis->getVelocity() * m_time_offset_for_predictions;

      geometry_msgs::Point position;
      eigenToGeometryMsgs(mean, position);
      hypothesis_marker.points.push_back(position);
    }
  }
  m_hypotheses_predicted_positions_publisher.publish(hypothesis_marker);
}

void VisualizationsPublisher::publishHypothesesFull(const Hypotheses& hypotheses,
                                                    const ros::Time& stamp)
{
  if(m_hypotheses_full_publisher.getNumSubscribers() == 0 || hypotheses.empty())
    return;

  HypothesesFullMsg::Ptr hypotheses_msg(new HypothesesFullMsg());
  hypotheses_msg->header.frame_id = m_world_frame_id;
  hypotheses_msg->header.stamp = stamp;

  multi_hypothesis_tracking_msgs::State state;
  multi_hypothesis_tracking_msgs::Box box;

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    // TODO: add something like confidence to message to be able to seperate hypotheses that were just assigned from those that are tracked for a while

    // fill state
    state.id = hypothesis->getID();

    eigenToGeometryMsgs(hypothesis->getPosition(), state.position);
    eigenToGeometryMsgs(hypothesis->getVelocity(), state.velocity);

    hypotheses_msg->states.push_back(state);

    // fill box
    box.id = hypothesis->getID();

    const Eigen::Array3f& min_box = hypothesis->getDetectionsBoundingBox().min_corner;
    box.min_corner.x = min_box(0);
    box.min_corner.y = min_box(1);
    box.min_corner.z = min_box(2);

    const Eigen::Array3f& max_box = hypothesis->getDetectionsBoundingBox().max_corner;
    box.max_corner.x = max_box(0);
    box.max_corner.y = max_box(1);
    box.max_corner.z = max_box(2);

    hypotheses_msg->boxes.push_back(box);
  }
  m_hypotheses_full_publisher.publish(hypotheses_msg);
}

void VisualizationsPublisher::publishHypothesesBoxesEvaluation(const Hypotheses& hypotheses,
                                                               const ros::Time& stamp)
{
  if(m_hypotheses_box_evaluation_publisher.getNumSubscribers() == 0)
    return;

  HypothesesEvaluationBoxesMsg::Ptr hypotheses_msg(new HypothesesEvaluationBoxesMsg());
  hypotheses_msg->header.frame_id = m_world_frame_id;
  hypotheses_msg->header.stamp = stamp;

  multi_hypothesis_tracking_msgs::EvaluationBox box;

  for(size_t i = 0; i < hypotheses.size(); ++i)
  {
    std::shared_ptr<Hypothesis> hypothesis = std::static_pointer_cast<Hypothesis>(hypotheses[i]);

    box.id = hypothesis->getID();
    box.dynamic = !hypothesis->isStatic();

    const Eigen::Array3f& min_box = hypothesis->getHypothesisBoundingBox().min_corner;
    box.min_corner.x = min_box(0);
    box.min_corner.y = min_box(1);
    box.min_corner.z = min_box(2);

    const Eigen::Array3f& max_box = hypothesis->getHypothesisBoundingBox().max_corner;
    box.max_corner.x = max_box(0);
    box.max_corner.y = max_box(1);
    box.max_corner.z = max_box(2);

    hypotheses_msg->boxes.push_back(box);
  }
  m_hypotheses_box_evaluation_publisher.publish(hypotheses_msg);
}

}

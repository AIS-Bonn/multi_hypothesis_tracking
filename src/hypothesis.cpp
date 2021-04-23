/** @file
 *
 * Hypothesis implementation.
 *
 * @author Jan Razlaw
 */

#include "multi_hypothesis_tracking/hypothesis.h"

namespace MultiHypothesisTracker
{

Hypothesis::Hypothesis(const Detection& detection,
                       unsigned int id,
                       double time_stamp,
                       float covariance_per_second)
  : m_id(id)
    , m_born_time(time_stamp)
    , m_is_static(true)
    , m_static_distance_threshold(1.f)
    , m_cap_velocity(true)
    , m_max_allowed_velocity(2.8) // 1.4m/s or 5km/h
    , m_max_tracked_velocity(0.0)
    , m_was_assigned_counter(0)
{
  int number_of_state_dimensions = (int)detection.position.size() * 2;   // position dimensions + velocity dimensions
  Eigen::VectorXf initial_hypothesis_state(number_of_state_dimensions);
  initial_hypothesis_state.setZero();
  for(int i = 0; i < 3; i++)
    initial_hypothesis_state(i) = detection.position(i);

  m_kalman = std::make_shared<KalmanFilter>(initial_hypothesis_state);
  m_kalman->setCovariancePerSecond(covariance_per_second);

  m_first_position_in_track = getPosition();
  m_position_history.push_back(m_first_position_in_track);
  m_was_assigned_history.push_back(true);

  m_points = detection.points;

  computeBoundingBox(m_points, m_min_corner_detection, m_max_corner_detection); // length width height
  m_min_corner_hypothesis = m_min_corner_detection;
  m_max_corner_hypothesis = m_max_corner_detection;
  m_min_corner_init_hypothesis = m_min_corner_detection;
  m_max_corner_init_hypothesis = m_max_corner_detection;
}

void Hypothesis::predict(float dt)
{
  auto current_position = getPosition();

  m_kalman->predict(dt);

  // safe predicted position
  m_position_history.push_back(getPosition());
  m_was_assigned_history.push_back(false);

  // update the positions of the points corresponding to that hypothesis
  auto transform_current_to_predicted = (getPosition() - current_position).eval();
  transformPoints(m_points, transform_current_to_predicted);

  m_min_corner_hypothesis = (m_min_corner_hypothesis + transform_current_to_predicted.array()).eval();
  m_max_corner_hypothesis = (m_max_corner_hypothesis + transform_current_to_predicted.array()).eval();
}

void Hypothesis::correct(const Detection& detection)
{
  auto current_position = getPosition();

  m_kalman->correct(detection.position, detection.covariance);

  // if hypothesis' position was corrected, replace the latest predicted position by the corrected
  m_position_history.back() = getPosition();
  m_was_assigned_history.back() = true;
  m_was_assigned_counter++;

  // update the positions of the points corresponding to that hypothesis
  auto transform_predicted_to_corrected = (getPosition() - current_position).eval();
  transformPoints(m_points, transform_predicted_to_corrected);


  // additional stuff and workarounds

  Eigen::Vector3f current_velocity = getVelocity();

  // keep track of maximal recorded velocity
  double current_velocity_magnitude = current_velocity.norm();
  if(current_velocity_magnitude > m_max_tracked_velocity)
  {
    m_max_tracked_velocity = current_velocity_magnitude;
  }

  if(m_cap_velocity)
  {
    // high pass filter on velocity to prevent movements that are induced due to sensor noise or the high vertical resolution between the scan rings
    Eigen::Vector3f horizontal_velocity_vec = current_velocity;
    horizontal_velocity_vec.z() = 0.f;
    if(horizontal_velocity_vec.norm() < 0.3f)
    {
      current_velocity.x() = 0.f;
      current_velocity.y() = 0.f;
    }
    if(current_velocity.z() < 0.4f)
      current_velocity.z() = 0.f;

    // capping velocity to prevent "shooting" away objects when for example the mapping jumps
    if(current_velocity.norm() > m_max_allowed_velocity)
    {
      current_velocity.normalize();
      current_velocity *= m_max_allowed_velocity;
      for(int i = 0; i < 3; i++)
        m_kalman->getState()(3 + i) = current_velocity(i);
    }

    // set the corrected velocity
    for(int i = 0; i < 3; i++)
      m_kalman->getState()(3 + i) = current_velocity(i);
  }

  // transform detection points to the current state's position and add them to the hypothesis' points
  auto transform_detection_to_corrected = (getPosition() - detection.position).eval();
  std::vector<Eigen::Vector3f> corrected_detection_points;
  corrected_detection_points.reserve(detection.points.size());
  for(const auto& point : detection.points)
    corrected_detection_points.emplace_back(Eigen::Vector3f(point + transform_detection_to_corrected));

  //TODO: filter if too many points in hypothesis
  m_points.reserve(m_points.size() + corrected_detection_points.size());
  m_points.insert(m_points.end(), corrected_detection_points.begin(), corrected_detection_points.end());

  // update hypothesis' bounding box using corrected detection points
//  computeBoundingBox(corrected_detection_points, m_min_corner_hypothesis, m_max_corner_hypothesis);
  // update bounding box of assigned detection
  computeBoundingBox(detection.points, m_min_corner_detection, m_max_corner_detection);

  auto detection_centroid_position = (m_max_corner_detection + m_min_corner_detection) / 2.f;
  auto detection_box_size = (m_max_corner_detection - m_min_corner_detection).eval();
  auto hypothesis_box_size = (m_max_corner_hypothesis - m_min_corner_hypothesis).eval();
  auto box_mean_size = (detection_box_size + hypothesis_box_size) / 2.f;

  m_max_corner_hypothesis = detection_centroid_position + (box_mean_size) / 2.f;
  m_min_corner_hypothesis = detection_centroid_position - (box_mean_size) / 2.f;

//	verifyStatic(m_min_corner_detection, m_max_corner_detection);
  verifyStatic();
}

void Hypothesis::transformPoints(std::vector<Eigen::Vector3f>& points,
                                 const Eigen::Vector3f& transform)
{
  for(auto& point : points)
    point += transform;
}

void Hypothesis::computeBoundingBox(const std::vector<Eigen::Vector3f>& points,
                                    Eigen::Array3f& min_bounding_box,
                                    Eigen::Array3f& max_bounding_box)
{
  min_bounding_box = Eigen::Array3f::Constant(std::numeric_limits<float>::max());
  max_bounding_box = Eigen::Array3f::Constant(-std::numeric_limits<float>::max());
  for(const auto& point : points)
  {
    for(std::size_t i = 0; i < 3; ++i)
    {
      min_bounding_box[i] = std::min(min_bounding_box[i], point[i]);
      max_bounding_box[i] = std::max(max_bounding_box[i], point[i]);
    }
  }
}

bool Hypothesis::exceedsMaxCovariance(const Eigen::Matrix3f& covariance,
                                      float max_covariance)
{
  Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
  auto eigen_values = eigen_solver.eigenvalues();

//  std::cout << "eigen values of hyp with id = " << m_id << " are " << eigen_values.col(0)[0].real() << " "
//            << eigen_values.col(0)[1].real() << " " << eigen_values.col(0)[2].real() << " " << std::endl;

  return (eigen_values.col(0)[0].real() > max_covariance ||
          eigen_values.col(0)[1].real() > max_covariance ||
          eigen_values.col(0)[2].real() > max_covariance);
}

bool Hypothesis::isSpurious(float max_covariance)
{
  return exceedsMaxCovariance(getCovariance(), max_covariance);
}

void Hypothesis::verifyStatic(Eigen::Array3f& min_corner_detection,
                              Eigen::Array3f& max_corner_detection)
{
  if(m_is_static)
  {
    // TODO: make sure object is visible. If part of object is occluded the centroid moves -> object becomes mistakenly dynamic. see issues for more informations

    float min_overlap_of_initial_bounding_box = 0.95f;

    double init_volume = (m_max_corner_init_hypothesis - m_min_corner_init_hypothesis).prod();

    // expand detection bounding box a little to prevent wrong classification due to sensor noise
    Eigen::Array3f expanded_min_corner_detection = min_corner_detection - 0.03f;
    Eigen::Array3f expanded_max_corner_detection = max_corner_detection + 0.03f;

    Eigen::Array3f intersection_min_corner = expanded_min_corner_detection.max(m_min_corner_init_hypothesis);
    Eigen::Array3f intersection_max_corner = expanded_max_corner_detection.min(m_max_corner_init_hypothesis);

    Eigen::Array3f diff = (intersection_max_corner - intersection_min_corner).max(Eigen::Array3f::Zero());

    float intersection_volume = diff.prod();

    // if bounding boxes do not intersect -> object is dynamic
    if(intersection_volume < 0.1e-8)
    {
      m_is_static = false;
    }
      // else assume it's static for now
    else
    {
      // if detection BB encloses init BB, update init BB as it's likely that object was occluded
      if(intersection_volume / init_volume >= min_overlap_of_initial_bounding_box)
      {
        m_min_corner_init_hypothesis = min_corner_detection.min(m_min_corner_init_hypothesis);
        m_max_corner_init_hypothesis = max_corner_detection.max(m_max_corner_init_hypothesis);
      }
    }
  }
}

void Hypothesis::verifyStatic()
{
  if(m_is_static)
  {
    // Compute just distance in xy direction and don't account for movement in z direction
    Eigen::Vector3f xy_difference = getPosition() - m_first_position_in_track;
    xy_difference.z() = 0.f;

    double distance_from_origin = xy_difference.norm();
    if(distance_from_origin > m_static_distance_threshold)
    {
      m_is_static = false;
    }
  }
}


std::shared_ptr<Hypothesis> HypothesisFactory::createHypothesis(const Detection& detection,
                                                                const unsigned int id,
                                                                const double time_stamp)
{
  return std::make_shared<Hypothesis>(detection, id, time_stamp, m_covariance_per_second);
}

};

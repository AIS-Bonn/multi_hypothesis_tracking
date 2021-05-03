/** @file
 *
 * Kalman filter implementation
 *
 * @author Jan Razlaw
 */

#include "multi_hypothesis_tracking/kalman_filter.h"

namespace MultiHypothesisTracker
{

KalmanFilter::KalmanFilter(const Eigen::VectorXf& initial_position)
  : m_detection_dimensions(initial_position.size())
    , m_control_dimensions(1)
    , m_process_noise_covariance_per_second(0.5f)
{
  m_state_dimensions = (int)initial_position.size() * 2;   // position dimensions + velocity dimensions
  m_state = Eigen::VectorXf(m_state_dimensions);
  m_state.setZero();
  for(int i = 0; i < m_detection_dimensions; i++)
    m_state(i) = initial_position(i);

  
  // next_state = m_state_transition_model * m_state + m_control_input_model * control + process_noise  with process_noise ~ N(0,m_process_noise_covariance)
  m_state_transition_model.resize(m_state_dimensions, m_state_dimensions);
  m_state_transition_model.setIdentity();

  m_control_input_model = Eigen::MatrixXf(m_state_dimensions, m_control_dimensions);
  m_control_input_model.setZero();

  m_process_noise_covariance = Eigen::MatrixXf(m_state_dimensions, m_state_dimensions);
  m_process_noise_covariance.setIdentity();


  // detection = m_observation_model * current_state + observation_noise  with observation_noise ~ N(0,m_observation_noise_covariance)
  m_observation_model = Eigen::MatrixXf(m_detection_dimensions, m_state_dimensions);
  m_observation_model.setZero();

  m_observation_noise_covariance = Eigen::MatrixXf(m_detection_dimensions, m_detection_dimensions);
  m_observation_noise_covariance.setIdentity();


  m_error_covariance = Eigen::MatrixXf(m_state_dimensions, m_state_dimensions);
  m_error_covariance.setIdentity();
}

void KalmanFilter::predict(float dt)
{
  Eigen::VectorXf control(m_control_dimensions);
  control.setZero();
  predict(dt, control);
}

void KalmanFilter::predict(float dt,
                           const Eigen::VectorXf& control)
{
  // set up state transition model
  m_state_transition_model.setIdentity();
  m_state_transition_model(0, 3) = dt;
  m_state_transition_model(1, 4) = dt;
  m_state_transition_model(2, 5) = dt;

  // set up control input model - here not used
  m_control_input_model.setZero();

  // update state according to models
  m_state = m_state_transition_model * m_state + m_control_input_model * control;

  // set up process_noise_covariance
  for(size_t i = 0; i < m_state_dimensions; i++)
    m_process_noise_covariance(i, i) = dt * m_process_noise_covariance_per_second;

  // update error covariance
  m_error_covariance =
    m_state_transition_model * m_error_covariance * m_state_transition_model.transpose() + m_process_noise_covariance;

  // check if cov matrix is symmetric as is should be
  if(!isAlmostSymmetric(m_error_covariance))
    std::cout << "KalmanFilter::predictNextHypothesesStates: m_error_covariance is not symmetric!!!!!" << std::endl;

  // TODO: check if matrix is positive definite ?!?
}

void KalmanFilter::correct(const Eigen::VectorXf& detection_position,
                           const Eigen::MatrixXf& detection_covariance)
{
  assert(detection_position.size() == m_detection_dimensions);

  // set up observation model
  m_observation_model.setZero();
  for(size_t i = 0; i < m_detection_dimensions; i++)
    m_observation_model(i, i) = 1.f;

  // set up observation noise covariance
  m_observation_noise_covariance = detection_covariance;

  // compute kalman gain
  Eigen::MatrixXf temp = m_error_covariance * m_observation_model.transpose();
  Eigen::MatrixXf kalman_gain = temp * (m_observation_model * temp + m_observation_noise_covariance).inverse();


  // compute the expected position
  Eigen::VectorXf expected_position = m_observation_model * m_state;

  // correct state
  m_state = m_state + kalman_gain * (detection_position - expected_position);


  // update error covariance
  Eigen::MatrixXf identity(kalman_gain.rows(), m_observation_model.cols());
  identity.setIdentity();
  m_error_covariance = (identity - kalman_gain * m_observation_model) * m_error_covariance;


  // TODO: check if calculations are responsible for need for epsilon - if calculations are right, make covariance matrix symmetric by P = (P + P^T) / 2
  // check if cov matrix is symmetric as it should be
  if(!isAlmostSymmetric(m_error_covariance))
    std::cout << "KalmanFilter::correct: m_error_covariance is not symmetric!!!!!\n" << m_error_covariance << std::endl;
}

bool KalmanFilter::isAlmostSymmetric(const Eigen::MatrixXf& matrix,
                                     float epsilon)
{
  for(int i = 1; i < matrix.rows(); i++)
    for(int j = i; j < matrix.cols(); j++)
      if(fabsf(matrix(i, j) - matrix(j, i)) > epsilon)
        return false;

  return true;
}

};

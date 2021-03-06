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
  : m_number_of_detection_positions_dimensions(initial_position.size())
    , m_number_of_control_dimensions(1)
    , m_process_noise_covariance_per_second(0.5f)
{
  m_number_of_state_dimensions = (int)initial_position.size() * 2;   // position dimensions + velocity dimensions
  setUpInitialState(initial_position);
  setUpMatricesForPrediction();
  setUpMatricesForCorrection();

  m_error_covariance = Eigen::MatrixXf(m_number_of_state_dimensions, m_number_of_state_dimensions);
  m_error_covariance.setIdentity();
}

void KalmanFilter::setUpInitialState(const Eigen::VectorXf& initial_position)
{
  m_state = Eigen::VectorXf(m_number_of_state_dimensions);
  m_state.setZero();
  for(size_t i = 0; i < m_number_of_detection_positions_dimensions; i++)
    m_state(i) = initial_position(i);
}

void KalmanFilter::setUpMatricesForPrediction()
{
  // next_state = m_state_transition_model * m_state + m_control_input_model * control + process_noise  with process_noise ~ N(0,m_process_noise_covariance)
  m_state_transition_model = Eigen::MatrixXf(m_number_of_state_dimensions, m_number_of_state_dimensions);
  m_state_transition_model.setIdentity();

  m_control_input_model = Eigen::MatrixXf(m_number_of_state_dimensions, m_number_of_control_dimensions);
  m_control_input_model.setZero();

  m_process_noise_covariance = Eigen::MatrixXf(m_number_of_state_dimensions, m_number_of_state_dimensions);
  m_process_noise_covariance.setIdentity();
}

void KalmanFilter::setUpMatricesForCorrection()
{
  // detection = m_observation_model * current_state + observation_noise  with observation_noise ~ N(0,m_observation_noise_covariance)
  m_observation_model = Eigen::MatrixXf(m_number_of_detection_positions_dimensions, m_number_of_state_dimensions);
  m_observation_model.setZero();
  for(size_t i = 0; i < m_number_of_detection_positions_dimensions; i++)
    m_observation_model(i, i) = 1.f;
  
  m_observation_noise_covariance = Eigen::MatrixXf(m_number_of_detection_positions_dimensions, m_number_of_detection_positions_dimensions);
  m_observation_noise_covariance.setIdentity();
}

void KalmanFilter::predict(float time_difference)
{
  Eigen::VectorXf control(m_number_of_control_dimensions);
  control.setZero();
  predict(time_difference, control);
}

void KalmanFilter::predict(float time_difference,
                           const Eigen::VectorXf& control)
{
  predictState(time_difference, control);
  predictErrorCovariance(time_difference);

  isErrorCovarianceValid();
}

void KalmanFilter::predictState(float time_difference,
                                const Eigen::VectorXf& control)
{
  m_state_transition_model.setIdentity();
  m_state_transition_model(0, 3) = time_difference;
  m_state_transition_model(1, 4) = time_difference;
  m_state_transition_model(2, 5) = time_difference;

  // set up control input model - here not used
  m_control_input_model.setZero();

  m_state = m_state_transition_model * m_state + m_control_input_model * control;
}

void KalmanFilter::predictErrorCovariance(float time_difference)
{
  for(size_t i = 0; i < m_number_of_state_dimensions; i++)
    m_process_noise_covariance(i, i) = time_difference * m_process_noise_covariance_per_second;

  m_error_covariance =
    m_state_transition_model * m_error_covariance * m_state_transition_model.transpose() + m_process_noise_covariance;
}

void KalmanFilter::isErrorCovarianceValid()
{
  // A valid error covariance matrix is positive definite
  if(!m_error_covariance.ldlt().isPositive())
    std::cout << "KalmanFilter: m_error_covariance is not positive!\n" << m_error_covariance << std::endl;
}

void KalmanFilter::correct(const Eigen::VectorXf& detection_position,
                           const Eigen::MatrixXf& detection_covariance)
{
  assert(detection_position.size() == m_number_of_detection_positions_dimensions);

  computeKalmanGain(detection_covariance);
  correctState(detection_position);
  correctErrorCovariance();

  isErrorCovarianceValid();
}

void KalmanFilter::computeKalmanGain(const Eigen::MatrixXf& detection_covariance)
{
  m_observation_noise_covariance = detection_covariance;

  Eigen::MatrixXf temp = m_error_covariance * m_observation_model.transpose();
  m_kalman_gain = temp * (m_observation_model * temp + m_observation_noise_covariance).inverse();
}

void KalmanFilter::correctState(const Eigen::VectorXf& detection_position)
{
  Eigen::VectorXf expected_position = m_observation_model * m_state;
  m_state = m_state + m_kalman_gain * (detection_position - expected_position);
}

void KalmanFilter::correctErrorCovariance()
{
  Eigen::MatrixXf identity(m_kalman_gain.rows(), m_observation_model.cols());
  identity.setIdentity();
  m_error_covariance = (identity - m_kalman_gain * m_observation_model) * m_error_covariance;
}

};

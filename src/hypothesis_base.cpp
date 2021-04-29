/** @file
 *
 * Basis hypothesis providing functionality every type of hypothesis should have.
 *
 * @author Jan Razlaw
 */

#include "multi_hypothesis_tracking/hypothesis_base.h"

namespace MultiHypothesisTracker
{

HypothesisBase::HypothesisBase(const Detection& detection,
                               unsigned int id,
                               double time_stamp,
                               float kalman_process_noise_covariance_per_second)
  : m_id(id)
    , m_time_stamp_of_birth(time_stamp)
    , m_is_static(true)
    , m_static_distance_threshold(1.f)
    , m_number_of_assignments(0)
{
  m_kalman_filter = std::make_shared<KalmanFilter>(detection.position);
  m_kalman_filter->setProcessNoiseCovariancePerSecond(kalman_process_noise_covariance_per_second);

  m_position_history.push_back(detection.position);
  m_was_assigned_history.push_back(true);
}

void HypothesisBase::predict(float dt)
{
  m_kalman_filter->predict(dt);
  updateHypothesisAfterPrediction();
}

void HypothesisBase::updateHypothesisAfterPrediction()
{
  const auto& predicted_position = getPosition();
  m_position_history.push_back(predicted_position);
  m_was_assigned_history.push_back(false);
}

void HypothesisBase::correct(const Detection& detection)
{
  m_kalman_filter->correct(detection.position,
                           detection.covariance);
  updateHypothesisAfterCorrection();
}

void HypothesisBase::updateHypothesisAfterCorrection()
{
  const auto& corrected_position = getPosition();

  // replace the latest predicted position by the corrected position
  m_position_history.back() = corrected_position;
  m_was_assigned_history.back() = true;
  m_number_of_assignments++;

  verifyStatic();
}

bool HypothesisBase::exceedsMaxCovariance(const Eigen::Matrix3f& covariance,
                                          float max_covariance)
{
  Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(covariance);
  auto eigen_values = eigen_solver.eigenvalues();

  return (eigen_values.col(0)[0].real() > max_covariance ||
          eigen_values.col(0)[1].real() > max_covariance ||
          eigen_values.col(0)[2].real() > max_covariance);
}

bool HypothesisBase::isSpurious(float max_covariance)
{
  return exceedsMaxCovariance(getCovariance(), max_covariance);
}

void HypothesisBase::verifyStatic()
{
  if(m_is_static)
  {
    double distance_from_initial_position = (getPosition() - getInitialPosition()).norm();
    if(distance_from_initial_position > m_static_distance_threshold)
      m_is_static = false;
  }
}

};

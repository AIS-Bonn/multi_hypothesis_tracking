/** @file
 *
 * Basis hypothesis providing functionality every type of hypothesis should have.
 *
 * @author Jan Razlaw
 */

#include "multi_hypothesis_tracking/hypotheses/hypothesis_base.h"

namespace MultiHypothesisTracker
{

HypothesisBase::HypothesisBase(const Detection& detection,
                               unsigned int id,
                               double time_stamp)
  : m_id(id)
    , m_time_stamp_of_birth(time_stamp)
    , m_do_verify_static(true)
    , m_is_static(true)
    , m_static_distance_threshold(1.f)
    , m_maximally_allowed_hypothesis_covariance(5.f)
{
  m_kalman_filter = std::make_shared<KalmanFilter>(detection.position);

  initHistory(detection);
}

void HypothesisBase::initHistory(const Detection& detection)
{
  m_history.position_history.push_back(detection.position);
  m_history.assignment_history.push_back(true);
  m_history.number_of_assignments = 1;
}

void HypothesisBase::predict(float dt)
{
  m_kalman_filter->predict(dt);
  updateHistoryAfterPrediction();
}

void HypothesisBase::updateHistoryAfterPrediction()
{
  const auto& predicted_position = getPosition();
  m_history.position_history.push_back(predicted_position);
  m_history.assignment_history.push_back(false);
}

void HypothesisBase::correct(const Detection& detection)
{
  m_kalman_filter->correct(detection.position,
                           detection.covariance);
  updateHistoryAfterCorrection();
  if(m_do_verify_static)
    verifyStatic();
}

void HypothesisBase::updateHistoryAfterCorrection()
{
  const auto& corrected_position = getPosition();

  // replace the latest predicted position by the corrected position
  m_history.position_history.back() = corrected_position;
  m_history.assignment_history.back() = true;
  m_history.number_of_assignments++;
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

bool HypothesisBase::isWeak()
{
  return exceedsMaxCovariance(getCovariance(), m_maximally_allowed_hypothesis_covariance);
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

};

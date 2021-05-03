/** @file
 *
 * Kalman filter implementation
 *
 * @author Jan Razlaw
 */

#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

#include <iostream>

// uncomment to disable assert()
// #define NDEBUG
#include <cassert>

namespace MultiHypothesisTracker
{
/**
 * @brief Kalman filter class
 * Naming of variables corresponds to wikipedia page.
 */
class KalmanFilter
{
public:

  explicit KalmanFilter(const Eigen::VectorXf& initial_position);
  virtual ~KalmanFilter() = default;

  /**
   * @brief Predicts the position after dt seconds without a control.
   *
   * @param[in] dt  time delta that has passed
   * @see predictNextHypothesesStates(float, const Eigen::VectorXf&)
   */
  void predict(float dt);

  /**
   * @brief Predicts the position after dt seconds.
   *
   * @param[in] dt      time delta that has passed
   * @param[in] control control vector that is applied during prediction
   */
  void predict(float dt,
               const Eigen::VectorXf& control);

  /**
   * @brief Corrects the position using the position of current detection.
   *
   * @param[in] detection_position    position of current detection.
   * @param[in] detection_covariance  covariance of current detection.
   */
  void correct(const Eigen::VectorXf& detection_position,
               const Eigen::MatrixXf& detection_covariance);

  /**
   * @brief Getter for m_state.
   * @return current state of filter.
   */
  Eigen::VectorXf& getState(){ return m_state; };

  /**
   * @brief Getter for m_error_covariance.
   * @return current error covariance matrix P.
   */
  Eigen::MatrixXf& getErrorCovariance(){ return m_error_covariance; };

  /** @brief Setter for #m_process_noise_covariance_per_second. */
  void setProcessNoiseCovariancePerSecond(float covariance_per_second)
  { 
    m_process_noise_covariance_per_second = covariance_per_second; 
  };

protected:

  /**
   * @brief Checks if a Matrix is at least almost symmetrical.
   *
   * @param matrix  matrix to be checked
   * @param epsilon maximally allowed difference between corresponding entries
   *
   * @return true if matrix is at least almost symmetrical, false otherwise
   */
  bool isAlmostSymmetric(const Eigen::MatrixXf& matrix,
                         float epsilon = 0.001f);

  /** @brief Vector encoding the current state x. */
  Eigen::VectorXf m_state;

  /** @brief State transition model F. */
  Eigen::MatrixXf m_state_transition_model;
  /** @brief Control input model B. */
  Eigen::MatrixXf m_control_input_model;
  /** @brief Observation model H. */
  Eigen::MatrixXf m_observation_model;

  /** @brief Error covariance matrix P. */
  Eigen::MatrixXf m_error_covariance;
  /** @brief Process noise covariance matrix Q. */
  Eigen::MatrixXf m_process_noise_covariance;
  /** @brief Observation noise covariance matrix R. */
  Eigen::MatrixXf m_observation_noise_covariance;

  /** @brief Number of state dimensions. */
  size_t m_state_dimensions;
  /** @brief Number of dimensions of the detections position or state. */
  size_t m_detection_dimensions;
  /** @brief Number of conrol dimensions. */
  size_t m_control_dimensions;

  /** @brief Covariance that is added to the #m_error_covariance per second of prediction without correction. */
  float m_process_noise_covariance_per_second;
};

};

#endif //__KALMAN_FILTER_H__

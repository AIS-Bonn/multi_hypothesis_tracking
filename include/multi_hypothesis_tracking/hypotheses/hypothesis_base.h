/** @file
 *
 * Basis hypothesis providing functionality every type of hypothesis should have.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_BASE_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_BASE_H

#include <memory> // for std::shared_ptr

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypotheses/hypothesis_interface.h>
#include <multi_hypothesis_tracking/kalman_filter.h>

namespace MultiHypothesisTracker
{

/**
 * @brief Basic hypothesis class used for tracking.
 */
class HypothesisBase : public HypothesisInterface
{
public:
  /**
   * @brief Constructor.
   *
   * @param[in] detection                                   initial state.
   * @param[in] id                                          unique id assigned to this hypothesis.
   * @param[in] time_stamp                                  time stamp when the detection was created.
   */
  HypothesisBase(const Detection& detection,
                 unsigned int id,
                 double time_stamp);
  ~HypothesisBase() override = default;

  // Methods implementing the interface

  /** @brief Predicts the state after time_difference without a control input using a kalman filter. */
  void predict(float time_difference) override;

  /** @brief Corrects the hypothesis' state using the detection. */
  void correct(const Detection& detection) override;

  inline Eigen::Vector3f getPosition() override
  {
    return m_kalman_filter->getState().block<3, 1>(0, 0);
  }
  inline Eigen::Matrix3f getCovariance() override
  {
    return m_kalman_filter->getErrorCovariance().block<3, 3>(0, 0);
  }

  /** @brief Currently calls exceedsMaxCovariance to check if hypothesis is spurious. */
  bool isWeak() override;

  
  // Additional methods

  /** @brief Getter for unique hypothesis ID. */
  inline unsigned int getID() const{ return m_id; }

  inline Eigen::Vector3f getVelocity(){ return m_kalman_filter->getState().block<3, 1>(3, 0); }

  /** @brief Getter for time stamp of hypothesis initialization. */
  inline double getTimeStampOfBirth() const{ return m_time_stamp_of_birth; }

  inline Eigen::Vector3f& getInitialPosition(){ return m_position_history[0]; }

  inline std::vector<Eigen::Vector3f>& getPositionHistory(){ return m_position_history; }

  /** @brief Getter for #m_was_assigned_history. 
   * 
   * The "was assigned history" consists of a vector with the same size as the #m_position_history.
   * If an entry in the assigned history is false, the position with the same index was only predicted, 
   * but not corrected by an assigned detection. 
   * If an entry is true, the position was predicted and corrected. */
  inline std::vector<bool>& getWasAssignedHistory(){ return m_was_assigned_history; }
  
  inline int getNumberOfAssignments() const{ return m_number_of_assignments; }

  void setProcessNoiseCovariancePerSecond(float covariance_per_second)
  {
    m_kalman_filter->setProcessNoiseCovariancePerSecond(covariance_per_second);
  }
  
  void setMaxAllowedHypothesisCovariance(float maximally_allowed_hypothesis_covariance)
  {
    m_maximally_allowed_hypothesis_covariance = maximally_allowed_hypothesis_covariance;
  }
  
  /** @brief Getter for static property. */
  inline bool isStatic() const { return m_is_static; }
  
protected:
  /**
   * @brief Checks if covariance exceeds max_covariance.
   *
   * The eigen values are computed to have a rotation invariant check. 
   * 
   * @param covariance      covariance matrix that is checked.
   * @param max_covariance  covariance threshold.
   *
   * @return true if exceeds, false otherwise
   */
  static bool exceedsMaxCovariance(const Eigen::Matrix3f& covariance,
                                   float max_covariance);

  /** @brief Update #m_is_static by checking if hypothesis is moved further than #m_static_distance_threshold from its 
   * initial position. */
  void verifyStatic();

  void updateHypothesisAfterPrediction();
  void updateHypothesisAfterCorrection();

  /** @brief Kalman filter for state estimation. */
  std::shared_ptr<KalmanFilter> m_kalman_filter;

  /** @brief Unique hypothesis ID. */
  unsigned int m_id;
  double m_time_stamp_of_birth;

  bool m_is_static;
  /** @brief Distance a hypothesis is allowed to move to still be considered static. */
  double m_static_distance_threshold;

  /** @brief Hypothesis is deleted if one eigen value of its covariance matrix is greater than this parameter. */
  float m_maximally_allowed_hypothesis_covariance;
  
  /** @brief All positions of the hypothesis. */
  std::vector<Eigen::Vector3f> m_position_history;
  /** @brief Vector of bools indicating whether this hypothesis was assigned to a detection in the corresponding step.*/
  std::vector<bool> m_was_assigned_history;
  /** @brief The number of times this hypothesis was assigned to a detection. */
  int m_number_of_assignments;
};

};

#endif 

/** @file
 *
 * Implementation of a hypothesis for objects measured with sparse lidars.
 * Based on a HypothesisWithBoundingBox and extending the class with a method to cap the velocity.
 * This is useful for detections with a lot of noise in size and position.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_SPARSE_LIDAR_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_SPARSE_LIDAR_H

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypotheses/hypothesis_with_bounding_box.h>
#include <multi_hypothesis_tracking/utils.h>

namespace MultiHypothesisTracker
{

/**
 * @brief Hypothesis for objects measured with sparse lidars.
 */
class HypothesisForSparseLidar : public HypothesisWithBoundingBox
{
public:
  /**
   * @brief Constructor.
   *
   * @param[in] detection    provides initial state.
   * @param[in] id           unique id assigned to this hypothesis.
   * @param[in] time_stamp   time stamp when the detection was created.
   */
  HypothesisForSparseLidar(const Detection& detection,
                           unsigned int id,
                           double time_stamp,
                           float min_valid_velocity = 0.3f,
                           float max_allowed_velocity = 2.8f);
  ~HypothesisForSparseLidar() override = default;

  /** @brief Predicts the next state and updates bounding box using transform from previous to next state. */
  void predict(float time_difference) override;

  /** @brief Corrects the hypothesis' state using the detection and updates the bounding box. */
  void correct(const Detection& detection) override;

  void setMinValidVelocity(float min_valid_velocity){ m_min_valid_velocity = min_valid_velocity; }
  void setMaxAllowedVelocity(float max_allowed_velocity){ m_max_allowed_velocity = max_allowed_velocity; }

protected:
  void capVelocity();

  /** @brief Lower velocities in meters per second will be set to zero - for sparse lidars low velocities are often 
   * erroneously induced by noisy measurements. */
  float m_min_valid_velocity;
  /** @brief Upper velocity cap in meters per second - for sparse lidars extremely high velocities can be erroneously 
   * induced by a large distance between scan rings. */
  float m_max_allowed_velocity;
};

};

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_SPARSE_LIDAR_H

/** @file
 *
 * Factory for hypotheses using detections from sparse lidar data. 
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_SPARSE_LIDAR_FACTORY_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_SPARSE_LIDAR_FACTORY_H

#include <multi_hypothesis_tracking/hypotheses/hypothesis_for_sparse_lidar.h>
#include <multi_hypothesis_tracking/hypotheses/hypothesis_with_bounding_box_factory.h>

namespace MultiHypothesisTracker
{

class HypothesisForSparseLidarFactory : public HypothesisWithBoundingBoxFactory
{
public:
  /** @brief Creates hypothesis suitable for tracking with data from sparse lidars.
   *
   * @param[in] detection       detection to initialize the hypothesis' state.
   * @param[in] time_stamp      time stamp when the detection was created.
   *
   * @return pointer to created hypothesis.
   */
  std::shared_ptr<HypothesisInterface> createHypothesis(const Detection& detection,
                                                        const double time_stamp) override
  {
    auto hypothesis = std::make_shared<HypothesisForSparseLidar>(detection,
                                                                 m_number_of_created_hypotheses++,
                                                                 time_stamp,
                                                                 m_min_valid_velocity,
                                                                 m_max_allowed_velocity);

    hypothesis->setProcessNoiseCovariancePerSecond(m_kalman_process_noise_covariance_per_second);
    hypothesis->setMaxAllowedHypothesisCovariance(m_maximally_allowed_hypothesis_covariance);

    hypothesis->setAccumulateDetectionPointsInHypothesis(m_accumulate_detection_points_in_hypothesis);

    hypothesis->setUpdateStaticStatusUsingBoundingBoxIntersection(
      m_update_static_status_using_bounding_box_intersection);

    return hypothesis;
  };

  inline void setMinValidVelocity(float min_valid_velocity){ m_min_valid_velocity = min_valid_velocity; }
  inline void setMaxAllowedVelocity(float max_allowed_velocity){ m_max_allowed_velocity = max_allowed_velocity; }

  /** @brief Lower velocities will be set to zero - for sparse lidars low velocities are often erroneously induced by 
   * noisy measurements. */
  float m_min_valid_velocity;
  /** @brief Upper velocity cap - for sparse lidars extremely high velocities can be erroneously induced by a large 
   * distance between scan rings. */
  float m_max_allowed_velocity;
};

}

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_SPARSE_LIDAR_FACTORY_H

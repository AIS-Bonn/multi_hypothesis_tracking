/** @file
 *
 * Factory for basic hypotheses. 
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_BASE_FACTORY_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_BASE_FACTORY_H

#include <multi_hypothesis_tracking/hypothesis_base.h>
#include <multi_hypothesis_tracking/hypothesis_factory_interface.h>

namespace MultiHypothesisTracker
{

class HypothesisBaseFactory : public HypothesisFactoryInterface
{
public:
  /** @brief Creates basic hypothesis.
   *
   * @param[in] detection       detection to initialize the hypothesis' state.
   * @param[in] time_stamp      time stamp when the detection was created.
   *
   * @return pointer to created basic hypothesis.
   */
  std::shared_ptr<HypothesisInterface> createHypothesis(const Detection& detection,
                                                        const double time_stamp) override
  {
    auto hypothesis = std::make_shared<HypothesisBase>(detection,
                                                       m_number_of_created_hypotheses++,
                                                       time_stamp);

    hypothesis->setProcessNoiseCovariancePerSecond(m_kalman_process_noise_covariance_per_second);
    hypothesis->setMaxAllowedHypothesisCovariance(m_maximally_allowed_hypothesis_covariance);
    
    return hypothesis;
  };

  inline void setKalmanProcessNoiseCovariancePerSecond(float covariance_per_second)
  {
    m_kalman_process_noise_covariance_per_second = covariance_per_second;
  }

  inline void setMaxAllowedHypothesisCovariance(float maximally_allowed_hypothesis_covariance)
  {
    m_maximally_allowed_hypothesis_covariance = maximally_allowed_hypothesis_covariance;
  }
  
  /** @brief Process noise covariance per second for kalman filter.*/
  float m_kalman_process_noise_covariance_per_second = 0.5f;
  float m_maximally_allowed_hypothesis_covariance = 5.f;
};

}

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_BASE_FACTORY_H

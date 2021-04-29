/** @file
 *
 * Interface for every hypothesis factory with methods that have to be implemented. 
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FACTORY_INTERFACE_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FACTORY_INTERFACE_H

#include <multi_hypothesis_tracking/hypothesis_interface.h>

namespace MultiHypothesisTracker
{

class HypothesisFactoryInterface
{
public:
  virtual ~HypothesisFactoryInterface() = default;

  /** @brief Creates hypothesis.
   *
   * @param[in] detection       initial state.
   * @param[in] id              ID of created hypothesis.
   * @param[in] time_stamp      time stamp when the detection was created.
   *
   * @return pointer to created hypothesis.
   */
  virtual std::shared_ptr<HypothesisInterface> createHypothesis(const Detection& detection,
                                                                double time_stamp) = 0;


  inline float getKalmanCovariancePerSecond()
  {
    return m_covariance_per_second;
  }

  inline void setKalmanCovariancePerSecond(float covariance_per_second)
  {
    m_covariance_per_second = covariance_per_second;
  }

  /** @brief Covariance per second for kalman filter.*/
  float m_covariance_per_second = 0.5f;

  int m_number_of_created_hypotheses = 0;
};

}

#endif

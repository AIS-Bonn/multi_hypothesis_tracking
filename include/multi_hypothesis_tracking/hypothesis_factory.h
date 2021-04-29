/** @file
 *
 * Factory for hypotheses. 
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FACTORY_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FACTORY_H

#include <multi_hypothesis_tracking/hypothesis.h>
#include <multi_hypothesis_tracking/hypothesis_factory_interface.h>

namespace MultiHypothesisTracker
{

class HypothesisFactory : public HypothesisFactoryInterface
{
public:
  /** @brief Creates hypothesis.
   *
   * @param[in] detection       detection to initialize the hypothesis' state.
   * @param[in] id              unique ID of created hypothesis.
   * @param[in] time_stamp      time stamp when the detection was created.
   *
   * @return pointer to created hypothesis.
   */
  std::shared_ptr<HypothesisInterface> createHypothesis(const Detection& detection,
                                                        const unsigned int id,
                                                        const double time_stamp) override
  {
    return std::make_shared<Hypothesis>(detection, id, time_stamp, m_covariance_per_second);
  };

};

}

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FACTORY_H

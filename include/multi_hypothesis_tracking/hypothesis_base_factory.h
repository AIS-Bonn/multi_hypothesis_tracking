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
   * @param[in] id              unique ID of created hypothesis.
   * @param[in] time_stamp      time stamp when the detection was created.
   *
   * @return pointer to created basic hypothesis.
   */
  std::shared_ptr<HypothesisInterface> createHypothesis(const Detection& detection,
                                                        const unsigned int id,
                                                        const double time_stamp) override
  {
    return std::make_shared<HypothesisBase>(detection, id, time_stamp, m_covariance_per_second);
  };

};

}

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_BASE_FACTORY_H

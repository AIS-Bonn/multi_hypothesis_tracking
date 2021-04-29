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
   * @param[in] time_stamp      time stamp when the detection was created.
   *
   * @return pointer to created hypothesis.
   */
  virtual std::shared_ptr<HypothesisInterface> createHypothesis(const Detection& detection,
                                                                double time_stamp) = 0;

  int m_number_of_created_hypotheses = 0;
};

}

#endif

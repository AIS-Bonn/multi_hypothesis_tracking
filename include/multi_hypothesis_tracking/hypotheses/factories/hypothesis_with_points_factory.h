/** @file
 *
 * Factory for hypotheses with points. 
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_POINTS_FACTORY_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_POINTS_FACTORY_H

#include <multi_hypothesis_tracking/hypotheses/hypothesis_with_points.h>
#include <multi_hypothesis_tracking/hypotheses/factories/hypothesis_base_factory.h>

namespace MultiHypothesisTracker
{

class HypothesisWithPointsFactory : public HypothesisBaseFactory
{
public:
  /** @brief Creates hypothesis with points.
   *
   * @param[in] detection       detection to initialize the hypothesis' state.
   * @param[in] time_stamp      time stamp when the detection was created.
   *
   * @return pointer to created hypothesis.
   */
  std::shared_ptr<HypothesisInterface> createHypothesis(const Detection& detection,
                                                        const double time_stamp) override
  {
    auto hypothesis = std::make_shared<HypothesisWithPoints>(detection,
                                                             m_number_of_created_hypotheses++,
                                                             time_stamp,
                                                             m_accumulate_detection_points_in_hypothesis);

    hypothesis->setProcessNoiseCovariancePerSecond(m_kalman_process_noise_covariance_per_second);
    hypothesis->setMaxAllowedHypothesisCovariance(m_maximally_allowed_hypothesis_covariance);

    return hypothesis;
  };

  inline void setAccumulateDetectionPointsInHypothesis(bool do_accumulate)
  {
    m_accumulate_detection_points_in_hypothesis = do_accumulate;
  }

  /** @brief If true, points are accumulated in hypothesis and transformed according to state updates.
   * If false, only latest detection points are stored as they are. */
  float m_accumulate_detection_points_in_hypothesis = false;
};

}

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_POINTS_FACTORY_H

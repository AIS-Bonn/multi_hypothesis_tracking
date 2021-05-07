/** @file
 *
 * Factory for hypotheses for human poses. 
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_HUMAN_POSE_FACTORY_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_HUMAN_POSE_FACTORY_H

#include <multi_hypothesis_tracking/hypotheses/hypothesis_for_human_pose.h>
#include <multi_hypothesis_tracking/hypotheses/factories/hypothesis_with_bounding_box_factory.h>

namespace MultiHypothesisTracker
{

class HypothesisForHumanPoseFactory : public HypothesisWithBoundingBoxFactory
{
public:
  /** @brief Creates hypothesis suitable for tracking a human pose.
   *
   * @param[in] detection       detection to initialize the hypothesis' state.
   * @param[in] time_stamp      time stamp when the detection was created.
   *
   * @return pointer to created hypothesis.
   */
  std::shared_ptr<HypothesisInterface> createHypothesis(const Detection& detection,
                                                        const double time_stamp) override
  {
    auto hypothesis = std::make_shared<HypothesisForHumanPose>(detection,
                                                               m_number_of_created_hypotheses++,
                                                               time_stamp);

    hypothesis->setProcessNoiseCovariancePerSecond(m_kalman_process_noise_covariance_per_second);
    hypothesis->setMaxAllowedHypothesisCovariance(m_maximally_allowed_hypothesis_covariance);

    return hypothesis;
  };

};

}

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_HUMAN_POSE_FACTORY_H

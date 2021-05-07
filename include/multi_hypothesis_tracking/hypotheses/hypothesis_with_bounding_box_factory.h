/** @file
 *
 * Factory for hypotheses with bounding boxes. 
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_BOUNDING_BOX_FACTORY_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_BOUNDING_BOX_FACTORY_H

#include <multi_hypothesis_tracking/hypotheses/hypothesis_with_bounding_box.h>
#include <multi_hypothesis_tracking/hypotheses/hypothesis_with_points_factory.h>

namespace MultiHypothesisTracker
{

class HypothesisWithBoundingBoxFactory : public HypothesisWithPointsFactory
{
public:
  /** @brief Creates hypothesis with bounding boxes.
   *
   * @param[in] detection       detection to initialize the hypothesis' state.
   * @param[in] time_stamp      time stamp when the detection was created.
   *
   * @return pointer to created hypothesis.
   */
  std::shared_ptr<HypothesisInterface> createHypothesis(const Detection& detection,
                                                        const double time_stamp) override
  {
    auto hypothesis = std::make_shared<HypothesisWithBoundingBox>(detection,
                                                                  m_number_of_created_hypotheses++,
                                                                  time_stamp,
                                                                  m_update_static_status_using_bounding_box_intersection);

    hypothesis->setProcessNoiseCovariancePerSecond(m_kalman_process_noise_covariance_per_second);
    hypothesis->setMaxAllowedHypothesisCovariance(m_maximally_allowed_hypothesis_covariance);

    hypothesis->setAccumulateDetectionPointsInHypothesis(m_accumulate_detection_points_in_hypothesis);

    return hypothesis;
  };

  inline void
  setUpdateStaticStatusUsingBoundingBoxIntersection(bool update_static_status_using_bounding_box_intersection)
  {
    m_update_static_status_using_bounding_box_intersection = update_static_status_using_bounding_box_intersection;
  }

  /** @brief If true, @see verifyStaticUsingBoundingBoxes() is used to update #m_is_static, 
   * otherwise the verifyStatic method from the HypothesisBase class will be called instead. */
  bool m_update_static_status_using_bounding_box_intersection;
};

}

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_BOUNDING_BOX_FACTORY_H

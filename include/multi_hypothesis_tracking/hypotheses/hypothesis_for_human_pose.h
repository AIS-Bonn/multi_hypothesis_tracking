/** @file
 *
 * Implementation of a hypothesis for tracking a human pose.
 * Based on a HypothesisWithBoundingBox and extending the class with a separate kalman filter for each joint.
 * This is useful for detections providing human pose estimations.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_HUMAN_POSE_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_HUMAN_POSE_H

#include <memory> // for std::shared_ptr

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypotheses/hypothesis_with_bounding_box.h>
#include <multi_hypothesis_tracking/utils.h>

namespace MultiHypothesisTracker
{

/**
 * @brief Hypothesis class for tracking the pose of a human.
 */
class HypothesisForHumanPose : public HypothesisWithBoundingBox
{
public:
  /**
   * @brief Constructor.
   *
   * @param[in] detection    initial state.
   * @param[in] id           unique id assigned to this hypothesis.
   * @param[in] time_stamp   time stamp when the detection was created.
   */
  HypothesisForHumanPose(const Detection& detection,
                         unsigned int id,
                         double time_stamp);
  ~HypothesisForHumanPose() override = default;

  void disableVerifyingStaticStatus();

  /** @brief Predicts the next state and pose after time_difference seconds. */
  void predict(float time_difference) override;

  /** @brief Corrects the hypothesis' state and joint positions using the detection. */
  void correct(const Detection& detection) override;

  inline std::vector<std::shared_ptr<KalmanFilter>>& getTrackedJoints(){ return m_tracked_joints; }
  std::vector<Eigen::Vector3f> getTrackedJointsPositions();

protected:
  void updateHypothesisAfterPrediction();
  void updateHypothesisAfterCorrection(const Detection& detection);

  std::vector<std::shared_ptr<KalmanFilter>> m_tracked_joints;
};

};

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_HUMAN_POSE_H

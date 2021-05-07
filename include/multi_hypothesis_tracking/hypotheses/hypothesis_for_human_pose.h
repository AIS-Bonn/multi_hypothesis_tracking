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

  inline std::vector<bool>& getIsTrackedJointInitialized(){ return m_is_tracked_joint_initialized; }
  inline std::vector<std::shared_ptr<KalmanFilter>>& getTrackedJoints(){ return m_tracked_joints; }
  inline std::vector<Eigen::Vector3f> getTrackedJointsPositions()
  {
    std::vector<Eigen::Vector3f> joint_positions;
    for(size_t joint_id = 0; joint_id < m_tracked_joints.size(); joint_id++)
    {
      if(m_is_tracked_joint_initialized[joint_id])
        joint_positions.emplace_back(m_tracked_joints[joint_id]->getState().block<3, 1>(0, 0));
    }
    return joint_positions;
  }

protected:
  void updateHypothesisAfterPrediction();
  void updateHypothesisAfterCorrection(const Detection& detection);

  std::vector<bool> m_is_tracked_joint_initialized;
  std::vector<std::shared_ptr<KalmanFilter>> m_tracked_joints;
};

};

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_FOR_HUMAN_POSE_H

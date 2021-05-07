/** @file
 *
 * Implementation of a hypothesis for tracking a human pose.
 * Based on a HypothesisWithBoundingBox and extending the class with a separate kalman filter for each joint.
 * This is useful for detections providing human pose estimations.
 *
 * @author Jan Razlaw
 */

#include "multi_hypothesis_tracking/hypotheses/hypothesis_for_human_pose.h"

namespace MultiHypothesisTracker
{

HypothesisForHumanPose::HypothesisForHumanPose(const Detection& detection,
                                               unsigned int id,
                                               double time_stamp)
  : HypothesisBase(detection, id, time_stamp)
    , HypothesisWithPoints(detection, id, time_stamp, false)
    , HypothesisWithBoundingBox(detection, id, time_stamp)
    , m_tracked_joints(detection.points.size(), nullptr)
{
  // Set static property to false right away, because humans are by default potentially dynamic 
  disableVerifyingStaticStatus();

  for(size_t joint_id = 0; joint_id < detection.points.size(); joint_id++)
    if(isFinite(detection.points[joint_id]))
      m_tracked_joints[joint_id]= std::make_shared<KalmanFilter>(detection.points[joint_id]); 
}

void HypothesisForHumanPose::disableVerifyingStaticStatus()
{
  m_is_static = false;
  setDoVerifyStatic(false);
  setUpdateStaticStatusUsingBoundingBoxIntersection(false);
}

void HypothesisForHumanPose::predict(float time_difference)
{
  HypothesisBase::predict(time_difference);

  for(auto& tracked_joint : m_tracked_joints)
    if(tracked_joint != nullptr)
      tracked_joint->predict(time_difference);

  updateHypothesisAfterPrediction();
}

void HypothesisForHumanPose::updateHypothesisAfterPrediction()
{
  m_points = getTrackedJointsPositions();
  computeBoundingBox(m_points, m_hypothesis_bounding_box);
}

void HypothesisForHumanPose::correct(const Detection& detection)
{
  HypothesisBase::correct(detection);

  for(size_t joint_id = 0; joint_id < m_tracked_joints.size(); joint_id++)
  {
    if(isFinite(detection.points[joint_id]))
    {
      if(m_tracked_joints[joint_id] != nullptr)
        m_tracked_joints[joint_id]->correct(detection.points[joint_id],
                                            detection.points_covariances[joint_id]);
      else
        m_tracked_joints[joint_id] = std::make_shared<KalmanFilter>(detection.points[joint_id]);
    }
  }

  updateHypothesisAfterCorrection(detection);
}

void HypothesisForHumanPose::updateHypothesisAfterCorrection(const Detection& detection)
{
  m_points = getTrackedJointsPositions();
  HypothesisWithBoundingBox::updateBoxesAfterCorrection(detection);
}

std::vector<Eigen::Vector3f> HypothesisForHumanPose::getTrackedJointsPositions()
{
  std::vector<Eigen::Vector3f> joint_positions;
  for(const auto& tracked_joint : m_tracked_joints)
    if(tracked_joint != nullptr)
      joint_positions.emplace_back(tracked_joint->getState().block<3, 1>(0, 0));

  return joint_positions;
}

};

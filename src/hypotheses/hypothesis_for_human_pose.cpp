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
    , m_is_tracked_joint_initialized(detection.points.size(), false)
{
  // Set static property to false right away, because humans are by default potentially dynamic 
  disableVerifyingStaticStatus();

  for(int joint_id = 0; joint_id < detection.points.size(); joint_id++)
  {
    if(std::isnan(detection.points[joint_id].x()))
    {
      // TODO: replace init with fake kalman filter 
      m_tracked_joints.emplace_back(std::make_shared<KalmanFilter>(Eigen::Vector3f(0.f, 0.f, 0.f)));
    }
    else
    {
      m_tracked_joints.emplace_back(std::make_shared<KalmanFilter>(detection.points[joint_id]));
      m_is_tracked_joint_initialized[joint_id] = true;
    }
  }
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

  for(int joint_id = 0; joint_id < m_tracked_joints.size(); joint_id++)
  {
    if(m_is_tracked_joint_initialized[joint_id])
      m_tracked_joints[joint_id]->predict(time_difference);
  }

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

  for(int joint_id = 0; joint_id < m_tracked_joints.size(); joint_id++)
  {
    if(!std::isnan(detection.points[joint_id].x()) && m_is_tracked_joint_initialized[joint_id])
    {
      if(m_is_tracked_joint_initialized[joint_id])
      {
        m_tracked_joints[joint_id]->correct(detection.points[joint_id],
                                            detection.points_covariances[joint_id]);
      }
      else
      {
        m_tracked_joints[joint_id] = std::make_shared<KalmanFilter>(detection.points[joint_id]);
        m_is_tracked_joint_initialized[joint_id] = true;
      }
    }
  }

  updateHypothesisAfterCorrection(detection);
}

void HypothesisForHumanPose::updateHypothesisAfterCorrection(const Detection& detection)
{
  m_points = getTrackedJointsPositions();
  HypothesisWithBoundingBox::updateBoxesAfterCorrection(detection);
}

};

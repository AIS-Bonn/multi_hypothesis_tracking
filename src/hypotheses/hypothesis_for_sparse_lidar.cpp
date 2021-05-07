/** @file
 *
 * Implementation of a hypothesis for objects measured with sparse lidars.
 * Based on a HypothesisWithBoundingBox and extending the class with a method to cap the velocity.
 * This is useful for detections with a lot of noise in size and position.
 * 
 * @author Jan Razlaw
 */

#include "multi_hypothesis_tracking/hypotheses/hypothesis_for_sparse_lidar.h"

namespace MultiHypothesisTracker
{

HypothesisForSparseLidar::HypothesisForSparseLidar(const Detection& detection,
                                                   unsigned int id,
                                                   double time_stamp,
                                                   float min_valid_velocity,
                                                   float max_allowed_velocity)
  : HypothesisBase(detection, id, time_stamp)
    , HypothesisWithPoints(detection, id, time_stamp)
    , HypothesisWithBoundingBox(detection, id, time_stamp)
    , m_min_valid_velocity(min_valid_velocity)
    , m_max_allowed_velocity(max_allowed_velocity)
{

}

void HypothesisForSparseLidar::predict(float time_difference)
{
  HypothesisWithBoundingBox::predict(time_difference);
}

void HypothesisForSparseLidar::correct(const Detection& detection)
{
  HypothesisWithBoundingBox::correct(detection);

  capVelocity();
}

void HypothesisForSparseLidar::capVelocity()
{
  Eigen::Vector3f current_velocity = getVelocity();
  if(current_velocity.norm() < m_min_valid_velocity)
    current_velocity.setZero();

  if(current_velocity.norm() > m_max_allowed_velocity)
  {
    current_velocity.normalize();
    current_velocity *= m_max_allowed_velocity;
  }

  for(int i = 0; i < 3; i++)
    m_kalman_filter->getState()(3 + i) = current_velocity(i);
}

};

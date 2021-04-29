/** @file
 *
 * Interface for every hypothesis with methods that have to be implemented. 
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_INTERFACE_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_INTERFACE_H

#include <multi_hypothesis_tracking/definitions.h>

namespace MultiHypothesisTracker
{

class HypothesisInterface
{
public:
  virtual ~HypothesisInterface() = default;

  /** @brief Predicts the hypothesis' state after time_difference. */
  virtual void predict(float time_difference) = 0;

  /** @brief Corrects the hypothesis' state using the detection. */
  virtual void correct(const Detection& detection) = 0;

  virtual Eigen::Vector3f getPosition() = 0;
  virtual Eigen::Matrix3f getCovariance() = 0;

  virtual bool isSpurious(float m_maximally_allowed_hypothesis_covariance) = 0;
  virtual bool isStatic() = 0;
};

};

#endif 

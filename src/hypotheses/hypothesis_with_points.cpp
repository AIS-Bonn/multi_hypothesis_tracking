/** @file
 *
 * Implementation of a hypothesis extended with point measurements on detected objects.
 *
 * @author Jan Razlaw
 */

#include "multi_hypothesis_tracking/hypotheses/hypothesis_with_points.h"

namespace MultiHypothesisTracker
{

HypothesisWithPoints::HypothesisWithPoints(const Detection& detection,
                                           unsigned int id,
                                           double time_stamp,
                                           bool accumulate_detection_points_in_hypothesis)
  : HypothesisBase(detection, id, time_stamp)
    , m_accumulate_detection_points_in_hypothesis(accumulate_detection_points_in_hypothesis)
{
  m_points = detection.points;
}

void HypothesisWithPoints::predict(float time_difference)
{
  HypothesisBase::predict(time_difference);
  HypothesisWithPoints::updatePointsAfterPrediction();
}

void HypothesisWithPoints::updatePointsAfterPrediction()
{
  const auto& current_position = m_history.position_history.end()[-2];
  const auto& predicted_position = m_history.position_history.end()[-1];

  const auto& translation_from_current_to_predicted_position = (predicted_position - current_position).eval();
  transformPoints(m_points, translation_from_current_to_predicted_position);
}

void HypothesisWithPoints::transformPoints(std::vector<Eigen::Vector3f>& points,
                                           const Eigen::Vector3f& translation)
{
  for(auto& point : points)
    point += translation;
}

void HypothesisWithPoints::correct(const Detection& detection)
{
  const auto predicted_position = m_history.position_history.back();

  HypothesisBase::correct(detection);

  if(m_accumulate_detection_points_in_hypothesis)
  {
    updatePointsAfterCorrection(predicted_position);
    appendDetectionPoints(detection);
  }
  else
    m_points = detection.points;
}

void HypothesisWithPoints::updatePointsAfterCorrection(const Eigen::Vector3f& predicted_position)
{
  const auto& corrected_position = m_history.position_history.back();

  auto translation_from_predicted_to_corrected_position = (corrected_position - predicted_position).eval();
  transformPoints(m_points, translation_from_predicted_to_corrected_position);
}

void HypothesisWithPoints::appendDetectionPoints(const Detection& detection)
{
  const auto transform_detection_to_corrected = (getPosition() - detection.position).eval();
  std::vector<Eigen::Vector3f> corrected_detection_points = detection.points;
  transformPoints(corrected_detection_points, transform_detection_to_corrected);

  m_points.reserve(m_points.size() + corrected_detection_points.size());
  m_points.insert(m_points.end(), corrected_detection_points.begin(), corrected_detection_points.end());
}

};

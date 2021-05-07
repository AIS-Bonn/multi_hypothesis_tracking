/** @file
 *
 * Implementation of a hypothesis with an axis aligned bounding box.
 *
 * @author Jan Razlaw
 */

#include "multi_hypothesis_tracking/hypotheses/hypothesis_with_bounding_box.h"

namespace MultiHypothesisTracker
{

HypothesisWithBoundingBox::HypothesisWithBoundingBox(const Detection& detection,
                                                     unsigned int id,
                                                     double time_stamp,
                                                     bool update_static_status_using_bounding_box_intersection)
  : HypothesisBase(detection, id, time_stamp)
    , HypothesisWithPoints(detection, id, time_stamp)
    , m_update_static_status_using_bounding_box_intersection(update_static_status_using_bounding_box_intersection)
{
  // disable verifyStatic() to prevent interference between methods verifying the static status 
  if(update_static_status_using_bounding_box_intersection)
    HypothesisBase::setDoVerifyStatic(false);

  computeBoundingBox(detection.points, m_detections_bounding_box);
  m_hypothesis_bounding_box = m_detections_bounding_box;
  m_initial_bounding_box = m_detections_bounding_box;
}

void HypothesisWithBoundingBox::computeBoundingBox(const std::vector<Eigen::Vector3f>& points,
                                                   AxisAlignedBox& bounding_box)
{
  bounding_box.min_corner = Eigen::Array3f::Constant(std::numeric_limits<float>::max());
  bounding_box.max_corner = Eigen::Array3f::Constant(-std::numeric_limits<float>::max());
  for(const auto& point : points)
  {
    if(isFinite(point))
    {
      for(Eigen::Index i = 0; i < 3; ++i)
      {
        bounding_box.min_corner[i] = std::min(bounding_box.min_corner[i], point[i]);
        bounding_box.max_corner[i] = std::max(bounding_box.max_corner[i], point[i]);
      }
    }
  }
}

void HypothesisWithBoundingBox::predict(float time_difference)
{
  HypothesisWithPoints::predict(time_difference);
  updateBoxAfterPrediction();
}

void HypothesisWithBoundingBox::updateBoxAfterPrediction()
{
  const auto& current_position = m_history.position_history.end()[-2];
  const auto& predicted_position = m_history.position_history.end()[-1];

  auto translation_from_current_to_predicted_position = (predicted_position - current_position).eval();
  m_hypothesis_bounding_box.moveBox(translation_from_current_to_predicted_position.array());
}

void HypothesisWithBoundingBox::correct(const Detection& detection)
{
  HypothesisWithPoints::correct(detection);

  updateBoxesAfterCorrection(detection);

  verifyStatic();
}

void HypothesisWithBoundingBox::updateBoxesAfterCorrection(const Detection& detection)
{
  computeBoundingBox(m_points, m_hypothesis_bounding_box);
  computeBoundingBox(detection.points, m_detections_bounding_box);
}

void HypothesisWithBoundingBox::verifyStatic()
{
  if(m_update_static_status_using_bounding_box_intersection)
    verifyStaticUsingBoundingBoxes();
  // else the verifyStatic method of the HypothesisBase class was already called automatically after the correction
}

void HypothesisWithBoundingBox::verifyStaticUsingBoundingBoxes()
{
  if(m_is_static)
  {
    AxisAlignedBox intersecting_box(m_hypothesis_bounding_box.min_corner.max(m_initial_bounding_box.min_corner),
                                    m_hypothesis_bounding_box.max_corner.min(m_initial_bounding_box.max_corner));

    Eigen::Array3f side_lengths = intersecting_box.getSideLengths();
    if((side_lengths <= 0.f).any())
      m_is_static = false;
  }
}

};

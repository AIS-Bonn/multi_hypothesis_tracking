/** @file
 *
 * Implementation of a hypothesis with an axis aligned bounding box.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_BOUNDING_BOX_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_BOUNDING_BOX_H

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypotheses/hypothesis_with_points.h>
#include <multi_hypothesis_tracking/utils.h>

namespace MultiHypothesisTracker
{

/**
 * @brief Hypothesis extended with an axis aligned bounding box.
 */
class HypothesisWithBoundingBox : virtual public HypothesisWithPoints
{
public:
  /**
   * @brief Constructor.
   *
   * @param[in] detection    provides initial state.
   * @param[in] id           unique id assigned to this hypothesis.
   * @param[in] time_stamp   time stamp when the detection was created.
   */
  HypothesisWithBoundingBox(const Detection& detection,
                            unsigned int id,
                            double time_stamp,
                            bool update_static_status_using_bounding_box_intersection = true);
  ~HypothesisWithBoundingBox() override = default;

  /** @brief Predicts the next state and updates bounding box using transform from previous to next state. */
  void predict(float time_difference) override;

  /** @brief Corrects the hypothesis' state using the detection and updates the bounding box. */
  void correct(const Detection& detection) override;

  inline AxisAlignedBox& getDetectionsBoundingBox(){ return m_detections_bounding_box; }
  inline AxisAlignedBox& getHypothesisBoundingBox(){ return m_hypothesis_bounding_box; }

  void setUpdateStaticStatusUsingBoundingBoxIntersection(bool update_static_status_using_bounding_box_intersection)
  {
    m_update_static_status_using_bounding_box_intersection = update_static_status_using_bounding_box_intersection;
    // if true disables verifyStatic method from HypothesisBase and vice versa, as they are mutually exclusive
    HypothesisBase::setDoVerifyStatic(!m_update_static_status_using_bounding_box_intersection);
  }

protected:
  /** @brief Computes the axis aligned bounding box of a point cloud. */
  void computeBoundingBox(const std::vector<Eigen::Vector3f>& points,
                          AxisAlignedBox& bounding_box);

  void updateBoxAfterPrediction();
  void updateBoxesAfterCorrection(const Detection& detection);

  /** @brief Sets #m_is_static to true once hypotheses moved too far from initial state. */
  void verifyStatic();

  /** @brief Hypothesis stays static until current and initial bounding boxes do not overlap. 
   * 
   * Tries to compute the intersecting box between the current and the initial bounding boxes.
   * If one side length of the computed intersecting box is zero or negative, the boxes to not intersect.
   * */
  void verifyStaticUsingBoundingBoxes();

  /** @brief Axis aligned bounding box of the detection that was assigned to this hypothesis. */
  AxisAlignedBox m_detections_bounding_box;
  /** @brief Axis aligned bounding box of this hypothesis. */
  AxisAlignedBox m_hypothesis_bounding_box;
  /** @brief Axis aligned bounding box of this hypothesis at creation. */
  AxisAlignedBox m_initial_bounding_box;
  /** @brief If true, @see verifyStaticUsingBoundingBoxes() is used to update #m_is_static. */
  bool m_update_static_status_using_bounding_box_intersection;
};

};

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_BOUNDING_BOX_H

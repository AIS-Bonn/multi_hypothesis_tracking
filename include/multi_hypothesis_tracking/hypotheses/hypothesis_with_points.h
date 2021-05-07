/** @file
 *
 * Implementation of a hypothesis extended with point measurements on detected objects.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_POINTS_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_POINTS_H

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypotheses/hypothesis_base.h>

namespace MultiHypothesisTracker
{

/**
 * @brief Hypothesis extended with a set of 3D points.
 */
class HypothesisWithPoints : virtual public HypothesisBase
{
public:
  /**
   * @brief Constructor.
   *
   * @param[in] detection                                   provides initial state.
   * @param[in] id                                          unique id assigned to this hypothesis.
   * @param[in] time_stamp                                  time stamp when the detection was created.
   * @param[in] accumulate_detection_points_in_hypothesis   if false only the points of the latest detection are stored.
   */
  HypothesisWithPoints(const Detection& detection,
                       unsigned int id,
                       double time_stamp,
                       bool accumulate_detection_points_in_hypothesis = false);
  /** @brief Destructor. */
  ~HypothesisWithPoints() override = default;

  /** @brief Predicts the next state and updates point positions using transform from previous to next state. */
  void predict(float time_difference) override;

  /** @brief Corrects the hypothesis' state using the detection and updates points. */
  void correct(const Detection& detection) override;

  /** @brief Transforms the #m_points using translation. */
  static void transformPoints(std::vector<Eigen::Vector3f>& points,
                              const Eigen::Vector3f& translation);

  inline std::vector<Eigen::Vector3f>& getPointCloud(){ return m_points; }

  void setAccumulateDetectionPointsInHypothesis(bool do_accumulate)
  {
    m_accumulate_detection_points_in_hypothesis = do_accumulate;
  }

protected:
  void updatePointsAfterPrediction();
  void updatePointsAfterCorrection(const Eigen::Vector3f& predicted_position);
  /** @brief Transforms detection's points to the current state's position and appends them to the hypothesis' points.*/
  void appendDetectionPoints(const Detection& detection);

  /** @brief Points representing object. */
  std::vector<Eigen::Vector3f> m_points;

  /** @brief If true, points are accumulated in hypothesis and transformed according to state updates.
   * If false, only latest detection points are stored as they are. */
  bool m_accumulate_detection_points_in_hypothesis;
};

};

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_WITH_POINTS_H

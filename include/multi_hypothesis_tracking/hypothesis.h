/** @file
 *
 * Hypothesis implementation.
 *
 * @author Jan Razlaw
 */

#ifndef MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_H
#define MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_H

#include <memory> // for std::shared_ptr

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypothesis_base.h>

namespace MultiHypothesisTracker
{

/**
 * @brief Hypothesis class used for tracking.
 */
class Hypothesis : public HypothesisBase
{
public:
  /**
   * @brief Constructor.
   *
   * @param[in] detection                                   initial state.
   * @param[in] id                                          unique id assigned to this hypothesis.
   * @param[in] time_stamp                                  time stamp when the detection was created.
   * @param[in] kalman_process_noise_covariance_per_second  parameter for kalman filter.
   */
  Hypothesis(const Detection& detection, 
             unsigned int id, 
             double time_stamp, 
             float kalman_process_noise_covariance_per_second);
  /** @brief Destructor. */
  ~Hypothesis() override = default;

  /**
   * @brief Predicts the state after time difference without control using kalman filter.
   *
   * @param[in] dt  time difference to last state.
   * @see predictNextHypothesesStates(float, Eigen::Vector3f&)
   */
  void predict(float dt) override;

  /**
   * @brief Corrects the hypothesis' state using the detection.
   *
   * @param detection   detection used for correction.
   */
  void correct(const Detection& detection) override;

  /**
   * @brief Transforms the #m_points using transform.
   *
   * @param[in,out] points      points that are transformed.
   * @param[in]     transform   translation that is applies to points.
   */
  void transformPoints(std::vector<Eigen::Vector3f>& points,
                       const Eigen::Vector3f& transform);

  /**
   * @brief Computes the axis aligned bounding box of a point cloud.
   *
   * @param[in]     points          point cloud.
   * @param[out]    bounding_box    axis aligned bounding box around point cloud.
   */
  void computeBoundingBox(const std::vector<Eigen::Vector3f>& points,
                          AxisAlignedBox& bounding_box);

  inline AxisAlignedBox& getDetectionsBoundingBox(){ return m_detections_bounding_box; }
  inline AxisAlignedBox& getHypothesisBoundingBox(){ return m_hypothesis_bounding_box; }

  /** @brief Getter for point cloud. */
  inline std::vector<Eigen::Vector3f>& getPointCloud(){ return m_points; }
  
protected:
  void updateHypothesisAfterPrediction();
  void updateHypothesisAfterCorrection();
  void capVelocity();
  void updatePoints(const Detection& detection);
  void updateBoundingBox(const Detection& detection);

  /** @brief Check if hypothesis is still static. */
  void verifyStatic(Eigen::Array3f& min_corner_detection,
                    Eigen::Array3f& max_corner_detection);

  /** @brief Flag that specifies if velocity should be capped. */
  bool m_cap_velocity;
  /** @brief Bound for velocity. */
  double m_max_allowed_velocity;

  /** @brief Axis aligned bounding box of the detection that was assigned to this hypothesis. */
  AxisAlignedBox m_detections_bounding_box;
  /** @brief Axis aligned bounding box of this hypothesis. */
  AxisAlignedBox m_hypothesis_bounding_box;
  /** @brief Axis aligned bounding box of this hypothesis at creation. */
  AxisAlignedBox m_initial_bounding_box;

  /** @brief Points representing object. */
  std::vector<Eigen::Vector3f> m_points;
};

};

#endif //MULTI_HYPOTHESIS_TRACKING_HYPOTHESIS_H

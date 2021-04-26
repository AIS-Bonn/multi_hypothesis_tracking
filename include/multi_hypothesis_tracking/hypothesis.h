/** @file
 *
 * Hypothesis implementation.
 *
 * @author Jan Razlaw
 */

#ifndef __HYPOTHESIS_H__
#define __HYPOTHESIS_H__

#include <memory> // for std::shared_ptr
#include <iostream>
#include <iomanip> // for std::setprecision
#include <queue>

#include <multi_hypothesis_tracking/kalman_filter.h>
#include <multi_hypothesis_tracking/definitions.h>

namespace MultiHypothesisTracker
{

/**
 * @brief Hypothesis class used for tracking.
 */
class Hypothesis
{
public:
  /**
   * @brief Constructor.
   *
   * @param[in] detection               initial state.
   * @param[in] id                      id of hypothesis.
   * @param[in] time_stamp              time stamp when the detection was created.
   * @param[in] covariance_per_second   parameter for kalman filter.
   */
  Hypothesis(const Detection& detection,
             unsigned int id,
             double time_stamp,
             float covariance_per_second = 0.5f);
  /** @brief Destructor. */
  virtual ~Hypothesis() = default;

  /**
   * @brief Predicts the state after time difference without control using kalman filter.
   *
   * @param[in] dt  time difference to last state.
   * @see predictNextHypothesesStates(float, Eigen::Vector3f&)
   */
  void predict(float dt);

  /**
   * @brief Corrects the hypothesis' state using the detection.
   *
   * @param detection   detection used for correction.
   */
  void correct(const Detection& detection);

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

  /**
   * @brief Checks if covariance exceeds max_covariance.
   *
   * @param covariance      covariance matrix that is checked.
   * @param max_covariance  covariance threshold.
   *
   * @return true if exceeds, false otherwise
   */
  bool exceedsMaxCovariance(const Eigen::Matrix3f& covariance,
                            float max_covariance);

  /** @brief Currently calls exceedsMaxCovariance to check if hypothesis is spurious. */
  bool isSpurious(float max_covariance);

  /** @brief Getter for hypothesis ID. */
  inline unsigned int getID(){ return m_id; }

  /** @brief Getter for current position. */
  inline Eigen::Vector3f getPosition(){ return m_kalman_filter->getState().block<3, 1>(0, 0); }
  /** @brief Getter for current velocity. */
  inline Eigen::Vector3f getVelocity(){ return m_kalman_filter->getState().block<3, 1>(3, 0); }
  /** @brief Getter for current position covariance matrix. */
  inline Eigen::Matrix3f getCovariance(){ return m_kalman_filter->getErrorCovariance().block<3, 3>(0, 0); }

  /** @brief Getter for static property. */
  inline bool isStatic(){ return m_is_static; }

  /** @brief Getter time of hypothesis initialization. */
  inline double getTimeStampOfBirth(){ return m_time_stamp_of_birth; }

  inline AxisAlignedBox& getDetectionsBoundingBox(){ return m_detections_bounding_box; }
  inline AxisAlignedBox& getHypothesisBoundingBox(){ return m_hypothesis_bounding_box; }

  inline Eigen::Vector3f& getInitialPosition(){ return m_position_history[0]; }
  /** @brief Getter for #m_position_history. */
  inline std::vector<Eigen::Vector3f>& getPositionHistory(){ return m_position_history; }
  /** @brief Getter for #m_was_assigned_history. */
  inline std::vector<bool>& getWasAssignedHistory(){ return m_was_assigned_history; }
  inline int getNumberOfAssignments(){ return m_number_of_assignments; }

  /** @brief Getter for point cloud. */
  inline std::vector<Eigen::Vector3f>& getPointCloud(){ return m_points; }
protected:

  /** @brief Check if hypothesis is still static. */
  void verifyStatic(Eigen::Array3f& min_corner_detection,
                    Eigen::Array3f& max_corner_detection);

  /** @brief Check if hypothesis is still static. */
  void verifyStatic();

  /** @brief Kalman filter for state estimation. */
  std::shared_ptr<KalmanFilter> m_kalman_filter;

  /** @brief Hypothesis ID. */
  unsigned int m_id;

  double m_time_stamp_of_birth;

  /** @brief Flag for immobility of hypothesis. */
  bool m_is_static;
  /** @brief Distance a hypothesis is allowed to move to still be considered static. */
  double m_static_distance_threshold;

  /** @brief Flag that specifies if velocity should be capped. */
  bool m_cap_velocity;
  /** @brief Bound for velocity. */
  double m_max_allowed_velocity;
  /** @brief Maximal velocity this hypothesis had. */
  double m_max_tracked_velocity;

  /** @brief Axis aligned bounding box of the detection that was assigned to this hypothesis. */
  AxisAlignedBox m_detections_bounding_box;
  /** @brief Axis aligned bounding box of this hypothesis. */
  AxisAlignedBox m_hypothesis_bounding_box;
  /** @brief Axis aligned bounding box of this hypothesis at creation. */
  AxisAlignedBox m_initial_bounding_box;

  /** @brief Points representing object. */
  std::vector<Eigen::Vector3f> m_points;

  /** @brief All positions of the hypothesis. */
  std::vector<Eigen::Vector3f> m_position_history;
  /** @brief Vector of bools indicating whether this hypothesis was assigned to a detection in the corresponding step.*/
  std::vector<bool> m_was_assigned_history;

  /** @brief The number of times this hypothesis was assigned to a detection. */
  int m_number_of_assignments;
};

/**
 * @brief Hypothesis factory.
 */
class HypothesisFactory
{
public:
  /** @brief Constructor. */
  HypothesisFactory() = default;
  /** @brief Destructor. */
  virtual ~HypothesisFactory() = default;

  /** @brief Setter for distance threshold #m_max_bhattacharyya_distance. */
  inline void
  setKalmanCovariancePerSecond(float covariance_per_second){ m_covariance_per_second = covariance_per_second; }

  /** @brief Covariance per second for kalman filter.*/
  float m_covariance_per_second = 0.5f;

  /** @brief Creates hypothesis.
   *
   * @param[in] detection       initial state.
   * @param[in] id              ID of created hypothesis.
   * @param[in] time_stamp      time stamp when the detection was created.
   *
   * @return pointer to created hypothesis.
   */
  std::shared_ptr<Hypothesis> createHypothesis(const Detection& detection,
                                               const unsigned int id,
                                               const double time_stamp);
};

};

#endif //__HYPOTHESIS_H__

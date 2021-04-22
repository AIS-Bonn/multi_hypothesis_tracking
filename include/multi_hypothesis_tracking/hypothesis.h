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
             double covariance_per_second = 0.5);
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
   * @brief Predicts the state after time difference using kalman filter.
   *
   * @param[in] dt          time difference to last state.
   * @param[in] control     control vector used for prediction.
   * @see predictNextHypothesesStates(float, Eigen::Vector3f&)
   */
  void predict(float dt,
               Eigen::Vector3f& control);

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
   * @brief Computes the bounding box of a point cloud.
   *
   * @param[in]     points              points of cloud.
   * @param[out]    min_bounding_box    min corner of bounding box.
   * @param[out]    max_bounding_box    max corner of bounding box.
   */
  void computeBoundingBox(const std::vector<Eigen::Vector3f>& points,
                          Eigen::Array3f& min_bounding_box,
                          Eigen::Array3f& max_bounding_box);

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

  /**
   * @brief Checks if hypothesis is spurious.
   *
   * Checks if hypothesis' covariance is below max covariance.
   *
   * @param[in] max_covariance              maximally allowed covariance.
   *
   * @return true if hypothesis is spurious, false otherwise.
   */
  bool isSpurious(float max_covariance = 5.f);

  /**
   * @brief Computes the likelihood of the detection given the state.
   *
   * @param[in] detection   detection.
   * @return likelihood.
   */
  float computeLikelihood(const Detection& detection);

  /** @brief Getter for hypothesis ID. */
  inline unsigned int getID(){ return m_id; }

  /** @brief Getter for current position. */
  inline Eigen::Vector3f getPosition(){ return m_kalman->getState().block<3, 1>(0, 0); }
  /** @brief Getter for current velocity. */
  inline Eigen::Vector3f getVelocity(){ return m_kalman->getState().block<3, 1>(3, 0); }
  /** @brief Getter for current position covariance matrix. */
  inline Eigen::Matrix3f getCovariance(){ return m_kalman->getErrorCovariance().block<3, 3>(0, 0); }

  /** @brief Getter for static property. */
  inline bool isStatic(){ return m_is_static; }

  /** @brief Getter time of hypothesis initialization. */
  inline double getBornTime(){ return m_born_time; }

  /** @brief Getter for #m_min_corner_detection. */
  inline Eigen::Array3f& getMinBoxDetection(){ return m_min_corner_detection; }
  /** @brief Getter for #m_max_corner_detection. */
  inline Eigen::Array3f& getMaxBoxDetection(){ return m_max_corner_detection; }
  /** @brief Getter for #m_min_corner_hypothesis. */
  inline Eigen::Array3f& getMinBoxHypothesis(){ return m_min_corner_hypothesis; }
  /** @brief Getter for #m_max_corner_hypothesis. */
  inline Eigen::Array3f& getMaxBoxHypothesis(){ return m_max_corner_hypothesis; }
  /** @brief Getter for size of hypothesis' bounding box. */
  inline Eigen::Array3f getHypothesisBoxSize(){ return (m_max_corner_hypothesis - m_min_corner_hypothesis).eval(); }
  /** @brief Getter for #m_position_history. */
  inline std::vector<Eigen::Vector3f>& getPositionHistory(){ return m_position_history; }
  /** @brief Getter for #m_was_assigned_history. */
  inline std::vector<bool>& getWasAssignedHistory(){ return m_was_assigned_history; }
  /** @brief Getter for #m_was_assigned_counter. */
  inline int getNumberOfAssignments(){ return m_was_assigned_counter; }

  /** @brief Getter for point cloud. */
  inline std::vector<Eigen::Vector3f>& getPointCloud(){ return m_points; }
protected:

  /** @brief Check if hypothesis is still static. */
  void verifyStatic(Eigen::Array3f& min_corner_detection,
                    Eigen::Array3f& max_corner_detection);

  /** @brief Check if hypothesis is still static. */
  void verifyStatic();

  /** @brief Kalman filter for state estimation. */
  std::shared_ptr<KalmanFilter> m_kalman;

  /** @brief Hypothesis ID. */
  unsigned int m_id;

  /** @brief Initial position. */
  Eigen::Vector3f m_first_position_in_track;

  /** @brief Time of initialization. */
  double m_born_time;

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

  /** @brief Minimum corner of the detections' bounding box that was assigned to this hypothesis. */
  Eigen::Array3f m_min_corner_detection;
  /** @brief Maximum corner of the detections' bounding box that was assigned to this hypothesis. */
  Eigen::Array3f m_max_corner_detection;
  /** @brief Minimum corner of the hypothesis' bounding box. */
  Eigen::Array3f m_min_corner_hypothesis;
  /** @brief Maximum corner of the hypothesis' bounding box. */
  Eigen::Array3f m_max_corner_hypothesis;
  /** @brief Minimum corner of the hypothesis' bounding box at its initialization. */
  Eigen::Array3f m_min_corner_init_hypothesis;
  /** @brief Maximum corner of the hypothesis' bounding box at its initialization. */
  Eigen::Array3f m_max_corner_init_hypothesis;

  /** @brief Points representing object. */
  std::vector<Eigen::Vector3f> m_points;

  /** @brief All positions of the hypothesis. */
  std::vector<Eigen::Vector3f> m_position_history;
  /** @brief Vector of bools indicating whether this hypothesis was assigned to a detection in the corresponding step.*/
  std::vector<bool> m_was_assigned_history;

  /** @brief The number of times this hypotheses was assigned to a detection. */
  int m_was_assigned_counter;
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
  setKalmanCovariancePerSecond(double covariance_per_second){ m_covariance_per_second = covariance_per_second; }

  /** @brief Covariance per second for kalman filter.*/
  double m_covariance_per_second = 0.5;

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

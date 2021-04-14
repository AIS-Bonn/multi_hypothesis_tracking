/** @file
 *
 * Multi hypothesis tracker implementation.
 *
 * @author Jan Razlaw
 */

#ifndef __MULTI_HYPOTHESIS_TRACKER_H__
#define __MULTI_HYPOTHESIS_TRACKER_H__

#include <vector>
#include <queue>
#include <iostream>
#include <limits.h> // for INT_MAX

#include <multi_object_tracking/hypothesis.h>
#include <multi_object_tracking/hungarian.h>

#include <multi_object_tracking/utils.h>

namespace MultiHypothesisTracker
{

class HypothesisFactory;

/**
 * @brief Multi hypothesis tracker class.
 */
class MultiHypothesisTracker
{
public:
  /** @brief Constructor. */
  explicit MultiHypothesisTracker(std::shared_ptr<HypothesisFactory> hypothesis_factory);
  /** @brief Destructor. */
  ~MultiHypothesisTracker() = default;

  /**
   * @brief Calls predict for each hypothesis.
   *
   * @param[in] time_diff   time difference between last and current prediction
   */
  virtual void predict(double time_diff);

  /**
   * @brief Calls predict for each hypothesis.
   *
   * @param[in] time_diff   time difference between last and current prediction
   * @param[in] control     control input for state prediction
   */
  virtual void predict(double time_diff,
                       Eigen::Vector3f& control);

  /**
   * @brief Uses Hungarian method to assign measurements to hypotheses and corrects the latter.
   *
   * @param[in] measurements    measurements
   */
  void correct(const std::vector<Measurement>& measurements);

  /**
   * @brief Deletes hypotheses that are spurious
   *
   * @param[in] max_covariance              maximally allowed covariance
   *
   * @see isSpurious()
   */
  void deleteSpuriousHypotheses(float max_covariance);

  /**
   * @brief Deletes all hypotheses that are too close to others.
   *
   * //TODO: implement a reasonable merging function.
   *
   * @param[in] distance_threshold  two hypotheses with a distance below that threshold are merged
   */
  void mergeCloseHypotheses(double distance_threshold);

  /** @brief Deletes hypotheses that were marked for deletion. */
  void deleteMarkedHypotheses();

  /** @brief Getter for #m_hypotheses. */
  inline std::vector<std::shared_ptr<Hypothesis>>& getHypotheses(){ return m_hypotheses; }

  /** @brief Getter for #m_heavens_gate. */
  inline std::queue<Hypothesis>& getDeletedHypotheses(){ return m_heavens_gate; }

  /** @brief Getter for #m_heavens_gate. */
  void clearDeletedHypotheses(){ std::queue<Hypothesis> empty; std::swap(m_heavens_gate, empty); }

  /** @brief Setter for distance threshold #m_max_bhattacharyya_distance. */
  inline void setMaxCorrespondenceDistance(double distance){ m_max_bhattacharyya_distance = (int)m_dist_scale * distance; }

  /** @brief Setter for distance threshold #m_max_bhattacharyya_distance. */
  inline void setKalmanCovariancePerSecond(double covariance_per_second){ m_hypothesis_factory->setKalmanCovariancePerSecond(covariance_per_second); }

  /** @brief Setter for #m_compute_likelihood. */
  inline void setComputeLikelihood(bool compute_likelihood){ m_compute_likelihood = compute_likelihood; }

  /** @brief Getter for the average likelihood. */
  float getAverageLikelihood(){ return m_likelihood_sum / m_assigned_hypotheses_counter; };

protected:
  /**
   * @brief Set up cost matrix for hungarian method.
   *
   * Top left block: Pairwise thresholded Mahalanobis distance between each
   * measurement and each hypothesis.
   * Top right block: Fake distance of hypotheses to dummy measurements
   *    -> #m_max_mahalanobis_distance
   * Bottom left block: Fake distance of measurements to dummy hypotheses
   *    -> #m_max_mahalanobis_distance
   * Bottom right block: Fake distance between dummy measurements and dummy
   * hyptheses -> Zeroes.
   *
   * @param[in]     measurements    detections.
   * @param[in]     hypotheses      already existing hypotheses.
   * @param[out]    cost_matrix     cost matrix.
   */
  void setupCostMatrix(const std::vector<Measurement>& measurements,
                       std::vector<std::shared_ptr<Hypothesis>>& hypotheses,
                       int**& cost_matrix);

  /**
   * @brief Use assignments for correction and initialization.
   *
   * If measurement assigned to hypothesis -> correct latter.
   * If measurement not assigned -> new hypothesis.
   * If hypothesis not assigned -> failed to detect.
   *
   * @param[in]     assignments     assignments from hungarian method.
   * @param[in]     cost_matrix     original cost_matrix hungarian method was initialized with.
   * @param[in]     measurements    detections.
   * @param[in,out] hypotheses      in current hypotheses, out corrected and new hypotheses.
   */
  void applyAssignments(int**& assignments,
                        int**& cost_matrix,
                        const std::vector<Measurement>& measurements,
                        std::vector<std::shared_ptr<Hypothesis>>& hypotheses);

  /** @brief Sets members used to compute the average likelihood to zero. */
  void resetAverageLikelihood()
  {
    m_likelihood_sum = 0.f;
    m_assigned_hypotheses_counter = 0;
  };

  /**
   * @brief Updates the #m_likelihood_sum and #m_assigned_hypotheses_counter.
   * @param[in] likelihood  the likelihood of the current measurement given its hypothesis' state.
   */
  void updateLikelihoodSum(float likelihood)
  {
    m_likelihood_sum += likelihood;
    m_assigned_hypotheses_counter++;
  };

  /**
   * @brief Copies hypothesis to heavens gate if conditions met and erases it from the vector.
   *
   * @param[in] hypotheses  vector of hypotheses.
   * @param[in] it          iterator to hypothesis that should be erased.
   *
   * @returns iterator to entry after deleted entry.
   */
  std::vector<std::shared_ptr<Hypothesis>>::iterator erase(std::vector<std::shared_ptr<Hypothesis>>& hypotheses,
                                                           std::vector<std::shared_ptr<Hypothesis>>::iterator& it);

  /** @brief Hypothesis factory.*/
  std::shared_ptr<HypothesisFactory> m_hypothesis_factory;
  /** @brief Vector storing all tracked hypotheses.*/
  std::vector<std::shared_ptr<Hypothesis>> m_hypotheses;

  /** @brief Counter for hypotheses IDs.*/
  unsigned int m_current_hypothesis_id;

  /** @brief Scale from double to int, because distance is in double but hungarian needs int costs.*/
  int m_dist_scale;
  /** @brief Scaled distance threshold for assignments.*/
  int m_max_bhattacharyya_distance;
  /** @brief If true, computes m_average_likelihood.*/
  bool m_compute_likelihood;
  /** @brief Sum of likelihoods of measurements given the states.*/
  float m_likelihood_sum;
  /** @brief Number of hypotheses that were assigned to measurements for the current time.*/
  int m_assigned_hypotheses_counter;

  /** @brief Queue for hypotheses that were marked for deletion.*/
  std::queue<Hypothesis> m_heavens_gate;
};

};

#endif //__MULTI_HYPOTHESIS_TRACKER_H__
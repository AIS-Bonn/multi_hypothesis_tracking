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

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypothesis.h>
#include <multi_hypothesis_tracking/hungarian.h>

#include <multi_hypothesis_tracking/utils.h>

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

  /** @brief Calls predictNextHypothesesStates for each hypothesis. */
  virtual void predictNextHypothesesStates(double duration_since_previous_prediction);

  /** @brief Uses Hungarian method to assign detections to hypotheses and corrects the latter. */
  void correctHypothesesStates(const Detections& detections);

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

  /** @brief Setter for #m_use_bhattacharyya_for_assignments. */
  inline void
  setUseBhattacharyyaDistance(bool use_bhattacharyya){ m_use_bhattacharyya_for_assignments = use_bhattacharyya; }

  /** @brief Setter for distance threshold #m_max_distance. */
  inline void setMaxCorrespondenceDistance(double distance){ m_max_distance = (int)m_dist_scale * distance; }

  /** @brief Setter for covariance increase per second. */
  inline void setKalmanCovariancePerSecond(double covariance_per_second)
  {
    m_hypothesis_factory->setKalmanCovariancePerSecond(covariance_per_second);
  }

protected:
  /**
   * @brief Set up cost matrix for hungarian method.
   *
   * Top left block: Pairwise thresholded distance between each
   * detection and each hypothesis.
   * Top right block: Fake distance of hypotheses to dummy detections
   *    -> #m_max_distance
   * Bottom left block: Fake distance of detections to dummy hypotheses
   *    -> #m_max_distance
   * Bottom right block: Fake distance between dummy detections and dummy
   * hypotheses -> Zeroes.
   *
   * @param[in]     detections      detections.
   * @param[in]     hypotheses      already existing hypotheses.
   * @param[out]    cost_matrix     cost matrix.
   */
  void setupCostMatrix(const Detections& detections,
                       std::vector<std::shared_ptr<Hypothesis>>& hypotheses,
                       int**& cost_matrix);

  /**
   * @brief Use assignments for correction and initialization.
   *
   * If detection assigned to hypothesis -> correct latter.
   * If detection not assigned -> new hypothesis.
   * If hypothesis not assigned -> failed to detect.
   *
   * @param[in]     assignments     assignments from hungarian method.
   * @param[in]     cost_matrix     original cost_matrix hungarian method was initialized with.
   * @param[in]     detections      detections.
   * @param[in,out] hypotheses      in current hypotheses, out corrected and new hypotheses.
   */
  void applyAssignments(int**& assignments,
                        int**& cost_matrix,
                        const Detections& detections,
                        std::vector<std::shared_ptr<Hypothesis>>& hypotheses);

  /** @brief Hypothesis factory.*/
  std::shared_ptr<HypothesisFactory> m_hypothesis_factory;
  /** @brief Vector storing all tracked hypotheses.*/
  std::vector<std::shared_ptr<Hypothesis>> m_hypotheses;

  /** @brief Counter for hypotheses IDs.*/
  unsigned int m_current_hypothesis_id;

  /** @brief If true, bhattacharyya distance is used for correspondences, else euclidean is used.*/
  bool m_use_bhattacharyya_for_assignments;
  /** @brief Scale from double to int, because distance is in double but hungarian needs int costs.*/
  int m_dist_scale;
  /** @brief Scaled distance threshold for assignments.*/
  int m_max_distance;
};

};

#endif //__MULTI_HYPOTHESIS_TRACKER_H__
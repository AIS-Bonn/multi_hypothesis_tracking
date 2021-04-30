/** @file
 *
 * Multi hypothesis tracker implementation.
 *
 * @author Jan Razlaw
 */

#ifndef __MULTI_HYPOTHESIS_TRACKER_H__
#define __MULTI_HYPOTHESIS_TRACKER_H__

#include <utility>
#include <vector>
#include <queue>
#include <iostream>
#include <limits.h> // for INT_MAX

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/hypotheses/hypothesis_factory.h>
#include <multi_hypothesis_tracking/hungarian.h>

#include <multi_hypothesis_tracking/utils.h>

namespace MultiHypothesisTracker
{

/**
 * @brief Multi hypothesis tracker class.
 */
class MultiHypothesisTracker
{
public:
  /** @brief Constructor. */
  MultiHypothesisTracker();
  /** @brief Destructor. */
  ~MultiHypothesisTracker() = default;

  /** @brief Calls predictNextHypothesesStates for each hypothesis. */
  virtual void predictNextHypothesesStates(double duration_since_previous_prediction);

  /** @brief Uses Hungarian method to assign detections to hypotheses and corrects the latter. */
  void correctHypothesesStates(const Detections& detections);

  /** @brief Calls methods to check which hypotheses are weak and deletes those. */
  void filterWeakHypotheses();

  /**
   * @brief Deletes hypotheses that are spurious
   *
   * @see isWeak()
   */
  void deleteWeakHypotheses();

  /** @brief Deletes the younger hypothesis if the distance between two hypotheses is below 
   * #m_distance_threshold_for_hypotheses_merge. */
  void mergeCloseHypotheses();

  /** @brief Getter for #m_hypotheses. */
  inline std::vector<std::shared_ptr<HypothesisInterface>>& getHypotheses(){ return m_hypotheses; }

  /** @brief Setter for #m_use_bhattacharyya_for_assignments. */
  inline void setUseBhattacharyyaDistance(bool use_bhattacharyya){ m_use_bhattacharyya_for_assignments = use_bhattacharyya; }

  /** @brief Setter for distance threshold #m_max_distance. */
  inline void setMaxCorrespondenceDistance(double distance){ m_max_distance = (int)m_dist_scale * distance; }

  /** @brief Setter for #m_distance_threshold_for_hypotheses_merge. */
  inline void setDistanceThresholdForHypothesesMerge(double distance){ m_distance_threshold_for_hypotheses_merge = distance; }

  inline void setHypothesisFactory(std::shared_ptr<HypothesisFactoryInterface> hypothesis_factory)
  { 
    m_hypothesis_factory = std::move(hypothesis_factory); 
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
   * @param[out]    cost_matrix     cost matrix.
   */
  void setupCostMatrix(const Detections& detections,
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
   */
  void applyAssignments(int**& assignments,
                        int**& cost_matrix,
                        const Detections& detections);

  // Variables
  /** @brief Hypothesis factory.*/
  std::shared_ptr<HypothesisFactoryInterface> m_hypothesis_factory;
  /** @brief Vector storing all tracked hypotheses.*/
  std::vector<std::shared_ptr<HypothesisInterface>> m_hypotheses;

  /** @brief Scale from double to int, because distance is in double but hungarian needs int costs.*/
  int m_dist_scale;
  
  // Parameters
  /** @brief If true, bhattacharyya distance is used for correspondences, else euclidean is used.*/
  bool m_use_bhattacharyya_for_assignments;
  /** @brief Scaled distance threshold for assignments.*/
  int m_max_distance;
  /** @brief Hypotheses are merged if their distance is below this parameter. */
  double m_distance_threshold_for_hypotheses_merge;
};

};

#endif //__MULTI_HYPOTHESIS_TRACKER_H__
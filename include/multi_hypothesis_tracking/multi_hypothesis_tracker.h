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

class MultiHypothesisTracker
{
public:
  MultiHypothesisTracker();
  ~MultiHypothesisTracker() = default;

  /** @brief Calls predictNextHypothesesStates for each hypothesis. */
  virtual void predictNextHypothesesStates(double duration_since_previous_prediction);

  /** @brief Uses Hungarian method to assign detections to hypotheses and corrects the latter. */
  void correctHypothesesStates(const Detections& detections);

  /** @brief Calls methods to check which hypotheses are weak and deletes those. */
  void filterWeakHypotheses();

  inline std::vector<std::shared_ptr<HypothesisInterface>>& getHypotheses()
  { 
    return m_hypotheses; 
  }
  
  inline void setUseBhattacharyyaDistanceInsteadOfEuclideanForAssignments(bool use_bhattacharyya)
  { 
    m_use_bhattacharyya_for_assignments = use_bhattacharyya; 
  }
  
  inline void setMaxCorrespondenceDistanceForAssignments(float distance)
  {
    m_max_correspondence_distance_for_assignments = distance;
    m_scaled_max_correspondence_distance_for_assignments = static_cast<int>((float)m_correspondence_distance_scale * distance); 
  }
  
  inline void setDistanceThresholdForHypothesesMerge(float distance)
  { 
    m_distance_threshold_for_hypotheses_merge = distance; 
  }
  
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
   * @brief Use assignments for hypothesis correction and initialization.
   *
   * If detection assigned to hypothesis -> correct latter.
   * If detection not assigned -> create new hypothesis.
   * If hypothesis not assigned -> failed to detect the object corresponding to the hypothesis.
   *
   * @param[in]     assignments     assignments from hungarian method.
   * @param[in]     cost_matrix     original cost_matrix hungarian method was initialized with.
   * @param[in]     detections      detections.
   */
  void applyAssignments(int**& assignments,
                        int**& cost_matrix,
                        const Detections& detections);

  /** @brief Deletes hypotheses that return true for their corresponding isWeak method.
   * @see isWeak()
   */
  void deleteWeakHypotheses();

  /** @brief Deletes the younger hypothesis if the distance between two hypotheses is below 
   * #m_distance_threshold_for_hypotheses_merge. */
  void mergeCloseHypotheses();
  
  /** @brief Creates hypotheses of the requested type. */
  std::shared_ptr<HypothesisFactoryInterface> m_hypothesis_factory;
  /** @brief Vector storing all tracked hypotheses. */
  std::vector<std::shared_ptr<HypothesisInterface>> m_hypotheses;
  /** @brief If true, bhattacharyya distance is used for correspondences, else euclidean is used. */
  bool m_use_bhattacharyya_for_assignments;
  /** @brief The hungarian methods expects distances as int values. The computed float distances are scaled with this 
   * factor and casted to integers. */
  int m_correspondence_distance_scale;
  /** @brief A detection can only be assigned to a detection if their distance is below. */
  float m_max_correspondence_distance_for_assignments;
  /** @brief Scaled distance threshold for assignments. */
  int m_scaled_max_correspondence_distance_for_assignments;
  /** @brief Hypotheses are merged if their distance is below this parameter. */
  float m_distance_threshold_for_hypotheses_merge;
};

};

#endif //__MULTI_HYPOTHESIS_TRACKER_H__
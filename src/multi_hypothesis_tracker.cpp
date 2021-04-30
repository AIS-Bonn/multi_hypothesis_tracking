/** @file
 *
 * Multi hypothesis tracker implementation.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/multi_hypothesis_tracker.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTracker::MultiHypothesisTracker()
  : m_use_bhattacharyya_for_assignments(true)
    , m_correspondence_distance_scale(10000)
    , m_max_correspondence_distance_for_assignments(20.f)
    , m_scaled_max_correspondence_distance_for_assignments(static_cast<int>((float)m_correspondence_distance_scale * m_max_correspondence_distance_for_assignments))
    , m_distance_threshold_for_hypotheses_merge(0.5f)
{
}

void MultiHypothesisTracker::predictNextHypothesesStates(double duration_since_previous_prediction)
{
  for(auto& hypothesis : m_hypotheses)
    hypothesis->predict(static_cast<float>(duration_since_previous_prediction));
}

void MultiHypothesisTracker::correctHypothesesStates(const Detections& detections)
{
  if(detections.detections.empty())
    return;

  int** cost_matrix;
  setupCostMatrix(detections, cost_matrix);

  hungarian_problem_t hung;
  size_t dim = detections.detections.size() + m_hypotheses.size();
  hungarian_init(&hung, cost_matrix, dim, dim, HUNGARIAN_MODE_MINIMIZE_COST);
  hungarian_solve(&hung);
  applyAssignments(hung.assignment, cost_matrix, detections);

  for(size_t i = 0; i < dim; i++)
    delete[] cost_matrix[i];
  delete[] cost_matrix;
  hungarian_free(&hung);
}

bool isRealHypothesis(const size_t index,
                      const size_t number_of_real_hypotheses)
{
  return index < number_of_real_hypotheses;
}

bool isDummyHypothesis(const size_t index,
                       const size_t number_of_real_hypotheses)
{
  return !isRealHypothesis(index, number_of_real_hypotheses);
}

bool isRealDetection(const size_t index,
                     const size_t number_of_real_detections)
{
  return index < number_of_real_detections;
}

bool isDummyDetection(const size_t index,
                      const size_t number_of_real_detections)
{
  return !isRealDetection(index, number_of_real_detections);
}

void MultiHypothesisTracker::setupCostMatrix(const Detections& detections,
                                             int**& cost_matrix)
{
  size_t hyp_size = m_hypotheses.size();
  size_t meas_size = detections.detections.size();
  size_t dim = hyp_size + meas_size;

  cost_matrix = new int* [dim];
  for(size_t hyp_id = 0; hyp_id < dim; hyp_id++)
  {
    cost_matrix[hyp_id] = new int[dim];
    for(size_t det_id = 0; det_id < dim; det_id++)
    {
      if(isRealHypothesis(hyp_id, hyp_size) && isRealDetection(det_id, meas_size))
      {
        float distance = calculateDistance(m_hypotheses[hyp_id], detections.detections[det_id]);
        if(distance < m_max_correspondence_distance_for_assignments)
          cost_matrix[hyp_id][det_id] = static_cast<int>((float)m_correspondence_distance_scale * distance);
        else
          cost_matrix[hyp_id][det_id] = DO_NOT_ASSIGN;
      }
      else if((isRealHypothesis(hyp_id, hyp_size) && isDummyDetection(det_id, meas_size)) ||
        (isDummyHypothesis(hyp_id, hyp_size) && isRealDetection(det_id, meas_size)))
      {
        cost_matrix[hyp_id][det_id] = m_scaled_max_correspondence_distance_for_assignments;
      }
      else if(isDummyHypothesis(hyp_id, hyp_size) && isDummyDetection(det_id, meas_size))
      {
        cost_matrix[hyp_id][det_id] = 0;
      }
    }
  }
}

float MultiHypothesisTracker::calculateDistance(const std::shared_ptr<HypothesisInterface>& hypothesis,
                                                const Detection& detection)
{
  if(m_use_bhattacharyya_for_assignments)
  {
    return bhattacharyya(hypothesis->getPosition(),
                         detection.position.block<3, 1>(0, 0),
                         hypothesis->getCovariance(),
                         detection.covariance.block<3, 3>(0, 0));
  }
  else
  {
    return (hypothesis->getPosition() - detection.position).norm();
  }
}

void MultiHypothesisTracker::applyAssignments(int**& assignments,
                                              int**& cost_matrix,
                                              const Detections& detections)
{
  size_t hyp_size = m_hypotheses.size();
  size_t meas_size = detections.detections.size();
  size_t dim = hyp_size + meas_size;

  for(size_t i = 0; i < dim; i++)
  {
    for(size_t j = 0; j < dim; j++)
    {
      if(i < hyp_size && j < meas_size)
      {
        // if hypothesis assigned to detection and distance below threshold -> correct hypothesis
        if(assignments[i][j] == HUNGARIAN_ASSIGNED && 
        cost_matrix[i][j] < m_scaled_max_correspondence_distance_for_assignments)
        {
          m_hypotheses[i]->correct(detections.detections[j]);
        }
        else if(assignments[i][j] == HUNGARIAN_ASSIGNED)
        {
          // if assigned but distance too high -> prohibited assignment -> hypothesis unassignment

          // create new hypothesis from detection
          m_hypotheses.emplace_back(m_hypothesis_factory->createHypothesis(detections.detections[j],
                                                                           detections.time_stamp));
        }
      }
      else if(i < hyp_size && j >= meas_size)
      {
        // if hypothesis assigned to dummy detection -> failed to detect hypothesis
      }
      else if(i >= hyp_size && j < meas_size)
      {
        // if detection assigned to dummy hypothesis -> create new hypothesis
        if(assignments[i][j] == HUNGARIAN_ASSIGNED)
          m_hypotheses.emplace_back(m_hypothesis_factory->createHypothesis(detections.detections[j],
                                                                           detections.time_stamp));
      }
      else if(i >= hyp_size && j >= meas_size)
      {
        // dummy hypothesis to dummy detection
      }
    }
  }
}

void MultiHypothesisTracker::filterWeakHypotheses()
{
  deleteWeakHypotheses();
  mergeCloseHypotheses();
}

void MultiHypothesisTracker::deleteWeakHypotheses()
{
  auto it = m_hypotheses.begin();
  while(it != m_hypotheses.end())
  {
    if((*it)->isWeak())
    {
      it = m_hypotheses.erase(it);
      continue;
    }
    ++it;
  }
}

void MultiHypothesisTracker::mergeCloseHypotheses()
{
  auto it1 = m_hypotheses.begin();
  while(it1 != m_hypotheses.end())
  {
    auto it2 = it1 + 1;
    while(it2 != m_hypotheses.end())
    {
      float distance_between_hypotheses = ((*it1)->getPosition() - (*it2)->getPosition()).norm();
      if(distance_between_hypotheses < m_distance_threshold_for_hypotheses_merge)
        it2 = m_hypotheses.erase(it2);
      else
        ++it2;
    }
    ++it1;
  }
}

};

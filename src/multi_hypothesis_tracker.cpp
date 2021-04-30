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

void MultiHypothesisTracker::setupCostMatrix(const Detections& detections,
                                             int**& cost_matrix)
{
  size_t hyp_size = m_hypotheses.size();
  size_t meas_size = detections.detections.size();
  size_t dim = hyp_size + meas_size;

  cost_matrix = new int* [dim];

  for(size_t i = 0; i < dim; i++)
  {
    cost_matrix[i] = new int[dim];

    for(size_t j = 0; j < dim; j++)
    {
      if(i < hyp_size && j < meas_size)
      {
        // Calculate distance between hypothesis and detection
        float distance = -1.0;
        if(m_use_bhattacharyya_for_assignments)
        {
          distance = bhattacharyya(m_hypotheses[i]->getPosition(),
                                   detections.detections[j].position.block<3, 1>(0, 0),
                                   m_hypotheses[i]->getCovariance(),
                                   detections.detections[j].covariance.block<3, 3>(0, 0));
        }
        else
        {
          distance = (m_hypotheses[i]->getPosition() - detections.detections[j].position.block<3, 1>(0, 0)).norm();
        }
        if(distance < m_max_correspondence_distance_for_assignments)
        {
          cost_matrix[i][j] = static_cast<int>((float)m_correspondence_distance_scale * distance);
        }
        else
        {
          cost_matrix[i][j] = DO_NOT_ASSIGN;
        }
      }
      else if(i < hyp_size && j >= meas_size)
      {
        // distance from a hypothesis to a dummy detection
        cost_matrix[i][j] = m_scaled_max_correspondence_distance_for_assignments;
      }
      else if(i >= hyp_size && j < meas_size)
      {
        // distance from a detection to a dummy hypothesis
        cost_matrix[i][j] = m_scaled_max_correspondence_distance_for_assignments;
      }
      else if(i >= hyp_size && j >= meas_size)
      {
        // distance from a dummy hypothesis to a dummy detection
        cost_matrix[i][j] = 0;
      }
    }
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

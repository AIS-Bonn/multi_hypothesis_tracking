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
  size_t number_of_real_hypotheses = m_hypotheses.size();
  size_t number_of_real_detections = detections.detections.size();
  size_t cost_matrix_dimension = number_of_real_hypotheses + number_of_real_detections;

  cost_matrix = new int* [cost_matrix_dimension];
  for(size_t hypothesis_id = 0; hypothesis_id < cost_matrix_dimension; hypothesis_id++)
  {
    cost_matrix[hypothesis_id] = new int[cost_matrix_dimension];
    for(size_t detection_id = 0; detection_id < cost_matrix_dimension; detection_id++)
    {
      if(isRealHypothesis(hypothesis_id, number_of_real_hypotheses) &&
         isRealDetection(detection_id, number_of_real_detections))
      {
        float distance = calculateDistance(m_hypotheses[hypothesis_id], 
                                           detections.detections[detection_id]);
        if(distance < m_max_correspondence_distance_for_assignments)
          cost_matrix[hypothesis_id][detection_id] = static_cast<int>((float)m_correspondence_distance_scale * distance);
        else
          cost_matrix[hypothesis_id][detection_id] = DO_NOT_ASSIGN;
      }
      else if((isRealHypothesis(hypothesis_id, number_of_real_hypotheses) &&
               isDummyDetection(detection_id, number_of_real_detections)) ||
              (isDummyHypothesis(hypothesis_id, number_of_real_hypotheses) &&
               isRealDetection(detection_id, number_of_real_detections)))
      {
        cost_matrix[hypothesis_id][detection_id] = m_scaled_max_correspondence_distance_for_assignments;
      }
      else if(isDummyHypothesis(hypothesis_id, number_of_real_hypotheses) &&
              isDummyDetection(detection_id, number_of_real_detections))
      {
        cost_matrix[hypothesis_id][detection_id] = 0;
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
  size_t number_of_real_hypotheses = m_hypotheses.size();
  size_t number_of_real_detections = detections.detections.size();
  size_t cost_matrix_dimension = number_of_real_hypotheses + number_of_real_detections;

  for(size_t hypothesis_id = 0; hypothesis_id < cost_matrix_dimension; hypothesis_id++)
  {
    for(size_t detection_id = 0; detection_id < cost_matrix_dimension; detection_id++)
    {
      if(assignments[hypothesis_id][detection_id] == HUNGARIAN_ASSIGNED)
      {
        if(isRealHypothesis(hypothesis_id, number_of_real_hypotheses) &&
           isRealDetection(detection_id, number_of_real_detections))
        {
          // if valid assignment -> correct hypothesis
          if(cost_matrix[hypothesis_id][detection_id] < m_scaled_max_correspondence_distance_for_assignments)
          {
            m_hypotheses[hypothesis_id]->correct(detections.detections[detection_id]);
          }
            // distance threshold exceeded -> create new hypothesis from detection
          else
          {
            m_hypotheses.emplace_back(m_hypothesis_factory->createHypothesis(detections.detections[detection_id],
                                                                             detections.time_stamp));
          }
        }
        else if(isDummyHypothesis(hypothesis_id, number_of_real_hypotheses) &&
                isRealDetection(detection_id, number_of_real_detections))
        {
          // if real detection assigned to dummy hypothesis -> create new hypothesis
          m_hypotheses.emplace_back(m_hypothesis_factory->createHypothesis(detections.detections[detection_id],
                                                                           detections.time_stamp));
        }
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

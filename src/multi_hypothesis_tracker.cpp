/** @file
 *
 * Multi hypothesis tracker implementation.
 *
 * @author Jan Razlaw
 */

#include <multi_hypothesis_tracking/multi_hypothesis_tracker.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTracker::MultiHypothesisTracker(std::shared_ptr<HypothesisFactory> hypothesis_factory)
  : m_hypothesis_factory(hypothesis_factory)
    , m_current_hypothesis_id(0)
    , m_use_bhattacharyya_for_assignments(true)
    , m_dist_scale(10000)
    , m_max_distance((int)(m_dist_scale * 20.0))
    , m_compute_likelihood(false)
    , m_likelihood_sum(0.f)
    , m_assigned_hypotheses_counter(0)
{
}

void MultiHypothesisTracker::predict(double time_diff)
{
  for(auto& hypothesis : m_hypotheses)
    hypothesis->predict(time_diff);
}

void MultiHypothesisTracker::predict(double time_diff,
                                     Eigen::Vector3f& control)
{
  for(auto& hypothesis : m_hypotheses)
    hypothesis->predict(time_diff, control);
}

void MultiHypothesisTracker::correct(const std::vector<Detection>& detections)
{
  if(detections.empty())
    return;

  int** cost_matrix;
  setupCostMatrix(detections, m_hypotheses, cost_matrix);

  hungarian_problem_t hung;
  size_t dim = detections.size() + m_hypotheses.size();
  hungarian_init(&hung, cost_matrix, dim, dim, HUNGARIAN_MODE_MINIMIZE_COST);

// 		hungarian_print_costmatrix(&hung);
  hungarian_solve(&hung);
// 		hungarian_print_assignment(&hung);

  applyAssignments(hung.assignment, cost_matrix, detections, m_hypotheses);

  for(size_t i = 0; i < dim; i++)
    delete[] cost_matrix[i];
  delete[] cost_matrix;
  hungarian_free(&hung);
}

void MultiHypothesisTracker::setupCostMatrix(const std::vector<Detection>& detections,
                                             std::vector<std::shared_ptr<Hypothesis>>& hypotheses,
                                             int**& cost_matrix)
{
  size_t hyp_size = hypotheses.size();
  size_t meas_size = detections.size();
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
        double distance = -1.0;
        if(m_use_bhattacharyya_for_assignments)
        {
          distance = bhattacharyya(m_hypotheses[i]->getPosition(),
                                   detections[j].position.block<3, 1>(0, 0),
                                   m_hypotheses[i]->getCovariance(),
                                   detections[j].covariance.block<3, 3>(0, 0));
        }
        else
        {
          distance = (m_hypotheses[i]->getPosition() - detections[j].position.block<3, 1>(0, 0)).norm();
        }
        int scaled_distance = (int)(m_dist_scale * distance);
        if(scaled_distance < m_max_distance)
        {
          cost_matrix[i][j] = scaled_distance;
        }
        else
        {
          // if threshold exceeded, make sure assignment algorithm doesn't assign here
          cost_matrix[i][j] = INT_MAX;
        }
      }
      else if(i < hyp_size && j >= meas_size)
      {
        // distance from a hypothesis to a dummy detection
        cost_matrix[i][j] = m_max_distance;
      }
      else if(i >= hyp_size && j < meas_size)
      {
        // distance from a detection to a dummy hypothesis
        cost_matrix[i][j] = m_max_distance;
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
                                              const std::vector<Detection>& detections,
                                              std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(m_compute_likelihood)
    resetAverageLikelihood();

  size_t hyp_size = hypotheses.size();
  size_t meas_size = detections.size();
  size_t dim = hyp_size + meas_size;

  for(size_t i = 0; i < dim; i++)
  {
    for(size_t j = 0; j < dim; j++)
    {
      if(i < hyp_size && j < meas_size)
      {
        // if hypothesis assigned to detection and distance below threshold -> correct hypothesis
        if(assignments[i][j] == HUNGARIAN_ASSIGNED && cost_matrix[i][j] < m_max_distance)
        {
          if(m_compute_likelihood)
            updateLikelihoodSum(m_hypotheses[i]->computeLikelihood(detections[j]));

          m_hypotheses[i]->correct(detections[j]);
        }
        else if(assignments[i][j] == HUNGARIAN_ASSIGNED)
        {
          // if assigned but distance too high -> prohibited assignment -> hypothesis unassignment

          // create new hypothesis from detection
          m_hypotheses.emplace_back(m_hypothesis_factory->createHypothesis(detections[j], m_current_hypothesis_id++));
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
          m_hypotheses.emplace_back(m_hypothesis_factory->createHypothesis(detections[j], m_current_hypothesis_id++));
      }
      else if(i >= hyp_size && j >= meas_size)
      {
        // dummy hypothesis to dummy detection
      }
    }
  }
}

void MultiHypothesisTracker::deleteSpuriousHypotheses(float max_covariance)
{
  auto it = m_hypotheses.begin();
  while(it != m_hypotheses.end())
  {
    if((*it)->isSpurious(max_covariance))
    {
      it = m_hypotheses.erase(it);
      continue;
    }
    ++it;
  }
}

//TODO: implement a reasonable merging function.
void MultiHypothesisTracker::mergeCloseHypotheses(double distance_threshold)
{
  bool erased_it1 = false;
  // implicitly deletes those hypothesis with latter born time
  auto it1 = m_hypotheses.begin();
  while(it1 != m_hypotheses.end())
  {
    erased_it1 = false;
    auto it2 = it1 + 1;
    while(it2 != m_hypotheses.end())
    {
      double distance = ((*it1)->getPosition() - (*it2)->getPosition()).norm();

      if(distance < distance_threshold)
      {
        // TODO: decide which hypothesis to delete
        if(!(*it1)->isStatic())
        {
          it2 = m_hypotheses.erase(it2);
        }
        else
        {
          if(!(*it2)->isStatic())
          {
            it1 = m_hypotheses.erase(it1);
            erased_it1 = true;
            break;
          }
          else
          {
            it2 = m_hypotheses.erase(it2);
          }
        }

        continue;
      }
      ++it2;
    }
    if(erased_it1)
      continue;
    else
      ++it1;
  }
}

};

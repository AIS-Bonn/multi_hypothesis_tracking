/** @file
 *
 * Multi hypothesis tracker implementation.
 *
 * @author Jan Razlaw
 */

#include <multi_object_tracking/multi_hypothesis_tracker.h>

namespace MultiHypothesisTracker
{

MultiHypothesisTracker::MultiHypothesisTracker(std::shared_ptr<HypothesisFactory> hypothesis_factory)
:	m_hypothesis_factory(hypothesis_factory)
 , m_current_hypothesis_id(0)
 , m_dist_scale(10000)
 , m_max_bhattacharyya_distance((int)(m_dist_scale * 20.0))
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

void MultiHypothesisTracker::correct(const std::vector<Measurement>& measurements)
{
  if(measurements.empty())
    return;

  int **cost_matrix;
  setupCostMatrix(measurements, m_hypotheses, cost_matrix);

  hungarian_problem_t hung;
  size_t dim = measurements.size() + m_hypotheses.size();
  hungarian_init(&hung, cost_matrix, dim, dim, HUNGARIAN_MODE_MINIMIZE_COST);

// 		hungarian_print_costmatrix(&hung);
  hungarian_solve(&hung);
// 		hungarian_print_assignment(&hung);

  applyAssignments(hung.assignment, cost_matrix, measurements, m_hypotheses);

  for(size_t i = 0; i < dim; i++)
    delete[] cost_matrix[i];
  delete[] cost_matrix;
  hungarian_free(&hung);
}

void MultiHypothesisTracker::setupCostMatrix(const std::vector<Measurement>& measurements,
                                             std::vector<std::shared_ptr<Hypothesis>>& hypotheses,
                                             int**& cost_matrix)
{
  size_t hyp_size = hypotheses.size();
  size_t meas_size = measurements.size();
  size_t dim = hyp_size + meas_size;

  cost_matrix = new int*[dim];

  for(size_t i=0; i < dim; i++)
  {
    cost_matrix[i] = new int[dim];

    for(size_t j=0; j < dim; j++)
    {
      if(i < hyp_size && j < meas_size)
      {
        // Calculate distance between hypothesis and measurement
        double bhattacharyya_distance = bhattacharyya(m_hypotheses[i]->getPosition(),
                                                      measurements[j].pos.block<3,1>(0, 0),
                                                      m_hypotheses[i]->getCovariance(),
                                                      measurements[j].cov.block<3,3>(0,0));
        
        int scaled_bhattacharyya_distance = (int)(m_dist_scale * bhattacharyya_distance);
        if(scaled_bhattacharyya_distance < m_max_bhattacharyya_distance)
        {
          if(measurements[j].class_a_detection)
            cost_matrix[i][j] = scaled_bhattacharyya_distance;
          else
            cost_matrix[i][j] = m_max_bhattacharyya_distance - 1; // if detection is here because of loosend thresholds, set highest valid distance to only assign if no other option available
        }
        else
        {
          // if threshold exceeded, make sure assignment algorithm doesn't assign here
          cost_matrix[i][j] = INT_MAX;
        }
      }
      else if(i < hyp_size && j >= meas_size)
      {
        // distance from a hypothesis to a dummy measurement
        cost_matrix[i][j] = m_max_bhattacharyya_distance;
      }
      else if(i >= hyp_size && j < meas_size)
      {
        // distance from a measurement to a dummy hypothesis
        cost_matrix[i][j] = m_max_bhattacharyya_distance;
      }
      else if(i >= hyp_size && j >= meas_size)
      {
        // distance from a dummy hypothesis to a dummy measurement
        cost_matrix[i][j] = 0;
      }
    }
  }
}

void MultiHypothesisTracker::applyAssignments(int**& assignments,
                                              int**& cost_matrix,
                                              const std::vector<Measurement>& measurements,
                                              std::vector<std::shared_ptr<Hypothesis>>& hypotheses)
{
  if(m_compute_likelihood)
    resetAverageLikelihood();

  size_t hyp_size = hypotheses.size();
  size_t meas_size = measurements.size();
  size_t dim = hyp_size + meas_size;

  for(size_t i = 0; i < dim; i++)
  {
    for(size_t j = 0; j < dim; j++)
    {
      if(i < hyp_size && j < meas_size)
      {
        // if hypothesis assigned to measurement and distance below threshold -> correct hypothesis
        if(assignments[i][j] == HUNGARIAN_ASSIGNED && cost_matrix[i][j] < m_max_bhattacharyya_distance)
        {
          if(m_compute_likelihood)
            updateLikelihoodSum(m_hypotheses[i]->computeLikelihood(measurements[j]));

          m_hypotheses[i]->correct(measurements[j]);
        }
        else if(assignments[i][j] == HUNGARIAN_ASSIGNED)
        {
          // if assigned but distance too high -> prohibited assignment -> hypothesis unassignment

          // create new hypothesis from measurement
          m_hypotheses.emplace_back(m_hypothesis_factory->createHypothesis(measurements[j], m_current_hypothesis_id++));
        }
      }
      else if(i < hyp_size && j >= meas_size)
      {
        // if hypothesis assigned to dummy measurement -> failed to detect hypothesis
      }
      else if(i >= hyp_size && j < meas_size)
      {
        // if measurement assigned to dummy hypothesis AND is class_a_detection -> create new hypothesis
        if(assignments[i][j] == HUNGARIAN_ASSIGNED && measurements[j].class_a_detection)
          m_hypotheses.emplace_back(m_hypothesis_factory->createHypothesis(measurements[j], m_current_hypothesis_id++));
      }
      else if(i >= hyp_size && j >= meas_size)
      {
        // dummy hypothesis to dummy measurement
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
      it = erase(m_hypotheses, it);
      continue;
    }
    ++it;
  }
}

//TODO: implement a reasonable merging function.
void MultiHypothesisTracker::mergeCloseHypotheses(double distance_threshold)
{
  // implicitly deletes those hypothesis with latter born time
	auto it1 = m_hypotheses.begin();
	while(it1 != m_hypotheses.end())
	{
		auto it2 = it1 + 1;
		while(it2 != m_hypotheses.end())
		{
			double distance = ((*it1)->getPosition() - (*it2)->getPosition()).norm();

			if(distance < distance_threshold)
			{
			  // TODO: decide which hypothesis to delete
        it2 = erase(m_hypotheses, it2);
				continue;
			}
			++it2;
		}
		++it1;
	}
}

std::vector<std::shared_ptr<Hypothesis>>::iterator MultiHypothesisTracker::erase(std::vector<std::shared_ptr<Hypothesis>>& hypotheses,
                                                                                 std::vector<std::shared_ptr<Hypothesis>>::iterator& it)
{
  // copy static objects with short life time to heavens gate
  if((*it)->isStatic() && (*it)->getLastCorrectionTime() - (*it)->getBornTime() < 3.0)
    m_heavens_gate.push(*(*it));

  // delete iterator and return iterator to next object
  return hypotheses.erase(it);
}

};

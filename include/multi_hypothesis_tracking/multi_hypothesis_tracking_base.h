/** @file
 *
 * Multi hypothesis tracking node receiving detection messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#ifndef __MULTI_OBJECT_TRACKING_BASE_H__
#define __MULTI_OBJECT_TRACKING_BASE_H__

#include <chrono>
#include <fstream>
#include <iostream>

#include <multi_hypothesis_tracking/definitions.h>
#include <multi_hypothesis_tracking/multi_hypothesis_tracker.h>
#include <multi_hypothesis_tracking/visualizer/visualizations_publisher.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


namespace MultiHypothesisTracker
{

/**
 * @brief Ros node to track multiple hypotheses simultaneously.
 */
class MultiHypothesisTrackingBase
{
public:
  /** @brief Constructor. */
  MultiHypothesisTrackingBase();
  /** @brief Destructor. */
  ~MultiHypothesisTrackingBase(){ m_time_file.close(); };

  void getRosParameters();
  void prepareMeasuringProcessingTime();

  /**
   * @brief Publishes the hypotheses in several versions.
   */
  void publish(const ros::Time& stamp);

  /**
   * @brief Transforms detections to the target_frame.
   *
   * @param[in,out] detections   detections.
   * @param[in]     header         header - in case there are no detections we still want to continue to tell the tracker exactly that.
   * @param[in]     target_frame   frame the detections are transformed to.
   *
   * @return false if at least one detection couldn't be transformed, true otherwise
   */
  bool transformToFrame(std::vector<Detection>& detections,
                        const std_msgs::Header& header,
                        const std::string& target_frame);

  /**
   * @brief Gets the transform from source_frame to target_frame at time time_stamp.
   *
   * @return false if transform was not available, true otherwise
   */
  bool getTransform(const std::string& source_frame,
                    const std::string& target_frame,
                    const ros::Time& time_stamp,
                    tf::StampedTransform& transform);

  /** @brief Transform the detections using the given transform and replace their frame_id with the target_frame. */
  void transformDetections(std::vector<Detection>& detections,
                           const tf::StampedTransform& transform,
                           const std::string& target_frame);
                                                        
  /**
   * @brief Performs one prediction and correction step for every hypothesis.
   *
   * Invokes prediction step for every hypothesis.
   * Passes detections to multi hypothesis tracker for correction step.
   * Filters out weak hypotheses.
   *
   * @param detections    current detections.
   * @param time_stamp    time stamp of detections.
   */
  void processDetections(const std::vector<Detection>& detections,
                         const ros::Time& time_stamp);

  /** @brief Getter for hypotheses vector. */
  const std::vector<std::shared_ptr<Hypothesis>>& getHypotheses();

public:

  /** @brief Publishes results. */
  VisualizationsPublisher m_visualizations_publisher;

  /** @brief Provides transforms to world frame. */
  std::shared_ptr<tf::TransformListener> m_transform_listener;

  /** @brief The functionality. */
  MultiHypothesisTracker m_multi_hypothesis_tracker;


  //Params
  /** @brief Fixed frame the detections and tracks are in. */
  std::string m_world_frame;
  /** @brief Hypotheses are merged if their distance is below this parameter. */
  double m_merge_distance;
  /** @brief Hypotheses are deleted if their covariance is above this parameter. */
  float m_max_covariance;
  /** @brief If true, the likelihood of the detections given the hypotheses' states is computed. */
  bool m_compute_likelihood;

  /** @brief Time when the last prediction was performed. */
  double m_last_prediction_time;

  std::string m_input_topic;
  bool m_measure_time;
  std::chrono::microseconds m_summed_time_for_callbacks;
  int m_number_of_callbacks;
  std::ofstream m_time_file;
  bool m_got_first_detections;
};

}

#endif

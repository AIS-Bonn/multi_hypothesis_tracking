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
  void updateProcessingTimeMeasurements(std::chrono::high_resolution_clock::time_point callback_start_time);

  /**
   * @brief Publishes the hypotheses in several versions.
   */
  void publish(const ros::Time& stamp);

  /**
   * @brief Transforms detections to the target_frame.
   *
   * @param[in,out] detections      detections.
   * @param[in]     target_frame    frame the detections are transformed to.
   *
   * @return false if at least one detection couldn't be transformed, true otherwise
   */
  bool transformToFrame(Detections& detections,
                        const std::string& target_frame);

  /**
   * @brief Gets the transform from source_frame to target_frame at time time_stamp.
   *
   * @return false if transform was not available, true otherwise
   */
  bool getTransform(const std::string& source_frame,
                    const std::string& target_frame,
                    const double time_stamp,
                    tf::StampedTransform& transform);

  /** @brief Transform the detections using the given transform and replace their frame_id with the target_frame. */
  void transformDetections(Detections& detections,
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
   */
  void processDetections(const Detections& detections);

  /** @brief Calls methods of tracker to delete weak hypotheses. */
  void filterWeakHypotheses();

  /** @brief Getter for hypotheses vector. */
  const std::vector<std::shared_ptr<Hypothesis>>& getHypotheses();

public:
  // Member variables
  /** @brief The functionality. */
  MultiHypothesisTracker m_multi_hypothesis_tracker;
  /** @brief Publishes results. */
  VisualizationsPublisher m_visualizations_publisher;
  /** @brief Provides transforms to the specified world frame. */
  std::shared_ptr<tf::TransformListener> m_transform_listener;
  /** @brief Time when the last prediction was performed. */
  double m_last_prediction_time;
  
  // Parameters retrievable from the ros parameter server
  /** @brief The name of the topic providing detections as inputs. */
  std::string m_input_topic;
  /** @brief Fixed frame the detections and tracks are in. */
  std::string m_world_frame_id;
  /** @brief Hypothesis is deleted if one eigen value of its covariance matrix is greater than this parameter. */
  float m_maximally_allowed_hypothesis_covariance;
  /** @brief Processing time per callback is measured if true. */
  bool m_measure_processing_time;
  
  // Variables for processing time measuring
  std::chrono::microseconds m_summed_time_for_callbacks;
  int m_number_of_callbacks;
  std::ofstream m_time_file;
  bool m_processed_first_detections;
};

}

#endif

/** @file
 *
 * Multi hypothesis tracking base node. 
 * Provides basic functionality using the multi hypothesis tracker.
 * Inheriting nodes have to receive detection messages and provide those to the processDetections method.
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

class MultiHypothesisTrackingBase
{
public:
  MultiHypothesisTrackingBase();
  ~MultiHypothesisTrackingBase(){ if(m_measure_processing_time) m_time_file.close(); };

  /** @brief Sets a suitable HypothesisFactory for the #m_multi_hypothesis_tracker.
   * @see MultiHypothesisTracker::setHypothesisFactory() */
  virtual void initializeHypothesisFactory(const ros::NodeHandle& private_node_handle) = 0;

  void getRosParameters();
  void prepareMeasuringProcessingTime();
  void updateProcessingTimeMeasurements(std::chrono::high_resolution_clock::time_point callback_start_time);

  /** @brief Publishes the detections and hypotheses states - for visualization and further processing. */
  void publishVisualizations(const Detections& detections);

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

  /** @brief Transforms the detections using the given transform and replaces their frame_id with the target_frame. */
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
   * @param[in] detections    current detections.
   */
  void processDetections(const Detections& detections);

  /** @brief Getter for hypotheses vector. */
  const std::vector<std::shared_ptr<HypothesisInterface>>& getHypotheses(){ return m_multi_hypothesis_tracker.getHypotheses(); };

public:
  // Member variables
  /** @brief The tracking functionality. */
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

/** @file
 *
 * Multi human tracking node receiving human messages and providing those to the multi hypothesis tracker.
 *
 * @author Jan Razlaw
 */

#ifndef __MULTI_HUMAN_TRACKING_NODE_H__
#define __MULTI_HUMAN_TRACKING_NODE_H__

#include <chrono>
#include <fstream>
#include <iostream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>

#include <person_msgs/PersonCovList.h>

#include <multi_hypothesis_tracking/multi_hypothesis_tracker.h>
#include <multi_hypothesis_tracking/mot_publisher.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


namespace MultiHypothesisTracker
{

typedef person_msgs::PersonCovList HumanMsg;

/**
 * @brief Ros node to track multiple hypotheses simultaneously.
 */
class Tracker
{
public:
  /** @brief Constructor. */
  Tracker();
  /** @brief Destructor. */
  ~Tracker(){ m_time_file.close(); };

  /**
   * @brief Publishes the hypotheses in several versions.
   */
  void publish(const ros::Time& stamp);

  /**
   * @brief Callback function for HumanMsg messages.
   *
   * Converts messages to detections.
   * Transforms detections to #m_world_frame and passes those to the tracking algorithm.
   *
   * @param [in] msg    detections.
   */
  void detectionCallback(const HumanMsg::ConstPtr& msg);

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
   * @brief Converts the human detections into the internal format.
   *
   * @param[in]     msg             detections.
   * @param[out]    detections    detections in tracker format.
   */
  void convert(const HumanMsg::ConstPtr& msg,
               std::vector<Detection>& detections);

  /**
   * @brief Performs one prediction and correction step for every hypothesis.
   *
   * Invokes prediction step for every hypothesis.
   * Passes detections to multi hypothesis tracker for correction step.
   * Filters out weak hypotheses.
   *
   * @param detections    new detections.
   */
  void processDetections(const std::vector<Detection>& detections);

  /** @brief Getter for hypotheses vector. */
  const std::vector<std::shared_ptr<Hypothesis>>& getHypotheses();

  /** @brief Getter for hypotheses that are about to get deleted. */
  std::queue<Hypothesis>& getDeletedHypotheses();

private:
  /** @brief Subscribes to detections. */
  ros::Subscriber m_human_detection_subscriber;
  /** @brief Publishes results. */
  MOTPublisher m_mot_publisher;

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

  bool m_measure_time;
  std::chrono::microseconds m_summed_time_for_callbacks;
  int m_number_of_callbacks;
  std::ofstream m_time_file;
  bool m_got_first_detections;
};

}

#endif

# Usage

This package depends on the [multi_hypothesis_tracker_msgs](https://github.com/AIS-Bonn/multi_hypothesis_tracking_msgs) package.

The master branch provides the functionality to track objects subscribing to detections either in the [multi_hypothesis_tracker_msgs/ObjectDetections](https://git.ais.uni-bonn.de/multi_hypothesis_tracking/multi_hypothesis_tracking_msgs/-/blob/master/msg/ObjectDetections.msg) format or in the geometry_msgs/PoseArray format.

Exemplary [nodes](../src/nodes) are provided, showing how to feed the detections to the tracker.  
- The [multi_hypothesis_tracking_node_for_pose_arrays](../src/nodes/multi_hypothesis_tracking_node_for_pose_arrays.cpp) serves as a minimal example subscribing to PoseArray messages.  
- The [multi_hypothesis_tracker_node_for_sparse_lidar](../src/nodes/multi_hypothesis_tracking_node_for_sparse_lidar.cpp) was utilized for our paper ["Detection and Tracking of Small Objects in Sparse 3D Laser Range Data"](https://arxiv.org/abs/1903.05889) subscribing to ObjectDetections messages.


# Install


Clone both packages into your catkin workspace and build them the usual way. 

# Demo 


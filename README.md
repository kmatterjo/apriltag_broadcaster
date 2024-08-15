# apriltag_broadcaster

## Overview

This package is meant as an extension for the apriltag_ros repository allowing to publish the tf's of the detected tags with respect to a global frame. In addition, this node will save the relative pose between tags in order to interpolate and broadcast the tf of tags even if they are not currently being detected. 

## Installation

### Dependencies

For installing the apriltag_ros and apriltag repository, refer to the apriltag_ros documentation.

- [apriltag](https://github.com/AprilRobotics/apriltag) (visual fiducial system library)
- [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) (ROS wrapper for the apriltag library)

## Usage

Define the global frame relative to which the tf's should be broadcasted in the **config.yaml** file. For defining the tag family and ID's that you are using, refer to the apriltag_ros documentation. 

Run the extension including the apriltag_ros package with:

	roslaunch apriltag_broadcaster apriltag_broadcaster.launch

## Nodes

### apriltag_broadcaster_node

the apriltag_broadcaster_node subscribes to the detection topic from apriltag_ros and transforms the poses to the global frame before broadcasting them. 

#### Subscribed Topics

* **`/tag_detections`** ([apriltag_ros::AprilTagDetectionArray])

#### Parameters

* **`frames/world`** (string, default: "map")

	The name of the global world frame


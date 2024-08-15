#ifndef TAG_BROADCASTER_HPP
#define TAG_BROADCASTER_HPP

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

#include <apriltag_broadcaster/GetTagPoses.h>

using Detec = apriltag_ros::AprilTagDetection;
using DetecArray = apriltag_ros::AprilTagDetectionArray;

class TagBroadcaster {
  public:
    TagBroadcaster();
    ~TagBroadcaster() {}

    void tagCallback(const DetecArray::Ptr& tag_detection_array);
    void updateTfMap(const DetecArray::ConstPtr& tag_detection_array);
    DetecArray::ConstPtr interpolatePoses(const DetecArray::Ptr& tag_detection_array);

  private:
    bool verbose_;
    ros::NodeHandle nh_;
    std::unique_ptr<tf::TransformListener> tf_listener_;
    std::unique_ptr<tf::TransformBroadcaster> tf_pub_;
    std::string world_frame_;
    std::map<std::pair<std::string, std::string>, tf::Transform> tf_map_;

    std::unique_ptr<geometry_msgs::PoseStamped> tag_5_pose_ = nullptr;
    std::unique_ptr<geometry_msgs::PoseStamped> tag_6_pose_ = nullptr;
    std::unique_ptr<geometry_msgs::PoseStamped> tag_7_pose_ = nullptr;
};

#endif // TAG_BROADCASTER_HPP
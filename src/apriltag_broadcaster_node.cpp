#include <apriltag_broadcaster/TagBroadcaster.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "apriltag_broadcaster_node");
    ros::NodeHandle nh;
    ROS_INFO("Started apriltag broadcaster node");

    // Subscribe to /tag_detections topic for broadcasting tag poses
    TagBroadcaster tag_bd;
    ros::Subscriber tag_sub =
        nh.subscribe("/tag_detections", 2, &TagBroadcaster::tagCallback, &tag_bd);

    ros::spin();

    return 0;
}
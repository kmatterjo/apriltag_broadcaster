#include "apriltag_broadcaster/TagBroadcaster.hpp"

TagBroadcaster::TagBroadcaster()
    : nh_(), tf_listener_(new tf::TransformListener), tf_pub_(new tf::TransformBroadcaster) {
    if (!nh_.getParam("/apriltag_broadcaster/verbose", verbose_)) {
        ROS_WARN("Failed to get param 'verbose'");
        verbose_ = false;
    }
    if (!nh_.getParam("/apriltag_broadcaster/frames/world", world_frame_)) {
        ROS_WARN("Failed to get param 'world'");
        world_frame_ = "map";
    }
}

void TagBroadcaster::tagCallback(const DetecArray::Ptr& tag_detection_array) {
    // Proceed only if there are detected tags
    if (tag_detection_array->detections.size() != 0) {

        // Update relative tag poses in the map if applicable
        updateTfMap(tag_detection_array);

        tf::StampedTransform camera_tf;
        try {
            tf_listener_->lookupTransform(world_frame_,
                                          tag_detection_array->header.frame_id,
                                          tag_detection_array->header.stamp,
                                          camera_tf);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            return;
        }

        // Interpolate poses of unknown tags from the map
        auto interpolated_array = interpolatePoses(tag_detection_array);
        if (verbose_)
            ROS_INFO_THROTTLE(2,
                              "directly detected tags: %zu, interpolated tags: %zu",
                              tag_detection_array->detections.size(),
                              interpolated_array->detections.size());

        // Broadcast tf for each tag
        for (unsigned int i = 0; i < interpolated_array->detections.size(); i++) {
            geometry_msgs::PoseStamped pose;
            pose.pose = interpolated_array->detections[i].pose.pose.pose;
            pose.header = interpolated_array->detections[i].pose.header;

            tf::Stamped<tf::Transform> tag_transform;
            tf::poseStampedMsgToTF(pose, tag_transform);

            // Transform tag pose to map frame and publish tf
            tag_transform.setData(camera_tf * tag_transform);
            tf_pub_->sendTransform(tf::StampedTransform(
                tag_transform,
                interpolated_array->header.stamp,
                world_frame_,
                "tag_" + std::to_string(interpolated_array->detections[i].id[0])));
        }
    }
}

void TagBroadcaster::updateTfMap(const DetecArray::ConstPtr& tag_detection_array) {
    int N = tag_detection_array->detections.size();
    if (N < 2) {
        return;
    }

    // Update tf_map with relative poses between tags
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            std::pair<std::string, std::string> tag_pair =
                std::make_pair("tag_" + std::to_string(tag_detection_array->detections[i].id[0]),
                               "tag_" + std::to_string(tag_detection_array->detections[j].id[0]));
            std::pair<std::string, std::string> tag_pair_inv =
                std::make_pair("tag_" + std::to_string(tag_detection_array->detections[j].id[0]),
                               "tag_" + std::to_string(tag_detection_array->detections[i].id[0]));

            if (tf_map_.find(tag_pair) == tf_map_.end()) {
                tf::Transform tag_i_tf;
                tf::Transform tag_j_tf;
                tf::poseMsgToTF(tag_detection_array->detections[i].pose.pose.pose, tag_i_tf);
                tf::poseMsgToTF(tag_detection_array->detections[j].pose.pose.pose, tag_j_tf);

                tf::Transform tag_i_to_j_tf = tag_i_tf.inverseTimes(tag_j_tf);
                tf_map_[tag_pair] = tag_i_to_j_tf;
                if (verbose_) {
                    ROS_INFO("Added relative pose between pair %s and %s to tf_map",
                             tag_pair.first.c_str(),
                             tag_pair.second.c_str());
                    ROS_INFO("translation is: %f, %f, %f",
                             tag_i_to_j_tf.getOrigin().x(),
                             tag_i_to_j_tf.getOrigin().y(),
                             tag_i_to_j_tf.getOrigin().z());
                }

                // Add inverse pose to tf_map
                tf::Transform tag_j_to_i_tf = tag_i_to_j_tf.inverse();
                tf_map_[tag_pair_inv] = tag_j_to_i_tf;
                if (verbose_) {
                    ROS_INFO("Added relative pose between pair %s and %s to tf_map",
                             tag_pair_inv.first.c_str(),
                             tag_pair_inv.second.c_str());
                    ROS_INFO("translation is: %f, %f, %f",
                             tag_j_to_i_tf.getOrigin().x(),
                             tag_j_to_i_tf.getOrigin().y(),
                             tag_j_to_i_tf.getOrigin().z());
                }
            }
        }
    }
}

DetecArray::ConstPtr TagBroadcaster::interpolatePoses(const DetecArray::Ptr& tag_detection_array) {
    int N = tag_detection_array->detections.size();

    std::vector<int> detected_tags;
    for (int i = 0; i < N; i++) {
        detected_tags.push_back(tag_detection_array->detections[i].id[0]);
    }

    for (int i = 0; i < 5; i++) {
        if (std::find(detected_tags.begin(), detected_tags.end(), i) == detected_tags.end()) {
            // Try to interpolate pose of tag i
            for (int j = 0; j < N; j++) {
                std::pair<std::string, std::string> tag_pair = std::make_pair(
                    "tag_" + std::to_string(tag_detection_array->detections[j].id[0]),
                    "tag_" + std::to_string(i));
                if (tf_map_.find(tag_pair) != tf_map_.end()) {
                    // Found relative pose between tag i and tag j
                    tf::Transform tag_j_tf;
                    tf::poseMsgToTF(tag_detection_array->detections[j].pose.pose.pose, tag_j_tf);
                    tf::Transform tag_i_tf = tag_j_tf * tf_map_[tag_pair];
                    geometry_msgs::Pose tag_i_pose;
                    tf::poseTFToMsg(tag_i_tf, tag_i_pose);

                    // Add tag i to tag_detection_array
                    Detec tag_i_detection;
                    tag_i_detection.id.push_back(i);
                    tag_i_detection.pose.pose.pose = tag_i_pose;
                    tag_i_detection.pose.header = tag_detection_array->detections[j].pose.header;
                    tag_detection_array->detections.push_back(tag_i_detection);

                    break;
                }
            }
        }
    }
    return tag_detection_array;
}
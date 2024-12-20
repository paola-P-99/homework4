#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

class ArucoTf : public rclcpp::Node {
public:
    ArucoTf()
        : Node("aruco_map_tf"),
          tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
          tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
          tf_broadcaster_(std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)),
          aruco_detected_(false), pose_stabilized_(false) {
        // Subscriber to ArUco pose
        aruco_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10, std::bind(&ArucoTf::aruco_marker_callback, this, std::placeholders::_1));
    }

private:
    void aruco_marker_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // If we already have a stabilized pose, don't process further
        if (pose_stabilized_) {
            return;
        }

        aruco_detected_ = true;
        geometry_msgs::msg::PoseStamped transformed_pose;

        try {
            // Lookup required transforms
            auto transform_camera_optical_to_camera = 
                tf_buffer_->lookupTransform("camera_link", "camera_link_optical", tf2::TimePointZero);
            auto transform_camera_to_base_link = 
                tf_buffer_->lookupTransform("base_link", "camera_link", tf2::TimePointZero);
            auto transform_base_link_to_base_footprint = 
                tf_buffer_->lookupTransform("base_footprint", "base_link", tf2::TimePointZero);
            auto transform_base_footprint_to_odom = 
                tf_buffer_->lookupTransform("fra2mo/odom", "base_footprint", tf2::TimePointZero);
            auto transform_odom_to_map = 
                tf_buffer_->lookupTransform("map", "fra2mo/odom", tf2::TimePointZero);

            // Transform the pose through all frames
            geometry_msgs::msg::PoseStamped pose_in_camera_link;
            tf2::doTransform(*msg, pose_in_camera_link, transform_camera_optical_to_camera);

            geometry_msgs::msg::PoseStamped pose_in_base_link;
            tf2::doTransform(pose_in_camera_link, pose_in_base_link, transform_camera_to_base_link);

            geometry_msgs::msg::PoseStamped pose_in_base_footprint;
            tf2::doTransform(pose_in_base_link, pose_in_base_footprint, transform_base_link_to_base_footprint);

            geometry_msgs::msg::PoseStamped pose_in_odom;
            tf2::doTransform(pose_in_base_footprint, pose_in_odom, transform_base_footprint_to_odom);

            tf2::doTransform(pose_in_odom, transformed_pose, transform_odom_to_map);

            // Stabilize pose after waiting a few cycles (simulate stabilization)
            rclcpp::Rate stabilization_rate(2);  // Wait 2 seconds for stabilization
            stabilization_rate.sleep();

            // Save the transformed pose once stabilized
            saved_pose_ = transformed_pose;
            pose_stabilized_ = true;

            // Log the transformed pose in the map frame
            RCLCPP_INFO(this->get_logger(), "Aruco pose transformed to map frame:");
            RCLCPP_INFO(this->get_logger(), "Position (x, y, z): (%f, %f, %f)",
                        saved_pose_.pose.position.x, saved_pose_.pose.position.y, saved_pose_.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Orientation (x, y, z, w): (%f, %f, %f, %f)",
                        saved_pose_.pose.orientation.x, saved_pose_.pose.orientation.y,
                        saved_pose_.pose.orientation.z, saved_pose_.pose.orientation.w);

            // Broadcast the static transform (tf_static)
            geometry_msgs::msg::TransformStamped tf_stamped;
            tf_stamped.header.stamp = this->now();
            tf_stamped.header.frame_id = "map";
            tf_stamped.child_frame_id = "aruco_marker";

            tf_stamped.transform.translation.x = saved_pose_.pose.position.x;
            tf_stamped.transform.translation.y = saved_pose_.pose.position.y;
            tf_stamped.transform.translation.z = saved_pose_.pose.position.z;

            tf_stamped.transform.rotation.x = saved_pose_.pose.orientation.x;
            tf_stamped.transform.rotation.y = saved_pose_.pose.orientation.y;
            tf_stamped.transform.rotation.z = saved_pose_.pose.orientation.z;
            tf_stamped.transform.rotation.w = saved_pose_.pose.orientation.w;

            // Publish static transform
            tf_broadcaster_->sendTransform(tf_stamped);

            // Log debug 
            RCLCPP_INFO(this->get_logger(),
                     "Published static transform: [frame: map -> aruco_marker]");
            RCLCPP_INFO(this->get_logger(),
            "Translation:\n"
            "    x : %.2f\n"
            "    y : %.2f\n"
            "    z : %.2f\n"
            "Orientation:\n"
            "    x : %.2f\n"
            "    y : %.2f\n"
            "    z : %.2f\n"
            "    w : %.2f",
            saved_pose_.pose.position.x, saved_pose_.pose.position.y, saved_pose_.pose.position.z,
            saved_pose_.pose.orientation.x, saved_pose_.pose.orientation.y, saved_pose_.pose.orientation.z,
            saved_pose_.pose.orientation.w);

        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        }
    }

    // Members
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_subscriber_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    bool aruco_detected_;
    bool pose_stabilized_;
    geometry_msgs::msg::PoseStamped saved_pose_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoTf>());
    rclcpp::shutdown();
    return 0;
}
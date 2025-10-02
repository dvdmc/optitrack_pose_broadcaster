#ifndef GT_POSE_BROADCASTER_HPP_
#define GT_POSE_BROADCASTER_HPP_

#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // This package is old and this import is unintuitive. Currently needed for tf2::toMsg. Remove ASAP.
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "rclcpp/rclcpp.hpp"
 
 namespace optitrack_pose_broadcaster {
 class GTPoseBroadcaster : public rclcpp::Node {
   rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr
       rigid_bodies_sub_;
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
 
 
   tf2::Transform earth_to_map_ = tf2::Transform::getIdentity();
   const tf2::Transform map_to_odom_ =
       tf2::Transform::getIdentity();  // ALWAYS IDENTITY
   tf2::Transform odom_to_base_ = tf2::Transform::getIdentity();
   tf2::Transform rigid_body_to_base_; // From the mocap detected rigid_body to the base of the robot 
 
   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
 
   bool has_earth_to_map_ = false;
   std::string mocap_topic_ = "/rigid_bodies";
   std::string rigid_body_name_;
   std::string earth_frame_;
   std::string map_frame_;
   std::string odom_frame_;
   std::string base_frame_;
   std::string rigid_body_frame_;
   double orientation_alpha_ = 1.0;
   geometry_msgs::msg::PoseStamped last_pose_msg_;
 
  public:
   explicit GTPoseBroadcaster(
       const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
       : Node("gt_pose_broadcaster", options)
      {
       tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
       tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
       tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
       static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
 
     this->declare_parameter<std::string>("rigid_body_name", "2");
     this->declare_parameter<double>("orientation_smooth_filter", 1.0);
     this->declare_parameter<std::string>("earth_frame", "earth");
     this->declare_parameter<std::string>("map_frame", "map");
     this->declare_parameter<std::string>("odom_frame", "odom");
     this->declare_parameter<std::string>("base_frame", "base_footprint");
     this->declare_parameter<std::string>("rigid_body_frame", "base_mocap");

     this->get_parameter("rigid_body_name", rigid_body_name_);
     if (rigid_body_name_.empty()) {
       RCLCPP_ERROR(this->get_logger(), "Rigid body name is empty");
       rclcpp::shutdown();
     }
 
     try {
       this->get_parameter("orientation_smooth_filter", orientation_alpha_);
     } catch (const rclcpp::ParameterTypeException& e) {
       RCLCPP_INFO(this->get_logger(),
                   "Parameter 'orientation_smooth_filter_cte' not set. Filter "
                   "disabled. Using "
                   "default value: %f",
                   orientation_alpha_);
     }
     this->get_parameter("earth_frame", earth_frame_);
     if (earth_frame_.empty()) {
       RCLCPP_ERROR(this->get_logger(), "Earth frame is empty");
       rclcpp::shutdown();
     }
     this->get_parameter("map_frame", map_frame_);
     if (map_frame_.empty()) {
       RCLCPP_ERROR(this->get_logger(), "Map frame is empty");
       rclcpp::shutdown();
     }
     this->get_parameter("odom_frame", odom_frame_);
     if (odom_frame_.empty()) {
       RCLCPP_ERROR(this->get_logger(), "Odom frame is empty");
       rclcpp::shutdown();
     }
     this->get_parameter("base_frame", base_frame_);
     if (base_frame_.empty()) {
       RCLCPP_ERROR(this->get_logger(), "Base frame is empty");
       rclcpp::shutdown();
     }
     this->get_parameter("rigid_body_frame", rigid_body_frame_);
     if (rigid_body_frame_.empty()) {
       RCLCPP_ERROR(this->get_logger(), "Rigid body frame is empty");
       rclcpp::shutdown();
     }
 
     rigid_bodies_sub_ =
         this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
             mocap_topic_, rclcpp::QoS(1000),
             std::bind(&GTPoseBroadcaster::rigid_bodies_callback, this,
                       std::placeholders::_1));
 
     pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
         "gt_pose", rclcpp::QoS(1));
 
     geometry_msgs::msg::TransformStamped map_to_odom;
     map_to_odom.transform = tf2::toMsg(map_to_odom_);
     map_to_odom.header.frame_id = map_frame_;
     map_to_odom.header.stamp = this->now();
     map_to_odom.child_frame_id = odom_frame_;
 
     // publish_static_transform(map_to_odom);
 
     has_earth_to_map_ = false;
   }
 
  private:
   void rigid_bodies_callback(
       const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) {
     auto pose_msg = geometry_msgs::msg::PoseStamped();
     pose_msg.header = msg->header;
     for (const auto& rigid_body : msg->rigidbodies) {
       if (rigid_body.rigid_body_name == rigid_body_name_) {
         pose_msg.pose = rigid_body.pose;
         break;
       }
     }
     process_mocap_pose(pose_msg);
   }
 
   void process_mocap_pose(const geometry_msgs::msg::PoseStamped& msg) {
     // mocap_pose could have a different frame_id, we will publish the transform
     // from earth to base_link without checking origin frame_id
 
     // if (!has_earth_to_map_) {
     //   earth_to_map_ = tf2::Transform(
     //       tf2::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y,
     //                       msg.pose.orientation.z, msg.pose.orientation.w),
     //       tf2::Vector3(msg.pose.position.x, msg.pose.position.y,
     //                    msg.pose.position.z));
 
     //   geometry_msgs::msg::TransformStamped earth_to_map;
     //   earth_to_map.transform = tf2::toMsg(earth_to_map_);
     //   earth_to_map.header.stamp = msg.header.stamp;
     //   earth_to_map.header.frame_id = earth_frame_;
     //   earth_to_map.child_frame_id = map_frame_;
     //   publish_static_transform(earth_to_map);
     //   has_earth_to_map_ = true;
     // }
 
     // Try to listen to rigid_body_to_base
     try {
       auto rigid_body_to_base_msg = tf_buffer_->lookupTransform(
         rigid_body_frame_, base_frame_, tf2::TimePointZero);
 
       tf2::fromMsg(rigid_body_to_base_msg.transform, rigid_body_to_base_);
     } catch (tf2::TransformException& ex) {
       RCLCPP_WARN(
         get_logger(), "Transform %s->%s exception: [%s]",
         rigid_body_frame_.c_str(), base_frame_.c_str(), ex.what());
     }
 
     // TODO: Check this
     // Print rigid_body_to_base:
     RCLCPP_INFO(get_logger(), "Rigid body to base: %f %f %f %f %f %f %f",
     rigid_body_to_base_.getOrigin().getX(), rigid_body_to_base_.getOrigin().getY(),
     rigid_body_to_base_.getOrigin().getZ(),
     rigid_body_to_base_.getRotation().getX(), rigid_body_to_base_.getRotation().getY(),
     rigid_body_to_base_.getRotation().getZ(),
     rigid_body_to_base_.getRotation().getW());
 
     // Publish transform
     tf2::Transform earth_to_rigid_body = tf2::Transform(
             tf2::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y,
                             msg.pose.orientation.z, msg.pose.orientation.w),
             tf2::Vector3(msg.pose.position.x, msg.pose.position.y,
                          msg.pose.position.z));
 
     odom_to_base_ =
         map_to_odom_.inverse() * earth_to_map_.inverse() *
         earth_to_rigid_body * rigid_body_to_base_;
 
     // Print odom_to_base:
     RCLCPP_INFO(get_logger(), "Odom to base: %f %f %f %f %f %f %f",
     odom_to_base_.getOrigin().getX(), odom_to_base_.getOrigin().getY(),
     odom_to_base_.getOrigin().getZ(),
     odom_to_base_.getRotation().getX(), odom_to_base_.getRotation().getY(),
     odom_to_base_.getRotation().getZ(),
     odom_to_base_.getRotation().getW());
     
     geometry_msgs::msg::TransformStamped odom_to_base_msg;
     odom_to_base_msg.transform = tf2::toMsg(odom_to_base_);
     odom_to_base_msg.header.stamp = msg.header.stamp;
     odom_to_base_msg.header.frame_id = odom_frame_;
     odom_to_base_msg.child_frame_id = base_frame_;
     publish_transform(odom_to_base_msg);
 
     // Publish pose
     geometry_msgs::msg::PoseStamped pose_msg;
     pose_msg.header.stamp = msg.header.stamp;
     pose_msg.header.frame_id = earth_frame_;
     pose_msg.pose = msg.pose;
     pose_msg.pose.orientation.x =
         orientation_alpha_ * msg.pose.orientation.x +
         (1 - orientation_alpha_) * last_pose_msg_.pose.orientation.x;
     pose_msg.pose.orientation.y =
         orientation_alpha_ * msg.pose.orientation.y +
         (1 - orientation_alpha_) * last_pose_msg_.pose.orientation.y;
     pose_msg.pose.orientation.z =
         orientation_alpha_ * msg.pose.orientation.z +
         (1 - orientation_alpha_) * last_pose_msg_.pose.orientation.z;
     pose_msg.pose.orientation.w =
         orientation_alpha_ * msg.pose.orientation.w +
         (1 - orientation_alpha_) * last_pose_msg_.pose.orientation.w;
     publish_pose(pose_msg);
 
     last_pose_msg_ = pose_msg;
   }
 
   inline void publish_transform(const geometry_msgs::msg::TransformStamped & transform)
   {
     tf_broadcaster_->sendTransform(transform);
   }
   inline void publish_static_transform(const geometry_msgs::msg::TransformStamped & transform)
   {
     static_tf_broadcaster_->sendTransform(transform);
   }
   inline void publish_pose(const geometry_msgs::msg::PoseStamped & pose)
   {
     pose_pub_->publish(pose);
   }
 
 };
 
 }  // namespace optitrack_pose_broadcaster
 
 #endif  // GT_POSE_BROADCASTER_HPP_
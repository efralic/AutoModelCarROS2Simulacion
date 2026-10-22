#ifndef AUTONOMOS_PLUGIN_HPP_
#define AUTONOMOS_PLUGIN_HPP_

// Plugin de Gazebo que mueve el AutoNOMOS_mini (steer_joint + ruedas traseras)
// a partir de dos tópicos de control: .../manual_control/steering y
// .../manual_control/speed. Puerto a ROS 2 Humble del autonomos_plugin.cc
// original (ROS 1) del repo AutomodelCar2026.
//
// Diferencias frente a la versión ROS 1:
//   - ros::NodeHandle / ros::Publisher / ros::Subscriber  -> gazebo_ros::Node
//     + rclcpp::Subscription (gazebo_ros_pkgs crea y hace "spin" del nodo
//     automáticamente; ya no hace falta rosQueue/QueueThread a mano).
//   - std_msgs::Int16ConstPtr -> std_msgs::msg::Int16::SharedPtr
//   - Se quitaron las ramas `#if GAZEBO_VERSION_MAJOR >= 8` porque Gazebo 11
//     (el que usa ROS 2 Humble) siempre las cumple.
//   - Se quitó el publicador de pose (PubQueue/geometry_msgs::Pose2D) porque
//     en el original nunca llegó a activarse (estaba comentado). Si lo
//     necesitas, se puede agregar como un rclcpp::Publisher normal.

#include <gazebo/common/CommonIface.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

namespace gazebo
{

class AutonomosPlugin : public ModelPlugin
{
public:
  AutonomosPlugin();
  ~AutonomosPlugin() override;

  /// Se llama cuando Gazebo carga el plugin sobre el modelo del carrito.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  void OnRosMsgSteering(const std_msgs::msg::Int16::SharedPtr _msg);
  void OnRosMsgVel(const std_msgs::msg::Int16::SharedPtr _msg);
  void OnUpdate(const common::UpdateInfo & _info);

  // Objetivos calculados a partir de los mensajes ROS (mismo mapeo que en ROS1)
  float position_{0.0f};
  float vel_{0.0f};

  physics::ModelPtr model_;
  physics::JointPtr joint_;             // steer_joint
  physics::JointPtr joint_left_wheel_;  // back_left_wheel_joint
  physics::JointPtr joint_right_wheel_; // back_right_wheel_joint

  common::PID pid_;
  common::PID pid_vel_left_;
  common::PID pid_vel_right_;

  common::Time prev_update_time_;
  event::ConnectionPtr update_connection_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr steering_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr vel_sub_;
};

}  // namespace gazebo

#endif  // AUTONOMOS_PLUGIN_HPP_

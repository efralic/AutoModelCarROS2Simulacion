#include "autonomos_plugin.hpp"

#include <string>

#define ADJ_FACT (1.0 / 12.0)  // original 1/50, ver comentario en OnRosMsgVel

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(AutonomosPlugin)

AutonomosPlugin::AutonomosPlugin() {}
AutonomosPlugin::~AutonomosPlugin() {}

void AutonomosPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (_model->GetJointCount() == 0) {
    std::cerr << "Invalid joint count, autonomos_plugin not loaded\n";
    return;
  }

  this->model_ = _model;

  // Se asume que el modelo tiene estos tres joints (ver AutoNOMOS_mini/model.sdf)
  this->joint_ = _model->GetJoint("steer_joint");
  this->joint_left_wheel_ = _model->GetJoint("back_left_wheel_joint");
  this->joint_right_wheel_ = _model->GetJoint("back_right_wheel_joint");

  // Mismas ganancias PID que en la versión ROS 1
  this->pid_ = common::PID(0.1, 0.001, 0.001);
  this->pid_vel_left_ = common::PID(0.1, 0.001, 0.001);
  this->pid_vel_right_ = common::PID(0.1, 0.001, 0.001);

  this->model_->GetJointController()->SetPositionPID(
    this->joint_->GetScopedName(), this->pid_);
  this->model_->GetJointController()->SetVelocityPID(
    this->joint_left_wheel_->GetScopedName(), this->pid_vel_left_);
  this->model_->GetJointController()->SetVelocityPID(
    this->joint_right_wheel_->GetScopedName(), this->pid_vel_right_);

  this->position_ = 0;
  this->vel_ = 0;

  // gazebo_ros_pkgs crea (o reutiliza) el nodo ROS 2 y lo hace "spin" solo;
  // no hace falta ros::init ni un hilo de cola manual como en ROS 1.
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);
  const gazebo_ros::QoS & qos = this->ros_node_->get_qos();

  const std::string steering_topic =
    "/" + this->model_->GetName() + "/manual_control/steering";
  const std::string vel_topic =
    "/" + this->model_->GetName() + "/manual_control/speed";

  this->steering_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Int16>(
    steering_topic, qos.get_subscription_qos(steering_topic, rclcpp::QoS(1)),
    std::bind(&AutonomosPlugin::OnRosMsgSteering, this, std::placeholders::_1));

  this->vel_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Int16>(
    vel_topic, qos.get_subscription_qos(vel_topic, rclcpp::QoS(1)),
    std::bind(&AutonomosPlugin::OnRosMsgVel, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->ros_node_->get_logger(),
    "AutonomosPlugin listo: suscrito a [%s] y [%s]",
    steering_topic.c_str(), vel_topic.c_str());

  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
    std::bind(&AutonomosPlugin::OnUpdate, this, std::placeholders::_1));
}

void AutonomosPlugin::OnRosMsgSteering(const std_msgs::msg::Int16::SharedPtr _msg)
{
  // Aproximación lineal a los ángulos reales del AutoNOMOS (idéntica a ROS 1):
  // comando en [0, 180] -> ángulo de la dirección en grados.
  this->position_ = 0.252556f * (_msg->data - 90) + 0.572957f;
}

void AutonomosPlugin::OnRosMsgVel(const std_msgs::msg::Int16::SharedPtr _msg)
{
  this->vel_ = _msg->data * -15.0f / 31.0f * ADJ_FACT;
}

void AutonomosPlugin::OnUpdate(const common::UpdateInfo &)
{
  const float pos_target = this->position_;

  const common::Time curr_time = this->model_->GetWorld()->SimTime();
  const ignition::math::Angle angle_aux(this->joint_->Position(0));  // radianes
  const float pos_curr = angle_aux.Degree();

  const common::Time step_time = curr_time - this->prev_update_time_;
  this->prev_update_time_ = curr_time;

  const float vel_target = this->vel_;
  const float vel_curr_left = this->joint_left_wheel_->GetVelocity(0);
  const float vel_curr_right = this->joint_right_wheel_->GetVelocity(0);

  const double pos_err = pos_curr - pos_target;
  const double vel_err_left = vel_curr_left - vel_target;
  const double vel_err_right = vel_curr_right - vel_target;

  const double effort_cmd = this->pid_.Update(pos_err, step_time);
  const double effort_cmd_vl = this->pid_vel_left_.Update(vel_err_left, step_time);
  const double effort_cmd_vr = this->pid_vel_right_.Update(vel_err_right, step_time);

  this->joint_->SetForce(0, effort_cmd);
  this->joint_left_wheel_->SetForce(0, effort_cmd_vl);
  this->joint_right_wheel_->SetForce(0, effort_cmd_vr);
}

}  // namespace gazebo

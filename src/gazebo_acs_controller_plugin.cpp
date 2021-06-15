/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *


 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gazebo_acs_controller_plugin.h>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(ACSControllerPlugin)

/* Keep these functions in the 'detail' namespace so that they
 * can be called from unit tests. */
namespace detail {
ignition::math::Vector3d ThreeAxisRot(double r11, double r12, double r21,
                                      double r31, double r32) {
  return ignition::math::Vector3d(atan2(r31, r32), asin(r21), atan2(r11, r12));
}

ignition::math::Vector3d QtoZXY(const ignition::math::Quaterniond &_q) {
  // taken from
  // http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
  // case zxy:
  ignition::math::Vector3d result = detail::ThreeAxisRot(
      -2 * (_q.X() * _q.Y() - _q.W() * _q.Z()),
      _q.W() * _q.W() - _q.X() * _q.X() + _q.Y() * _q.Y() - _q.Z() * _q.Z(),
      2 * (_q.Y() * _q.Z() + _q.W() * _q.X()),
      -2 * (_q.X() * _q.Z() - _q.W() * _q.Y()),
      _q.W() * _q.W() - _q.X() * _q.X() - _q.Y() * _q.Y() + _q.Z() * _q.Z());
  return result;
}
} // namespace detail

ACSControllerPlugin::ACSControllerPlugin() {}

void ACSControllerPlugin::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf) {
  this->_model = _model;
  this->_sdf = _sdf;

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_acs_controller] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // get imu sensors
  std::string imuSensorName = "camera_imu";
  if (_sdf->HasElement("acs_imu")) {
    // Add names to map
    imuSensorName = _sdf->Get<std::string>("acs_imu");
  }
  imuSensor = std::static_pointer_cast<sensors::ImuSensor>(
      sensors::SensorManager::Instance()->GetSensor(imuSensorName));

  if (!imuSensor) {
    gzerr << "ACSControllerPlugin::Load ERROR! Can't get imu sensor '"
          << imuSensorName << "' " << endl;
  }

  // topics
  new_xy_pub_topic_ = "~/" + _model->GetName() + "/new_xy_status";
  roll_pitch_pub_topic_ = "~/" + _model->GetName() + "/roll_pitch_status";
  roll_pitch_setpoint_pub_topic_ = "~/" + _model->GetName() + "/roll_pitch_setpoint";
  thruster_pub_topic_ = "~/" + _model->GetName() + "/thruster_status";
  crater_sub_topic_ = "~/" + _model->GetName() + "/crater_catalog";

  // publishers
  new_xy_status_pub_ = node_handle_->Advertise<sensor_msgs::msgs::NewXYStatus>(new_xy_pub_topic_, 10);
  roll_pitch_status_pub_ = node_handle_->Advertise<sensor_msgs::msgs::RollPitchStatus>(roll_pitch_pub_topic_, 10);
  roll_pitch_setpoint_pub_ = node_handle_->Advertise<sensor_msgs::msgs::RollPitchSetpoint>(roll_pitch_setpoint_pub_topic_, 10);
  thruster_status_pub_ = node_handle_->Advertise<sensor_msgs::msgs::ThrusterStatus>(thruster_pub_topic_, 10);

  // subscriber
  crater_sub_ = node_handle_->Subscribe<sensor_msgs::msgs::Crater>(crater_sub_topic_, &ACSControllerPlugin::craterCallback, this);
}

void ACSControllerPlugin::craterCallback(CraterMsgPtr &msg)
{
    closestCrater_.name = msg->name();
    closestCrater_.relPos.X(msg->xrel());
    closestCrater_.relPos.Y(msg->yrel());
    closestCrater_.relPos.Z(msg->zrel());

    std::cout << "Received crater update: " << closestCrater_.name << "; " << closestCrater_.relPos << std::endl;
}

void ACSControllerPlugin::Init() {
  // plugin update
  connections.push_back(event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ACSControllerPlugin::OnUpdate, this)));

  std::cout << "ACSControllerPlugin::Init" << std::endl;
}

void ACSControllerPlugin::OnUpdate() {
  handle_control();
  sendACSStatus();
}

void ACSControllerPlugin::handle_control() {
  /// currentAngleYPRVariable is defined in roll-pitch-yaw-fixed-axis
  /// and gimbal is constructed using yaw-roll-pitch-variable-axis
  ignition::math::Vector3d currentAngleYPRVariable(
      imuSensor->Orientation().Euler());

  ignition::math::Vector3d currentAnglePRYVariable(
      detail::QtoZXY(ignition::math::Quaterniond(currentAngleYPRVariable)));

  double coord_theta = 0.7854;
  double oldXComp = sin(currentAnglePRYVariable.Y());
  double oldYComp = sin(currentAnglePRYVariable.X());
  double newXComp = oldXComp * cos(coord_theta) + oldYComp * -sin(coord_theta);

  double newYComp = oldXComp * sin(coord_theta) + oldYComp * cos(coord_theta);
  double newVecLength = sqrt(pow(newXComp, 2) + pow(cos(currentAnglePRYVariable.Y()), 2));

  _newX = asin(newYComp / newVecLength);
  _newY = asin(newXComp / newVecLength);

  // Get thrusters from model
  static actuator acs_po("lander::thruster_1");
  static actuator acs_sb("lander::thruster_3");
  static actuator acs_st("lander::thruster_2");
  static actuator acs_bo("lander::thruster_4");

  acs_po.link = _model->GetChildLink(acs_po.path);
  acs_sb.link = _model->GetChildLink(acs_sb.path);
  acs_st.link = _model->GetChildLink(acs_st.path);
  acs_bo.link = _model->GetChildLink(acs_bo.path);

  const ignition::math::Vector3<double> &force = {0, 0, 0};

  // Apply force to thrusters
  acs_po.link->AddLinkForce(force);
  acs_sb.link->AddLinkForce(force);
  acs_st.link->AddLinkForce(force);
  acs_bo.link->AddLinkForce(force);
}

void ACSControllerPlugin::sendACSStatus() {
  // fill NewXYStatus msg
  sensor_msgs::msgs::NewXYStatus new_xy_status_msg;

  new_xy_status_msg.set_new_x(_newX);
  new_xy_status_msg.set_new_y(_newY);

  // fill RollPitchStatus msg
  sensor_msgs::msgs::RollPitchStatus roll_pitch_status_msg;

  roll_pitch_status_msg.set_roll_target(_rollTarget);
  roll_pitch_status_msg.set_pitch_target(_pitchTarget);

  // fill RollPitchSetpoint msg
  sensor_msgs::msgs::RollPitchSetpoint roll_pitch_setpoint_msg;

  roll_pitch_setpoint_msg.set_roll_setpoint(_rollSetpoint);
  roll_pitch_setpoint_msg.set_pitch_setpoint(_pitchSetpoint);

  // fill ThrusterStatus msg
  sensor_msgs::msgs::ThrusterStatus thruster_status_msg;

  thruster_status_msg.set_thruster_1(_thrusterStatus[0]);
  thruster_status_msg.set_thruster_2(_thrusterStatus[1]);
  thruster_status_msg.set_thruster_3(_thrusterStatus[2]);
  thruster_status_msg.set_thruster_4(_thrusterStatus[3]);

  // Publish status msgs to gazebo_custom_mavlink_interface
  new_xy_status_pub_->Publish(new_xy_status_msg);
  roll_pitch_status_pub_->Publish(roll_pitch_status_msg);
  roll_pitch_setpoint_pub_->Publish(roll_pitch_setpoint_msg);
  thruster_status_pub_->Publish(thruster_status_msg);
}
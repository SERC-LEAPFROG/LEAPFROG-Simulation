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

#include <gazebo_tvc_controller_plugin.h>

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(TVCControllerPlugin)

TVCControllerPlugin::TVCControllerPlugin() {}

void TVCControllerPlugin::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf) {
  model_ = _model;
  sdf_ = _sdf;
  world_ = model_->GetWorld();

  // default params
  namespace_.clear();

  if (_sdf->HasElement("robotNamespace")) {
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_tvc_controller] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  // TVC pub/sub topics
  tvc_status_pub_topic_ = "~/" + model_->GetName() + "/tvc_status";
  tvc_target_sub_topic_ = "~/" + model_->GetName() + "/tvc_target";

  // TVC pub/sub
  tvc_status_pub_ = node_handle_->Advertise<sensor_msgs::msgs::TVCStatus>(
    tvc_status_pub_topic_, 10);
  tvc_target_sub_ = node_handle_->Subscribe<sensor_msgs::msgs::TVCTarget>(
    tvc_target_sub_topic_, &TVCControllerPlugin::TVCTargetCallback, this);

  // Get linear actuator joints (from lander.sdf)
  actuator_1_joint_name = "actuator_1_outer__actuator_1_inner__prismatic";
  actuator_2_joint_name = "actuator_2_outer__actuator_2_inner__prismatic";

  actuator_1_joint = model_->GetJoint(actuator_1_joint_name);
  actuator_2_joint = model_->GetJoint(actuator_2_joint_name);
}

void TVCControllerPlugin::Init() {
  // plugin update
  connections.push_back(event::Events::ConnectWorldUpdateBegin(
      boost::bind(&TVCControllerPlugin::OnUpdate, this)));

  std::cout << "TVCControllerPlugin::Init" << std::endl;
}

void TVCControllerPlugin::OnUpdate() {
  handle_control();
  sendTVCStatus();
}

void TVCControllerPlugin::handle_control() {
  // Get current linear actuator joint positions
  _actuator_current_1 = actuator_1_joint->Position(0);
  _actuator_current_2 = actuator_2_joint->Position(0);

  // Get actual forces for linear actuators
  _actuator_status_1 = -1;
  _actuator_status_2 = -1;

  // Save velocity of linear actuators
  _actuator_velocity_1 = actuator_1_joint->GetVelocity(0);
  _actuator_velocity_2 = actuator_2_joint->GetVelocity(0);

  // Apply forces to linear actuators
  actuator_1_joint->SetForce(0, _actuator_status_1);
  actuator_2_joint->SetForce(0, _actuator_status_2);
}

void TVCControllerPlugin::sendTVCStatus() {
  sensor_msgs::msgs::TVCStatus tvc_status_msg;

  tvc_status_msg.set_actuator_status_1(_actuator_status_1);
  tvc_status_msg.set_actuator_target_1(_actuator_target_1);
  tvc_status_msg.set_actuator_current_1(_actuator_current_1);
  tvc_status_msg.set_actuator_velocity_1(_actuator_velocity_1);
  tvc_status_msg.set_actuator_status_2(_actuator_status_2);
  tvc_status_msg.set_actuator_target_2(_actuator_target_2);
  tvc_status_msg.set_actuator_current_2(_actuator_current_2);
  tvc_status_msg.set_actuator_velocity_2(_actuator_velocity_2);

  tvc_status_pub_->Publish(tvc_status_msg);
}

void TVCControllerPlugin::TVCTargetCallback(TVCTargetPtr &msg) {
  _actuator_target_1 = msg->actuator_1_target();
  _actuator_target_2 = msg->actuator_2_target();
}
/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_TVC_CONTROLLER_PLUGIN_HH_
#define _GAZEBO_TVC_CONTROLLER_PLUGIN_HH_

#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <ignition/math.hh>

#include "Groundtruth.pb.h"
#include <iostream>
#include <time.h>

#include "TVCStatus.pb.h"
#include "TVCTarget.pb.h"

namespace gazebo {

typedef const boost::shared_ptr<const sensor_msgs::msgs::TVCTarget> TVCTargetPtr;

class GAZEBO_VISIBLE TVCControllerPlugin : public ModelPlugin {

public:
  TVCControllerPlugin();

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();

private:
  void OnUpdate();
  void handle_control();
  void sendTVCStatus();
  void TVCTargetCallback(TVCTargetPtr &msg);

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  sdf::ElementPtr sdf_;

  std::vector<event::ConnectionPtr> connections;
  transport::NodePtr node_handle_;
  std::string namespace_;

  std::string actuator_1_joint_name;
  std::string actuator_2_joint_name;

  physics::JointPtr actuator_1_joint;
  physics::JointPtr actuator_2_joint;

  // Topics for TVC messaging across plugins
  std::string tvc_status_pub_topic_;
  std::string tvc_target_sub_topic_;

  // Pub and Sub for messaging across plugins
  transport::PublisherPtr tvc_status_pub_;
  transport::SubscriberPtr tvc_target_sub_;

  // linear actuator parameters
  double _actuator_status_1;
  double _actuator_target_1;
  double _actuator_current_1;
  double _actuator_velocity_1;

  double _actuator_status_2;
  double _actuator_target_2;
  double _actuator_current_2;
  double _actuator_velocity_2;
};
#endif
}
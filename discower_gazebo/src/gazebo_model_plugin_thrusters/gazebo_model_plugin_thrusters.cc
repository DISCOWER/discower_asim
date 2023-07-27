/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

// Gazebo includes
#include <astrobee_gazebo/astrobee_gazebo.h>
#include <ff_util/ff_timer.h>
#include <geometry_msgs/msg/wrench.hpp>

namespace geometry_msgs {
typedef msg::Wrench Wrench;
}  // geometry_msgs

// STL includes
#include <string>
#include <map>

namespace gazebo {

FF_DEFINE_LOGGER("gazebo_model_plugin_thrusters");

// This class is a plugin that calls the GNC autocode to predict
// the forced to be applied to the rigid body
class GazeboModelPluginThrusters : public FreeFlyerModelPlugin {
 public:
  // static constexpr size_t NUMBER_OF_PMCS      = 2;
  // static constexpr size_t NUMBER_OF_NOZZLES   = 6;
  // static constexpr double RADS_PER_SEC_TO_RPM = 9.549296585514;

  // Constructor
  GazeboModelPluginThrusters() : FreeFlyerModelPlugin("thrusters_actuator", "", true),
    bypass_thrusters_model_(true) {

  }

  // Destructor
  ~GazeboModelPluginThrusters() {
    update_.reset();
  }

 protected:
  // Called when the plugin is loaded into the simulator
  void LoadCallback(NodeHandle &nh,
    physics::ModelPtr model, sdf::ElementPtr sdf) {

    // If we specify a frame name different to our sensor tag name
    if (sdf->HasElement("bypass_thrusters_model")){
      bypass_thrusters_model_ = sdf->Get<bool>("bypass_thrusters_model");
    }else{
      bypass_thrusters_model_ = true;
      FF_WARN("THRUSTERS Plugin: Only possible to apply CoM forces at the moment.");
    }

    // Set timeout
    if (sdf->HasElement("rate")){
      control_rate_hz_ = sdf->Get<bool>("rate");
    }else{
      control_rate_hz_ = 10.0;
    }

    // Now register to be called back every time the Thrusters receive a new wrench
    if (bypass_thrusters_model_)
      sub_wrench_ = FF_CREATE_SUBSCRIBER(nh, geometry_msgs::Wrench,
        TOPIC_GNC_CTL_COMMAND, 5,
        std::bind(&GazeboModelPluginThrusters::WrenchCallback, this, std::placeholders::_1));
  
    // Create a watchdog timer to ensure the PMC commands are set
    timer_command_.createTimer(1.0/control_rate_hz_,
      std::bind(&GazeboModelPluginThrusters::CommandTimerCallback, this), nh_, false, true);

    // Called before each iteration of simulated world update
    update_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboModelPluginThrusters::WorldUpdateCallback, this));
  }

  // Called on simulation reset
  void Reset() {}

  // This is called whenever the controller has new force/torque to apply
  void WrenchCallback(geometry_msgs::Wrench const& msg) {
      wrench_ = msg;
  }

  // This is called at 10Hz to resemble the real system - the frequency can be changed from the parameters
  void CommandTimerCallback() {
    force_ = ignition::math::Vector3d(wrench_.force.x, wrench_.force.y, wrench_.force.z);
    torque_ = ignition::math::Vector3d(wrench_.torque.x, wrench_.torque.y, wrench_.torque.z);
  }

  // Called on each sensor update event
  void WorldUpdateCallback() {
    if (bypass_thrusters_model_) {
      GetLink()->AddRelativeForce(ignition::math::Vector3d(
        wrench_.force.x, wrench_.force.y, wrench_.force.z));
      GetLink()->AddRelativeTorque(ignition::math::Vector3d(
        wrench_.torque.x, wrench_.torque.y, wrench_.torque.z));
    } else {
      FF_WARN("THRUSTER Plugin: Bypass Thrusters is set to false!");
    }
  }

 private:
  rclcpp::Subscription<geometry_msgs::Wrench>::SharedPtr sub_wrench_;    // Body-Thruster command subscriber
  ff_util::FreeFlyerTimer timer_command_;                                // Timers
  event::ConnectionPtr update_;                                          // Update event from gazeo
  ignition::math::Vector3d force_;                                       // Current body-frame force
  ignition::math::Vector3d torque_;                                      // Current body-frame torque

  geometry_msgs::Wrench wrench_;                                         // Used when bypassing PMC

  bool bypass_thrusters_model_;                                          // Bypass the blower model

  double control_rate_hz_;                                               // Control rate
  double max_timeout_;                                                   // Maximum timeout allowed for PMC
  std::string frame_id_;                                                 // Frame
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboModelPluginThrusters)

}   // namespace gazebo

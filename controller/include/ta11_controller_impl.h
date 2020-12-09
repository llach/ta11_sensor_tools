/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Luca Lach
*/

#ifndef TA11_CONTROLLER_TA11_CONTROLLER_IMPL_H
#define TA11_CONTROLLER_TA11_CONTROLLER_IMPL_H

#include "ta11_controller.h"

namespace ta11_controller {
template <class TactileSensors>
inline bool TA11TrajectoryController<TactileSensors>::init(hardware_interface::PositionJointInterface* hw,
                                                           ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    name_ = "TA11C";
    ROS_INFO_NAMED(name_, "Initializing TA11TrajectoryController.");
    bool ret = 0;
    try {
        ret = force_controller::ForceTrajectoryController<TactileSensors>::init(hw, root_nh, controller_nh);
    } catch(std::runtime_error& e) {
        ROS_ERROR_STREAM_NAMED(name_, "Could not init FTC: " << e.what());
    }
    // print verbose errors
    this->verbose_ = true;

  max_forces_= std::make_shared<std::vector<float>>(num_sensors_, 1.0);
  k_= std::make_shared<std::vector<float>>(num_sensors_, 850.0);

    debug_pub_ = root_nh.advertise<tiago_tactile_msgs::TA11Debug>("/ta11_debug", 1);

    ROS_INFO_NAMED(name_, "regitering dynamic reconfigure server ...");
    f_ = boost::bind(&TA11TrajectoryController<TactileSensors>::dr_callback, this, _1, _2);
    server_.setCallback(f_);

    NOISE_THRESH = 0.25;

   return ret;
}

template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::update_sensors() {
    sensors_->update();
}

template <class TactileSensors>
inline bool TA11TrajectoryController<TactileSensors>::check_controller_transition() {
    for (auto& ss : sensor_states_){
        if (ss < GOT_CONTACT || ss >= GOAL)
            return false;
    }
    return true;
}

template <class TactileSensors>
inline bool TA11TrajectoryController<TactileSensors>::check_finished() {
    for (int l = 0; l < num_sensors_; l++){
      if(sensor_states_[l] == GOAL){
        ROS_INFO_THROTTLE_NAMED(1, name_, "Sensor %d is already in goal", l);
        continue;
      }
      if (std::abs((*forces_)[l]) < (*max_forces_)[l])
          return false;
    }
    return true;
}

template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::publish_debug_info() {
    tiago_tactile_msgs::TA11Debug dbg_msg = tiago_tactile_msgs::TA11Debug();
    dbg_msg.header.stamp = ros::Time::now();

    dbg_msg.k = {(*k_)[0], (*k_)[1]};
    dbg_msg.f = {(*forces_)[0], (*forces_)[1]};
    dbg_msg.p = {current_state_.position[0], current_state_.position[1]};
    dbg_msg.delta_F = {(*delta_F_)[0], (*delta_F_)[1]};
    dbg_msg.delta_p = {(*delta_p_)[0], (*delta_p_)[1]};
    dbg_msg.delta_p_T = {(*delta_p_T_)[0], (*delta_p_T_)[1]};
    dbg_msg.p_T = {(*pos_T_)[0], (*pos_T_)[1]};

    dbg_msg.delta_p_force = {(*delta_p_force_)[0], (*delta_p_force_)[1]};
    dbg_msg.delta_p_vel = {(*delta_p_vel_)[0], (*delta_p_vel_)[1]};

    dbg_msg.noise_treshold = {-NOISE_THRESH, NOISE_THRESH};
    dbg_msg.max_forces = {-(*max_forces_)[0], (*max_forces_)[1]};

    dbg_msg.error_integral = {(*error_integral_)[0], (*error_integral_)[1]};
    dbg_msg.f_error_integral = f_error_integral_;

    dbg_msg.c_state = c_state_;

    dbg_msg.des_vel = {(*des_vel_)[0], (*des_vel_)[1]};

    dbg_msg.vel_limit = vel_limit_;

    for (auto& j : sensor_states_)
      dbg_msg.joint_states.push_back(j);

    debug_pub_.publish(dbg_msg);
}

template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::dr_callback(ta11_controller::TA11ControllerDRConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request:\n\tmax_forces: %f\n\tlambda: %f\n\tk: %d\n\tK_i: %f",
          config.max_force_thresh, config.lambda, config.init_k,
          config.k_i);
  max_forces_= std::make_shared<std::vector<float>>(num_sensors_, config.max_force_thresh);
  init_k_= config.init_k;
  lambda_ = config.lambda;
  goal_maintain_ = config.goal_maintain;
  K_i_ = config.k_i;
  K_p_ = config.k_p;
}
}

#endif  // TA11_CONTROLLER_TA11_CONTROLLER_IMPL_H

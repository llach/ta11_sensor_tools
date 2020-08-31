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

    max_forces_= std::make_shared<std::vector<float>>(num_sensors_, 2.5);

    debug_pub_ = root_nh.advertise<ta11_controller::TA11Debug>("/ta11_debug", 1);

    NOISE_THRESH = 0.05;

   return ret;
}

template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::update_sensors() {
    sensors_->update();
}

template <class TactileSensors>
inline bool TA11TrajectoryController<TactileSensors>::check_controller_transition() {
    for (auto& ss : sensor_states_){
        if (ss < GOT_CONTACT)
            return false;
    }
    return true;
}

template <class TactileSensors>
inline bool TA11TrajectoryController<TactileSensors>::check_finished() {
    for (int l = 0; l < num_sensors_; l++){
        if ((*forces_)[l] < (*max_forces_)[l])
            return false;
    }
    return true;
}

template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::publish_debug_info() {
    if (c_state_ > TRANSITION) {
//        ROS_DEBUG_NAMED(name_ + "hook", "\nk:  [%.6f, %.6f]\ndF: [%.6f, %.6f]\ndp: [%.6f, %.6f]\ncp: [%.6f, %.6f]\nnp: [%.6f, %.6f]",
//                        (*k_)[0], (*k_)[1],
//                        (*delta_F_)[0], (*delta_F_)[1],
//                        (*delta_p_)[0], (*delta_p_)[1],
//                        current_state_.position[0], current_state_.position[1],
//                        desired_state_.position[0], desired_state_.position[1]);
        ta11_controller::TA11Debug dbg_msg = ta11_controller::TA11Debug();
        dbg_msg.header.stamp = ros::Time::now();

        dbg_msg.k = {(*k_)[0], (*k_)[1]};
        dbg_msg.delta_F = {(*delta_F_)[0], (*delta_F_)[1]};
        dbg_msg.delta_p = {(*delta_p_)[0], (*delta_p_)[1]};
        dbg_msg.cur_p = {current_state_.position[0], current_state_.position[1]};
        dbg_msg.new_p = {desired_state_.position[0], desired_state_.position[1]};
        dbg_msg.tra_p = {(*pos_T_)[0], (*pos_T_)[1]};
        debug_pub_.publish(dbg_msg);
    }
}

}

#endif  // TA11_CONTROLLER_TA11_CONTROLLER_IMPL_H

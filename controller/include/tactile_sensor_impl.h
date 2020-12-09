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

#ifndef TA11_CONTROLLER_TACTILE_SENSOR_IMPL_H

#include <tactile_sensor.h>

namespace ta11_controller {
TactileSensorBase::TactileSensorBase(ros::NodeHandle& nh, std::vector<std::shared_ptr<double>> forces, bool simulation) : nh_(nh), forces_(forces), sim(simulation){
  for (int i = 0; i < forces_.size(); i++)
    tmp_forces_.push_back(0.0);
}

TactileSensorSub::TactileSensorSub(ros::NodeHandle& nh, std::vector<std::shared_ptr<double>> forces) : TactileSensorBase(nh, forces, true) {
    sub_ = nh.subscribe("/ta11", 0, &TactileSensorSub::sensor_cb_, this);
    ROS_INFO_STREAM("Registered subscriber for \"/ta11\"");
}

void TactileSensorSub::sensor_cb_(const tiago_tactile_msgs::TA11 ts) {
    for (int i = 0; i < tmp_forces_.size(); i++){
        tmp_forces_[i] = ts.sensor_values[i];
    }
}

void TactileSensorSub::update() {
    for (int i = 0; i < tmp_forces_.size(); i++){
        *forces_[i] = tmp_forces_[i];
    }
}

TactileSensorReal::TactileSensorReal(ros::NodeHandle& nh, std::vector<std::shared_ptr<double>> forces) : TactileSensorBase(nh, forces, false) {}
}

#endif  // TA11_CONTROLLER_TACTILE_SENSOR_IMPL_H

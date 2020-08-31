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

#ifndef TA11_CONTROLLER_TACTILE_SENSOR_H
#define TA11_CONTROLLER_TACTILE_SENSOR_H

#include <ta11_controller.h>
#include <tiago_tactile_msgs/TA11.h>

namespace ta11_controller {
class TactileSensorBase {
public:
    TactileSensorBase(ros::NodeHandle& root_nh, std::shared_ptr<std::vector<float>> forces, bool simulation);
    virtual void update() {};

    bool sim = false;
protected:
    ros::NodeHandle& nh_;
    std::shared_ptr<std::vector<float>> tmp_forces_;
    std::shared_ptr<std::vector<float>> forces_;
};

// listens to topic for simulation use
class TactileSensorSim : public TactileSensorBase
{
public:
    TactileSensorSim(ros::NodeHandle& root_nh, std::shared_ptr<std::vector<float>> forces);
    void update() override;
private:
    ros::Subscriber sub_;
    void sensor_cb_(const tiago_tactile_msgs::TA11 tactile_state);
};

// reads from real sensors
class TactileSensorReal : public TactileSensorBase
{
public:
	TactileSensorReal(ros::NodeHandle& root_nh, std::shared_ptr<std::vector<float>> forces);
};
}

#include <tactile_sensor_impl.h>

#endif  // TA11_CONTROLLER_TACTILE_SENSOR_H

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

#ifndef TA11_CONTROLLER_TA11_CONTROLLER_H
#define TA11_CONTROLLER_TA11_CONTROLLER_H

#include <mutex>
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"

#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <joint_trajectory_controller/joint_trajectory_segment.h>
#include <trajectory_interface/quintic_spline_segment.h>

#include <dynamic_reconfigure/server.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <tiago_tactile_msgs/TA11Debug.h>
#include <ta11_controller/TA11ControllerDRConfig.h>

#include <tiago_tactile_msgs/GetForceThreshold.h>

#include "force_controller_core/force_controller.h"

namespace ta11_controller {

template <class TactileSensors>
class TA11TrajectoryController
: public joint_trajectory_controller::JointTrajectoryController<trajectory_interface::QuinticSplineSegment<double>,
        hardware_interface::PositionJointInterface>
{
    void goalCB(GoalHandle gh) override;
    void cancelCB(GoalHandle gh) override;
    void update(const ros::Time& time, const ros::Duration& period) override;

protected:
    bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& root_nh,
              ros::NodeHandle& controller_nh) override;

    double object_pos();
    bool check_finished();
    void reset_parameters();
    void publish_debug_info();
    bool check_controller_transition();

    void setHoldForce(const ros::Time& time);

    bool kill_goal();
    bool kill_goal_srv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    void grav_cb(const std_msgs::Float64ConstPtr msg);

    ros::Subscriber grav_sub_;
    ros::Publisher debug_pub_;
    dynamic_reconfigure::Server<ta11_controller::TA11ControllerDRConfig> server_;
    dynamic_reconfigure::Server<ta11_controller::TA11ControllerDRConfig>::CallbackType f_;

    void dr_callback(ta11_controller::TA11ControllerDRConfig &config, uint32_t level);

    ros::ServiceServer kill_service_;

    int num_sensors_ = 0;

    bool realtime_busy_ = false;

    bool goal_maintain_ = false;

    ros::Time curr_time_;

    RealtimeGoalHandlePtr fgh_;

    typedef std::shared_ptr<TactileSensors> TactileSensorsPtr;
    TactileSensorsPtr sensors_;

    std::vector<double> last_q_;

    double F_O_ = 0.0;
    double O_T_ = 0.0;
    double O_t_ = 0.0;
    double delta_O_ = 0.0;
    double last_O_ = 0.0;
    double last_F_O_ = 0.0;

    // pointer to force vector. Written to by TactileSensors and read by ForceController
    std::vector<std::shared_ptr<double>> forces_;

    // list of joint-level force controllers
    std::vector<fcc::JointForceController> jfc_;

    // Force Controller parameters
    double noise_thresh = 0.04;

    double target_force = 1.2;

    double init_k_ = 875;

    double K_p_ = 1;
    double K_i_ = 0.002;

    double dc_factor_ = 0.0;
    double dc_thresh_ = noise_thresh*dc_factor_;

    bool opening_ = false;
    bool drift_corr_ = true;
    bool cond_drift_corr_ = true;
    bool gravity_corr_ = true;

    std::mutex grav_mtx_;
    float cosGrav_ = 0.0;
    float gravDrift_ = 0.0;
    float mass_ = 48; // in gramms

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


    // indicates whether we are executing the trajectory, doing force control or are in transition between the two
    fcc::CONTROLLER_STATE state_;
};

}

#include <ta11_controller_impl.h>

#endif  // TA11_CONTROLLER_TA11_CONTROLLER_IMPL_H

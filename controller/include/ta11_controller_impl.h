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
    ROS_INFO_NAMED(name_, "Initializing TA11TrajectoryController ...");
    bool ret = 0;
    try {
        ret = JointTrajectoryController::init(hw, root_nh, controller_nh);
    } catch(std::runtime_error& e) {
        ROS_ERROR_STREAM_NAMED(name_, "Could not init JTC: " << e.what());
    }
    // print verbose errors
    this->verbose_ = true;

    // get numbers of joints
    num_sensors_ = joints_.size();

    ROS_INFO_NAMED(name_, "reloading action server ...");
    action_server_.reset(new ActionServer(controller_nh_, "follow_joint_trajectory",
                                          boost::bind(&TA11TrajectoryController::goalCB,   this, _1),
                                          boost::bind(&TA11TrajectoryController::cancelCB, this, _1),
                                          false));
    action_server_->start();

    for (int i=0; i<joint_names_.size(); i++){
      ROS_INFO_STREAM_NAMED(name_, "joint_name[" << i << "] = " << joint_names_[i]);

      std::shared_ptr<float> fp = std::make_shared<float>(0.0);
      fcc::JointForceController jfc(joint_names_[i],
                                    fp,
                                    0.25, // force threshold
                                    2.2, // target force
                                    875, // initial k
                                    0.01, // minimum velocity
                                    5, // K_p
                                    0.001, // K_i
                                    1.1, // maximum error integral value
                                    200 // force error integral windows length
                                    );

      jfc_.push_back(jfc);
      forces_.push_back(fp);
    }

    sensors_ = std::make_shared<TactileSensors>(root_nh, forces_);

    debug_pub_ = root_nh.advertise<tiago_tactile_msgs::TA11Debug>("/ta11_debug", 1);

    ROS_INFO_NAMED(name_, "registering dynamic reconfigure server ...");
    f_ = boost::bind(&TA11TrajectoryController<TactileSensors>::dr_callback, this, _1, _2);
    server_.setCallback(f_);

    ROS_INFO_NAMED(name_, "TA11 controller setup done!");

  return ret;
}

template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::update(const ros::Time& time, const ros::Duration& period) {

  realtime_busy_ = true;
  // Get currently followed trajectory
  TrajectoryPtr curr_traj_ptr;
  curr_trajectory_box_.get(curr_traj_ptr);
  Trajectory& curr_traj = *curr_traj_ptr;

  // Update time data
  TimeData time_data;
  time_data.time = time;  // Cache current time
  time_data.period = period;  // Cache current control period
  time_data.uptime = time_data_.readFromRT()->uptime + period;  // Update controller uptime
  time_data_.writeFromNonRT(time_data);  // TODO: Grrr, we need a lock-free data structure here!

  curr_time_ = time;

  // NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
  // trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
  // The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
  // control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
  // If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time we
  // fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts in
  // the
  // next control cycle, leaving the current cycle without a valid trajectory.

  // read new force values from sensors
  sensors_->update();

  /*
   * Update forces and sensor states.
   */
  RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
  if (current_active_goal) {

  // update joint state changes and report them
  for (auto& fc : jfc_) {
    fc.update_joint_states(period.toSec());

    ROS_DEBUG_NAMED(name_ + ".sensorState",
                    "%s has state %s with force %.4f", fc.joint_name_.c_str(), fcc::STATE_STRING[fc.sensor_state_].c_str(), *fc.force_);

    if (fc.sensor_state_ != fc.last_sensor_state_) {
      ROS_INFO_NAMED(name_ + ".sensorStateChange",
                     "%s's state changed from %s to %s. f_t-1 %.4f, f_t %.4f", fc.joint_name_.c_str(),
                     fcc::STATE_STRING[fc.last_sensor_state_].c_str(), fcc::STATE_STRING[fc.sensor_state_].c_str(),
                     fc.last_force_, *fc.force_);
    }
  }

  // update controller state
  if (state_ == fcc::CONTROLLER_STATE::TRANSITION) {
    // if the transition was detected last time, we enter force_control from here onwards
    state_ = fcc::CONTROLLER_STATE::FORCE_CTRL;
  } else if (state_ == fcc::CONTROLLER_STATE::TRAJECTORY_EXEC) {
    if (check_controller_transition()) {
      ROS_INFO_NAMED(name_, "CONTROLLER TRANSITION");
      state_ = fcc::CONTROLLER_STATE::TRANSITION;

      // Store reference data.
      for (auto& fc : jfc_)
        fc.on_transition();
    }
    // if we were to revert back to trajectory mode, here is the place to do so
  }


//  vel_limit_ = 0;

    // Update desired state and state error
    for (unsigned int i = 0; i < joints_.size(); ++i) {

      current_state_.position[i] = joints_[i].getPosition();
      current_state_.velocity[i] = joints_[i].getVelocity();
      // There's no acceleration data available in a joint handle

//      float state_err = current_state_.position[i] - desired_state_.position[i];

      typename TrajectoryPerJoint::const_iterator segment_it;
      if (false) {
//    if (c_state_ > TRANSITION) {
//      ROS_INFO_STREAM_NAMED(name_, "IN FC " << i);
//
//      // prevent integral from exploding
//      if ((*error_integral_)[i] < max_error_int_){
//        (*error_integral_)[i] += state_err;
//      }
//
//      (*delta_p_T_)[i] = ((*pos_T_)[i] - current_state_.position[i]);
//
//      // estimate k
////                double k_bar_t = (*forces_)[i] / (*delta_p_T_)[i];
////                k_bar_t = std::max(k_bar_t, 10000.0);
//
////                if (!std::isinf(k_bar_t)){
////                    (*k_)[i] = lambda_*k_bar_t + (1-lambda_)*(*k_)[i];
////                }
//
//      // calculate new desired position
//      double f_des = (*max_forces_)[i] - std::abs((*forces_)[i]);
//      double delta_p_force = (f_des / (*k_)[i]);
//      (*delta_F_)[i] = f_des;
//
//      if (f_error_queue_.size() > f_error_window_)
//        f_error_queue_.pop_back();
//
//      f_error_queue_.push_front(delta_p_force);
//      f_error_integral_ = std::accumulate(f_error_queue_.begin(), f_error_queue_.end(), 0.0);
//
//      // --> use PI[D] controller here
//      double delta_p_max = K_p_ * (min_vel_ * period.toSec());
//      delta_p_max += K_i_ * (*error_integral_)[i];
//
//      // enforce velocity limits
//      if (std::abs(delta_p_force) > std::abs(delta_p_max)){
//        vel_limit_ = 1;
//        (*delta_p_)[i] = delta_p_max;
//      } else {
//        (*delta_p_)[i] = delta_p_force;
//      }
//
//      // calculate new position and velocity
//      desired_joint_state_.position[0] = current_state_.position[i] - (*delta_p_)[i];
//      desired_joint_state_.velocity[0] = (desired_joint_state_.position[0] - (*last_des_p_)[i]) / period.toSec();
//
//      // store debug info
//      (*delta_p_vel_)[i] = std::abs(delta_p_max);
//      (*delta_p_force_)[i] = std::abs(delta_p_force);

      } else {
        segment_it =
                sample(curr_traj[i], time_data.uptime.toSec(), desired_joint_state_); // todo use joint_times_
        if (curr_traj[i].end() == segment_it) {
          // Non-realtime safe, but should never happen under normal operation
          ROS_ERROR_NAMED(
                  name_,
                  "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
          cancelCB(current_active_goal->gh_);
          return;
        }
      }

      desired_state_.position[i] = desired_joint_state_.position[0];
      desired_state_.velocity[i] = desired_joint_state_.velocity[0];
      desired_state_.acceleration[i] = desired_joint_state_.acceleration[0];


      state_joint_error_.position[0] =
              angles::shortest_angular_distance(current_state_.position[i], desired_joint_state_.position[0]);
      state_joint_error_.velocity[0] = desired_joint_state_.velocity[0] - current_state_.velocity[i];
      state_joint_error_.acceleration[0] = 0.0;

      state_error_.position[i] =
              angles::shortest_angular_distance(current_state_.position[i], desired_joint_state_.position[0]);
      state_error_.velocity[i] = desired_joint_state_.velocity[0] - current_state_.velocity[i];
      state_error_.acceleration[i] = 0.0;

      // Check tolerances
      if (true) { // => Trajectory Execution || TODO this might cause non terminating controller if case 2 SHOULD ONLY BE NOT IN FORCE
        const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
        if (rt_segment_goal && rt_segment_goal == rt_active_goal_) {
          // Check tolerances
          if (time_data.uptime.toSec() < segment_it->endTime()) {
            // Currently executing a segment: check path tolerances
            const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar> &joint_tolerances =
                    segment_it->getTolerances();
            if (!checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance)) {
              if (verbose_) {
                ROS_ERROR_STREAM_NAMED(name_, "Path tolerances failed for joint: " << joint_names_[i]);
                checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance, true);

              }
//            sensor_states_[i] = VIOLATED; TODO
              if (rt_segment_goal && rt_segment_goal->preallocated_result_) {
                ROS_INFO_NAMED(name_, "Trajectory execution aborted (path tolerances)");
                rt_segment_goal->preallocated_result_->error_code =
                        control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
                rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
                rt_active_goal_.reset();
                successful_joint_traj_.reset();
                reset_parameters();
              } else {
                ROS_ERROR_STREAM("rt_segment_goal->preallocated_result_ NULL Pointer");
              }
            }
          } else if (segment_it == --curr_traj[i].end()) {
            // this else if should activate once a joint trajectory reaches it's last segment. this should only happen if we have increased joint timings for joint[i]

            if (verbose_)
              ROS_INFO_STREAM_THROTTLE_NAMED(1, name_,
                                             "Finished executing last segment, checking goal tolerances");

            // Controller uptimegit st
            const ros::Time uptime = time_data_.readFromRT()->uptime;

            // Checks that we have ended inside the goal tolerances
            const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar> &tolerances = segment_it->getTolerances();
            const bool inside_goal_tolerances =
                    checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance);

            if (inside_goal_tolerances) {
              successful_joint_traj_[i] = 1;
              ROS_INFO_NAMED(name_, "Joint %d inside goal tolerances!", i);
//            sensor_states_[i] = GOAL; TODO

//            if (last_sensor_states_[i] != GOAL) { TODO
//              ROS_INFO_NAMED(name_, "Joint %d inside goal tolerances!", i);
//            }
            } else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance) {
              // Still have some time left to meet the goal state tolerances
            } else {
              if (verbose_) {
                ROS_ERROR_STREAM_NAMED(name_, "Goal tolerances failed for joint: " << joint_names_[i]);
                // Check the tolerances one more time to output the errors that occurs
                checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance, true);
              }
//            sensor_states_[i] = VIOLATED; TODO
              if (rt_segment_goal) {
                ROS_INFO_NAMED(name_, "Trajectory execution aborted (goal tolerances)");
                rt_segment_goal->preallocated_result_->error_code =
                        control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
                rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
              } else {
                ROS_ERROR_STREAM("rt_segment_goal->preallocated_result_ NULL Pointer");
              }
              rt_active_goal_.reset();
              successful_joint_traj_.reset();
            }
          }
        }
      }
    }
  }


  // If there is an active goal and all segments finished successfully then set goal as succeeded
  // current_active_goal is reused from above the state update
  if (current_active_goal && current_active_goal->preallocated_result_ &&
      successful_joint_traj_.count() == joints_.size()) {
    ROS_INFO_NAMED(name_, "Trajectory execution succeeded");
    current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
    rt_active_goal_.reset();
    successful_joint_traj_.reset();
    reset_parameters();
  } else if (current_active_goal && current_active_goal->preallocated_result_) {
//    if (check_finished() && !goal_maintain_) {
//      ROS_INFO_NAMED(name_, "Non-trajectory success");
//
//      for (unsigned int i = 0; i < joints_.size(); ++i) {
//        ROS_INFO_NAMED(name_, "Setting joint %d to %f", i, current_state_.position[i]); // TODO or hold_state()?
//        desired_state_.position[i] = current_state_.position[i];
//        desired_state_.velocity[i] = current_state_.velocity[i];
//        desired_state_.acceleration[i] = current_state_.acceleration[i];
//
//        state_joint_error_.position[0] = 0.0;
//        state_joint_error_.velocity[0] = 0.0;
//        state_joint_error_.acceleration[0] = 0.0;
//
//        state_error_.position[i] = 0.0;
//        state_error_.velocity[i] = 0.0;
//        state_error_.acceleration[i] = 0.0;
//      }
//
//      current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
//      current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
//      rt_active_goal_.reset();
//      successful_joint_traj_.reset();
//      reset_parameters();
//
//      // Hold current position
//      setHoldPosition(time_data.uptime);
//    }
  }

  // Hardware interface adapter: Generate and send commands
  hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period, desired_state_, state_error_);

  // Set action feedback
  if (rt_active_goal_ && rt_active_goal_->preallocated_feedback_) {
    rt_active_goal_->preallocated_feedback_->header.stamp = time_data_.readFromRT()->time;
    rt_active_goal_->preallocated_feedback_->desired.positions = desired_state_.position;
    rt_active_goal_->preallocated_feedback_->desired.velocities = desired_state_.velocity;
    rt_active_goal_->preallocated_feedback_->desired.accelerations = desired_state_.acceleration;
    rt_active_goal_->preallocated_feedback_->actual.positions = current_state_.position;
    rt_active_goal_->preallocated_feedback_->actual.velocities = current_state_.velocity;
    rt_active_goal_->preallocated_feedback_->error.positions = state_error_.position;
    rt_active_goal_->preallocated_feedback_->error.velocities = state_error_.velocity;
    rt_active_goal_->setFeedback(rt_active_goal_->preallocated_feedback_);

  }

//  for (int j = 0; j < des_vel_->size(); j++){
//    if (period.toSec() == 0.0){
//      (*des_vel_)[j] = 0.0;
//    } else {
//      (*des_vel_)[j] = (desired_state_.position[j] - (*last_des_p_)[j]) / period.toSec();
//    }
//  }
//
//  publish_debug_info();
//
//  /*
//   * Store data for next loop & cleanup.
//   */
//  for (int j = 0; j < forces_->size(); j++){
//    (*last_forces_)[j] = (*forces_)[j];
//    last_sensor_states_[j] = sensor_states_[j];
//    (*last_des_p_)[j] = desired_state_.position[j];
//  }

  // Publish state
  publishState(time_data.uptime);
  realtime_busy_ = false;
}

template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::cancelCB(GoalHandle gh) {
  ROS_INFO_NAMED(name_, "Canceling action goal");
  JointTrajectoryController::cancelCB(gh);
}

template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::goalCB(GoalHandle gh) {
  ROS_INFO_NAMED(name_, "Received new action goal");

  // Precondition: Running controller
  if (!this->isRunning()) {
    ROS_ERROR_NAMED(name_, "Can't accept new action goals. Controller is not running.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code =
            control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;  // TODO: Add better error status to msg?
    gh.setRejected(result);
    return;
  }

  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!allow_partial_joints_goal_) {
    if (gh.getGoal()->trajectory.joint_names.size() != joint_names_.size()) {
      ROS_ERROR_NAMED(name_, "Joints on incoming goal don't match the controller joints.");
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
      gh.setRejected(result);
      return;
    }
  }

  // Goal should specify valid controller joints (they can be ordered differently). Reject if this is not the case
  using joint_trajectory_controller::internal::mapping; // todo error_string in newer JTC::internal?
  std::vector<unsigned int> mapping_vector = mapping(gh.getGoal()->trajectory.joint_names, joint_names_);

  if (mapping_vector.empty()) {
    ROS_ERROR_NAMED(name_, "Joints on incoming goal don't match the controller joints.");
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
    gh.setRejected(result);
    return;
  }

  // Try to update new trajectory
  RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
  std::string error_string = "";  // todo upstream passed this one to updateTrajctoryCommand
  const bool update_ok = updateTrajectoryCommand(
          joint_trajectory_controller::internal::share_member(gh.getGoal(), gh.getGoal()->trajectory), rt_goal);
  rt_goal->preallocated_feedback_->joint_names = joint_names_;

  if (update_ok) {
    // Accept new goal
    preemptActiveGoal();
    gh.setAccepted();
    rt_active_goal_ = rt_goal;

    // Setup goal status checking timer
    goal_handle_timer_ =
            controller_nh_.createTimer(action_monitor_period_, &RealtimeGoalHandle::runNonRealtime, rt_goal);
    goal_handle_timer_.start();

    // reset force control parameters
    reset_parameters();

  } else {
    // Reject invalid goal
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
    result.error_string = error_string;
    gh.setRejected(result);
  }
}

template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::reset_parameters() {
  ROS_INFO_NAMED(name_, "resetting force control parameters");

  state_ = fcc::CONTROLLER_STATE::TRAJECTORY_EXEC;
  for (auto& fc : jfc_)
    fc.reset_parameters();
}

template <class TactileSensors>
inline bool TA11TrajectoryController<TactileSensors>::check_controller_transition() {
    for (auto& fc : jfc_){
        if (fc.sensor_state_ != fcc::SENSOR_STATE::GOT_CONTACT)
            return false;
    }
    return true;
}
//
//template <class TactileSensors>
//inline bool TA11TrajectoryController<TactileSensors>::check_finished() {
//    for (int l = 0; l < num_sensors_; l++){
//      if(sensor_states_[l] == GOAL){
//        ROS_INFO_THROTTLE_NAMED(1, name_, "Sensor %d is already in goal", l);
//        continue;
//      }
//      if (std::abs((*forces_)[l]) < (*max_forces_)[l])
//          return false;
//    }
//    return true;
//}



template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::publish_debug_info() {
  tiago_tactile_msgs::TA11Debug dbg_msg = tiago_tactile_msgs::TA11Debug();
//    dbg_msg.header.stamp = ros::Time::now();
//
//    dbg_msg.k = {(*k_)[0], (*k_)[1]};
//    dbg_msg.f = {(*forces_)[0], (*forces_)[1]};
//    dbg_msg.p = {current_state_.position[0], current_state_.position[1]};
//    dbg_msg.delta_F = {(*delta_F_)[0], (*delta_F_)[1]};
//    dbg_msg.delta_p = {(*delta_p_)[0], (*delta_p_)[1]};
//    dbg_msg.delta_p_T = {(*delta_p_T_)[0], (*delta_p_T_)[1]};
//    dbg_msg.p_T = {(*pos_T_)[0], (*pos_T_)[1]};
//
//    dbg_msg.delta_p_force = {(*delta_p_force_)[0], (*delta_p_force_)[1]};
//    dbg_msg.delta_p_vel = {(*delta_p_vel_)[0], (*delta_p_vel_)[1]};
//
//    dbg_msg.noise_treshold = {-NOISE_THRESH, NOISE_THRESH};
//    dbg_msg.max_forces = {-(*max_forces_)[0], (*max_forces_)[1]};
//
//    dbg_msg.error_integral = {(*error_integral_)[0], (*error_integral_)[1]};
//    dbg_msg.f_error_integral = f_error_integral_;
//
//    dbg_msg.c_state = c_state_;
//
//    dbg_msg.des_vel = {(*des_vel_)[0], (*des_vel_)[1]};
//
//    dbg_msg.vel_limit = vel_limit_;
//
//    for (auto& j : sensor_states_)
//      dbg_msg.joint_states.push_back(j);

  debug_pub_.publish(dbg_msg);
}

template <class TactileSensors>
inline void TA11TrajectoryController<TactileSensors>::dr_callback(ta11_controller::TA11ControllerDRConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request:\n\tmax_forces: %f\n\tlambda: %f\n\tk: %d\n\tK_i: %f",
           config.max_force_thresh, config.lambda, config.init_k,
           config.k_i);
//  max_forces_= std::make_shared<std::vector<float>>(num_sensors_, config.max_force_thresh);
//  init_k_= config.init_k;
//  lambda_ = config.lambda;
//  goal_maintain_ = config.goal_maintain;
//  K_i_ = config.k_i;
//  K_p_ = config.k_p;
}

}

#endif  // TA11_CONTROLLER_TA11_CONTROLLER_IMPL_H

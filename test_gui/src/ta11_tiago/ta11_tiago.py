import os
import rospy
import rosbag
import actionlib

import numpy as np

from threading import Thread
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QThread
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import QtCore

from tiago_tactile_msgs.msg import TA11
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

from controller_manager.controller_manager_interface import *
from controller_manager_msgs.srv import ListControllers

class ACThread(QThread):

    def __init__(self, ac, wid, timeout=10):
        super(QThread, self).__init__()

        self.ac = ac
        self.wid = wid
        self.timeout = timeout

    def run(self):
        rospy.loginfo("waiting for result for {}s ...".format(self.timeout))

        self.wid.btn_close.setEnabled(False)
        self.wid.btn_open.setEnabled(False)

        if self.ac.wait_for_result(rospy.Duration(secs=self.timeout)):
            res = self.ac.get_result()
            rospy.loginfo("Trajectory execution finished: {}".format(res))
            self.wid.lbl_rt_code.setText("{}".format(res.error_code))
        else:
            rospy.loginfo("AS timeout!".format(self.timeout))
            self.wid.lbl_rt_code.setText("T")

        self.wid.btn_close.setEnabled(True)
        self.wid.btn_open.setEnabled(True)


class TA11TIAGo(Plugin):
    LEFT_IDX  = 1
    RIGHT_IDX = 0

    FRC_THRESH = 1.5
    FRC_MAX    = 3.0
    FRC_STEP   = 0.010

    J_OPEN  = 0.05
    J_CLOSE = 0.0

    def __init__(self, context):
        super(TA11TIAGo, self).__init__(context)
        self.setObjectName('TA11TIAGo')

        # Create QWidget
        self._widget = QWidget()

        # load widget UI layout
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'ta11_tiago.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('TA11TIAGoUI')

        # Show _widget.windowTitle on left-top of each plugin
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)

        # these bags only contain one trajectory each
        open_bag = rosbag.Bag(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               '../../resources/steel_open.bag'))
        close_bag = rosbag.Bag(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                           '../../resources/steel_close.bag'))

        open_traj, close_traj = None, None
        for topic, msg, t in open_bag.read_messages():
            open_traj = msg.goal.trajectory
        for topic, msg, t in close_bag.read_messages():
            close_traj = msg.goal.trajectory
        open_bag.close()
        close_bag.close()

        open_traj.points = self.gen_traj_points(self.J_CLOSE, self.J_OPEN, 5)
        close_traj.points = self.gen_traj_points(self.J_OPEN, self.J_CLOSE, 5)

        rospy.loginfo("Opening trajectory with {} points and closing trajectory with {} points.".format(len(open_traj.points), len(close_traj.points)))
        self.open_traj = open_traj
        self.close_traj = close_traj

        self.traj_ac = None
        self.lc = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)

        # register button signals
        self._widget.btn_close.clicked.connect(self.on_btn_close)
        self._widget.btn_open.clicked.connect(self.on_btn_open)

        self._widget.btn_load_fc.clicked.connect(self.on_btn_load_fc)
        self._widget.btn_reload_fc.clicked.connect(self.on_btn_reload_fc)
        self._widget.btn_check_fc.clicked.connect(self.on_btn_check_fc)

        self.state_lbl = self._widget.lbl_fc_state
        self.state_lbl.setFixedWidth(95)
        self.state_lbl.setAlignment(QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter)

        self.on_btn_check_fc()
        self.act = None

    def gen_traj_points(self, first, last, total_time, num_points=5):
        pts = []
        for t, j in zip(np.linspace(0, total_time, num_points), np.linspace(first, last, num_points)):
            jp = JointTrajectoryPoint()
            jp.positions = [j, j]

            if t == 0.0: t += 0.1
            tm = rospy.Time(t)
            jp.time_from_start.secs = tm.secs
            jp.time_from_start.nsecs = tm.nsecs

            pts.append(jp)
        return pts

    def on_btn_load_fc(self):
        rospy.loginfo("Loading Force Controller ...")

        stop_controller("gripper_controller")
        rospy.loginfo("     - gripper_controller stopped")

        load_controller("gripper_force_controller")
        rospy.loginfo("     - gripper_force_controller loaded")

        start_controller("gripper_force_controller")
        rospy.loginfo("     - gripper_force_controller started")

        self.on_btn_check_fc()
        rospy.loginfo("Done!")

    def on_btn_reload_fc(self):
        rospy.loginfo("Reloading Force Controller ...")

        stop_controller("gripper_force_controller")
        rospy.loginfo("     - gripper_force_controller stopped")

        unload_controller("gripper_force_controller")
        rospy.loginfo("     - gripper_force_controller unloaded")

        load_controller("gripper_force_controller")
        rospy.loginfo("     - gripper_force_controller loaded")

        start_controller("gripper_force_controller")
        rospy.loginfo("     - gripper_force_controller started")

        self.on_btn_check_fc()
        rospy.loginfo("Done!")

    def on_btn_check_fc(self):
        try:
            cons = self.lc()
        except:
            self.state_lbl.setText("ERR")
            return

        if cons.controller == []:
            self.state_lbl.setText("UNL")
            return

        for c in cons.controller:
            if c.name == "gripper_force_controller":
                if c.state == 'running':
                    self.state_lbl.setText("RUN")
                elif c.state == 'stopped':
                    self.state_lbl.setText("STOP")
                return
        self.state_lbl.setText("UNL")

    def on_btn_close(self):
        rospy.loginfo("Closing Gripper")
        self.send_traj('close')

    def on_btn_open(self):
        rospy.loginfo("Opening Gripper")
        self.send_traj('open')

    def send_traj(self, traj_name):
        if not self.traj_ac:
            self.init_ac()

        g = FollowJointTrajectoryGoal()
        if traj_name == 'open':
            g.trajectory = self.open_traj
        elif traj_name == 'close':
            g.trajectory = self.close_traj

        self.traj_ac.send_goal(g)
        timeout = max(1.0, g.trajectory.points[-1].time_from_start.secs*1.5)
        self.act = ACThread(self.traj_ac, self._widget, timeout=timeout)
        self.act.start()

    def init_ac(self):
        self.traj_ac = actionlib.SimpleActionClient('/gripper_force_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        rospy.loginfo("Waiting for gripper_controller ...")
        self.traj_ac.wait_for_server()
        rospy.loginfo("Ready!")

    def shutdown_plugin(self):
        rospy.loginfo("TA11Test plugin shutting down ...")
        self.active = False
        rospy.loginfo("TA11Test plugin exited.")

    def save_settings(self, plugin_settings, instance_settings):
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # v = instance_settings.value(k)
        pass

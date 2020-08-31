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

from tiago_tactile_msgs.msg import TA11
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


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


class TA11Test(Plugin):
    LEFT_IDX  = 1
    RIGHT_IDX = 0

    FRC_THRESH = 1.5
    FRC_MAX    = 3.0
    FRC_STEP   = 0.010

    J_OPEN  = 0.05
    J_CLOSE = 0.0

    def __init__(self, context):
        super(TA11Test, self).__init__(context)
        self.setObjectName('TA11Test')

        # Create QWidget
        self._widget = QWidget()

        # load widget UI layout
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                               'ta11_test.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('TA11TestUi')

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

        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument('--publish', dest='publish', action='store_true')
        parser.add_argument('--no-publish', dest='publish', action='store_false')
        parser.set_defaults(publish=False)

        args, _ = parser.parse_known_args(context.argv())

        self._widget.check_pub.setChecked(args.publish)

        # register button signals
        self._widget.btn_close.clicked.connect(self.on_btn_close)
        self._widget.btn_open.clicked.connect(self.on_btn_open)

        self._widget.btn_frc_zero_both.clicked.connect(self.on_frc_zero_both)
        self._widget.btn_frc_zero_left.clicked.connect(self.on_frc_zero_left)
        self._widget.btn_frc_zero_right.clicked.connect(self.on_frc_zero_right)

        self._widget.btn_frc_thresh_both.clicked.connect(self.on_frc_thresh_both)
        self._widget.btn_frc_thresh_left.clicked.connect(self.on_frc_thresh_left)
        self._widget.btn_frc_thresh_right.clicked.connect(self.on_frc_thresh_right)

        self._widget.btn_frc_raise_both.clicked.connect(self.on_frc_raise_both)
        self._widget.btn_frc_raise_left.clicked.connect(self.on_frc_raise_left)
        self._widget.btn_frc_raise_right.clicked.connect(self.on_frc_raise_right)

        self._widget.check_pub.stateChanged.connect(self.check_pub_changed)

        self.active = True
        self.publishing_enabled = self._widget.check_pub.isChecked()
        self.pub_thread = Thread(target=self.publish_force)

        self.pub = rospy.Publisher('ta11', TA11, queue_size=10)
        self.pub_rate = rospy.Rate(50)

        self.current_force = [0.0, 0.0]
        self.force_raise = [False, False]

        self.sensor_frames = [
            "ta11_left_finger_link",
            "ta11_right_finger_link"
        ]

        self.act = None
        self.pub_thread.start()

    """
    Force = 0
    """
    def on_frc_zero_both(self):
        self.force_raise = [False, False]
        self.current_force = [0.0, 0.0]

    def on_frc_zero_left(self):
        self.force_raise[self.LEFT_IDX] = False
        self.current_force[self.LEFT_IDX] = 0.0

    def on_frc_zero_right(self):
        self.force_raise[self.RIGHT_IDX] = False
        self.current_force[self.RIGHT_IDX] = 0.0

    """
    Force = THRESH
    """
    def on_frc_thresh_both(self):
        self.force_raise = [False, False]
        self.current_force = [self.FRC_THRESH, self.FRC_THRESH]

    def on_frc_thresh_left(self):
        self.force_raise[self.LEFT_IDX] = False
        self.current_force[self.LEFT_IDX] = self.FRC_THRESH

    def on_frc_thresh_right(self):
        self.force_raise[self.RIGHT_IDX] = False
        self.current_force[self.RIGHT_IDX] = self.FRC_THRESH

    """
    Force -> LIM
    """
    def on_frc_raise_both(self):
        self.force_raise = [True, True]
        self.current_force = [self.FRC_THRESH, self.FRC_THRESH]

    def on_frc_raise_left(self):
        self.force_raise[self.LEFT_IDX] = True
        self.current_force[self.LEFT_IDX] = self.FRC_THRESH

    def on_frc_raise_right(self):
        self.force_raise[self.RIGHT_IDX] = True
        self.current_force[self.RIGHT_IDX] = self.FRC_THRESH

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

    def check_pub_changed(self, i):
        self.publishing_enabled = not self.publishing_enabled

    def publish_force(self):
        while self.active:
            if self.publishing_enabled:
                if any(self.force_raise):
                    for i in range(len(self.current_force)):
                        if not self.force_raise[i]: continue
                        self.current_force[i] = min(self.current_force[i]+self.FRC_STEP, self.FRC_MAX)

                tac = TA11()
                tac.header.frame_id = "base_link"
                tac.frame_names = self.sensor_frames
                tac.sensor_values = [self.current_force[self.RIGHT_IDX], self.current_force[self.LEFT_IDX]]

                self.pub.publish(tac)
                self.pub_rate.sleep()

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
        self.traj_ac = actionlib.SimpleActionClient('/gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        rospy.loginfo("Waiting for gripper_controller ...")
        self.traj_ac.wait_for_server()
        rospy.loginfo("Ready!")

    def shutdown_plugin(self):
        rospy.loginfo("TA11Test plugin shutting down ...")
        self.active = False
        self.pub_thread.join()
        rospy.loginfo("TA11Test plugin exited.")

    def save_settings(self, plugin_settings, instance_settings):
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # v = instance_settings.value(k)
        pass

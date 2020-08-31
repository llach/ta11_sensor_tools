## TA11 Force Sensor RViz Plugin

This plugin is designed to visualize force measurements of two opposing TA11 force sensors. It assumes that force measurements are published using `tactile_msgs/TactileState` with both measurements contained in the `sensors` list. The sensor names for this list are `(left|right)_gripper_tactile`, their force arrows will be shown in the frames `ta11_(left|right)_finger_link`. You need to make sure these frames are published.

`tactile_msgs` can be found [here](https://github.com/ubi-agni/tactile_toolbox).

When build in a catkin workspace, this plugin becomes available in RViz. 
# TA11 Force Controller

A reactive Joint Trajectory Controller using the [force_controller_core](https://github.com/llach/force_controller_core).
Expects force measurements to be published on `/ta11` using `TA11.msg` from [tiago_tactile_msgs](https://github.com/llach/tiago_tactile_msgs).
All relevant controller parameters can be changed via `rqt_reconfigure`.
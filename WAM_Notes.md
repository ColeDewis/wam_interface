# WAM Notes

Barrett Support Doc:
https://docs.google.com/document/d/1wcQizaw9qFHioFR587Kzm9KInagjELLxBfObVsJrqaA/edit?tab=t.0

## Internal vs External
Internal: switches C, D "In" (closer to motherboard)
External: switches C, D "Out"

SSH:
user: robot, password: wam
IP: 192.168.1.xx, xx=10 for zeus (middle), 11 slax (far), 12 ares (haptic).

May need to add these to /etc/hosts, and yourself to that on the WAM.

## Using
Code is in barrett-ros-pkg-zeusV

typically run roscore on external pc, then have setup file.

`source setup_wam.sh` with the PC name, need to define file for PC (should do for odin, laptop)

`rosrun wam_bringup wam_node`

Logging is in /var/log/syslog

Config path shows up there: typically in /etc/barrett. The exact file depends on wam setup (for gravity comp stuff)

etc/barrett/calibration_data also has data for gravity calibration, zeros, etc

If you need to change, should setup some file somwehere else, e.g. look at Dylan's Projects "setup_env.sh"
- uses some library "libconfig++" if need to google.

"Puck not found" => check can port

Shift+Idle powers pucks. To turn off wam node, better to shift + idle. however this keeps some config, so if you need to change that, shift idle then estop to reset it.

## ROS related
holds joint positions after you send them

`hold_joint_pos` for a flag to hold or not

pose published should be eef pose

go_home goes vertical first

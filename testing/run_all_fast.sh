#!/bin/bash
sleep 1s
{
gnome-terminal -t "start_kinect" -- bash -c "source ~/AirSim/ros/devel/setup.bash && roslaunch airsim_ros_pkgs kinect_publish_node.launch;exec bash"
}&

sleep 1s
{
gnome-terminal -t "start_airsim" -- bash -c "source ~/AirSim/ros/devel/setup.bash && roslaunch airsim_ros_pkgs airsim_node_fast.launch;exec bash"
}&

sleep 1s
{
gnome-terminal -t "start_controller" -- bash -c "source ~/AirSim/ros/devel/setup.bash && roslaunch airsim_ros_pkgs position_controller.launch;exec bash"
}&
/usr/bin/python3 ~/catkin_ws/src/3MT/pysrc/takeoff.py
{
gnome-terminal -t "start_rviz" -- bash -c "source ~/catkin_ws/devel/setup.bash && roslaunch plan_manage rviz.launch;exec bash"
}&
sleep 1s
{
gnome-terminal -t "start_planner" -- bash -c "source ~/catkin_ws/devel/setup.bash && roslaunch plan_manage kino_replan_airsim.launch;exec bash"
}&
sleep 2s
{
gnome-terminal -t "start_fly" -- bash -c 'rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ""
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"'
}&

#! /usr/bin/env bash
# Autoware_practice 第2章の02-02を一括実行するためのスクリプト
source install/setup.bash 
gnome-terminal -- ros2 launch autoware_practice_launch practice.launch.xml
sleep 1s # ノード同時起動防止
gnome-terminal -- ros2 topic echo /localization/kinematic_state
gnome-terminal -- ros2 run autoware_practice_course p_controller --ros-args -p kp:=0.5 -p target_velocity:=1.0 # kp0.5の場合はこちらをコメント解除する
# gnome-terminal -- ros2 run autoware_practice_course p_controller --ros-args -p kp:=5.0 -p target_velocity:=1.0 # kp5.0の場合はこちらをコメント解除する。
gnome-terminal -- ros2 bag record -o velocity.bag /localization/kinematic_state 
gnome-terminal -- ros2 run plotjuggler plotjuggler
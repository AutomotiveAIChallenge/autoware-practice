#! /usr/bin/env bash
# Autoware_practice 第2章の02-03(速度計画)を一括実行するためのスクリプト
source install/setup.bash
gnome-terminal -- ros2 launch autoware_practice_launch practice.launch.xml
sleep 1s # ノード同時起動防止
gnome-terminal -- ros2 run plotjuggler plotjuggler
sleep 60s # plotjugglerの設定するための待機時間
gnome-terminal -- ros2 run autoware_practice_course trajectory_loader --ros-args -p path_file:=src/autoware_practice_course/config/trajectory.csv
sleep 1s # ノード同時起動防止
gnome-terminal -- ros2 run autoware_practice_course longitudinal_controller --ros-args -p kp:=5.0

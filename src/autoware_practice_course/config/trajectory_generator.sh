#!/bin/bash

# ファイル名: trajectory_generator.sh

# 出力ファイル名
output_file="trajectory.csv"

# ヘッダー行を追加（必要な場合）
echo "position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w, longitudinal_velocity_mps, acceleration_mps2, heading_rate_rps, front_wheel_angle_rad, rear_wheel_angle_rad" > $output_file

# 軌道データの生成
for i in $(seq 0 30); do
  x=$(awk "BEGIN {printf \"%.1f\", $i * 1.0}")
  y=0.0
  other="0.0,0.0,0.0,0.0,1.0,5.0,0.0,0.0,0.0,0.0,0.0"
  echo "$x,$y,$other" >> $output_file
done

for i in $(seq 0 30); do
  x=30.0
  y=$(awk "BEGIN {printf \"%.1f\", $i * 1.0}")
  other="0.0,0.0,0.0,0.0,1.0,5.0,0.0,0.0,0.0,0.0,0.0"
  echo "$x,$y,$other" >> $output_file
done

for i in $(seq 30 100); do
  x=$(awk "BEGIN {printf \"%.1f\", $i * 1.0}")
  y=30.0
  other="0.0,0.0,0.0,0.0,1.0,5.0,0.0,0.0,0.0,0.0,0.0"
  echo "$x,$y,$other" >> $output_file
done


echo "Trajectory data has been generated and saved to $output_file"


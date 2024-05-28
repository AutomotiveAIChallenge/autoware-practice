#!/bin/bash

# 出力ファイル名
output_file="trajectory.csv"

echo "position_x, position_y, longitudinal_velocity_mps" > $output_file

# 軌道データの生成
for i in $(seq 0 30); do
  x=$(awk "BEGIN {printf \"%.1f\", $i * 1.0}")
  y=0.0
  longitudinal_velocity="3.0"
  echo "$x,$y,$longitudinal_velocity" >> $output_file
done

for i in $(seq 0 30); do
  x=30.0
  y=$(awk "BEGIN {printf \"%.1f\", $i * 1.0}")
  longitudinal_velocity="3.0"
  echo "$x,$y,$longitudinal_velocity" >> $output_file
done

for i in $(seq 30 100); do
  x=$(awk "BEGIN {printf \"%.1f\", $i * 1.0}")
  y=30.0
  longitudinal_velocity="3.0"
  echo "$x,$y,$longitudinal_velocity" >> $output_file
done


echo "Trajectory data has been generated and saved to $output_file"


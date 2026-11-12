#!/bin/bash
CFG=$(realpath $1); TAG=$2; OUT=/home/artrc/grasp_sync_backups/claude_front_v10_stagewise_20261110/gpu_$TAG; mkdir -p $OUT
cd /home/artrc/grasp_sync_backups/claude_front_v10_stagewise_20261110
for y in 3 -3; do
  YAW_DEG=$y timeout 500 python3 yaw_sim.py --headless --climb-start --climb-config $CFG --climb-speed 1 --climb-joint-speed 1.8 --climb-metrics $OUT/m_yaw$y.json > $OUT/log_yaw$y.txt 2>&1
  echo "yaw $y: $(grep -E 'ROTATED' $OUT/log_yaw$y.txt | cut -c1-45) | $(grep -E 'PREVIEW VERDICT' $OUT/log_yaw$y.txt | cut -c20-45) $(grep -oE 'FAILED [A-Z_0-9]+' $OUT/log_yaw$y.txt | tail -1)"
done

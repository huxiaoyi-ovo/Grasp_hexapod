#!/usr/bin/env bash
set +u
source /opt/ros/noetic/setup.bash
source /home/artrc/guangfu/Grasp_hexapod/devel/setup.bash
roscore > /tmp/roscore_e2e.log 2>&1 &
CORE_PID=$!
sleep 3
rosrun grasp_hexapod_control_cpp run_real_cpp \
  _enable_real_dock:=false _dock_system_config:=/nonexistent.yaml \
  > /tmp/cpp_node_e2e.log 2>&1 &
NODE_PID=$!
sleep 2
timeout 30 python3 /tmp/fake_hardware.py
FAKE_EXIT=$?
sleep 1
kill -INT $NODE_PID 2>/dev/null
sleep 1
kill -9 $NODE_PID 2>/dev/null
kill -9 $CORE_PID 2>/dev/null
sleep 1
echo "FAKE_EXIT=$FAKE_EXIT"
echo "---NODE LOG---"
grep -E "control tick|B pressed|Stand init|Motion enabled|paused|feedback lost|CLIMB" /tmp/cpp_node_e2e.log | head -10
exit 0

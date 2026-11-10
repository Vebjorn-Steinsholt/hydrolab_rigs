#!/usr/bin/env bash
set -eo pipefail
[[ -n "$ROS_DOMAIN_ID" && "$ROS_DOMAIN_ID" != 0 ]] || { echo "set ROS_DOMAIN_ID" >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
source "$HOME/ros_ws/install/setup.bash"
# One button, published to every controller that can run on this Pi.
exec ros2 run rig_controller soft_stop --ros-args -r __ns:="/$PI" -p pin:=17 \
  -p "topics:=['/wavemakers/$PI/soft_stop', '/oscillators/vertical/soft_stop', '/oscillators/sloshing/soft_stop']"

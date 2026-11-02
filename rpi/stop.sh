#!/usr/bin/env bash
# ExecStop: shut the managed nodes down in order (bridge, then controller) through the lifecycle
# manager. systemd sends SIGINT when this finishes, so if it fails, the SIGINT path still stops
# the drive.
set -eo pipefail

# After the idle auto-stop, ros2 launch has already exited: nothing to shut down.
[[ -n "$MAINPID" ]] || exit 0

source /opt/ros/jazzy/setup.bash
source "$HOME/ros_ws/install/setup.bash"

case "${RIG:-wavemaker}" in
  oscillator) namespace=/oscillators/sloshing ;;
  *)          namespace="/wavemakers/$WAVEMAKER" ;;
esac

# ros2 service call has no timeout; without one it waits forever for the service.
timeout 15 ros2 service call "$namespace/lifecycle_manager/manage_nodes" \
  nav2_msgs/srv/ManageLifecycleNodes "{command: 4}"   # 4 = SHUTDOWN

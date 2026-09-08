#!/usr/bin/env bash
# Setup standalone workspace from THIS repo only (no dofbot_ws needed).
# Usage: ./setup_standalone.sh
set -e
WS="$(cd "$(dirname "$0")" && pwd)"
SRC="$WS/src"

echo "=== 1/4 sys deps (stockfish, python-chess) ==="
sudo apt update && sudo apt install -y stockfish
pip install -U chess

echo "=== 2/4 dofbot_urdf (Yahboom, not vendored, ~104M) ==="
if [ -d "$SRC/dofbot_urdf" ]; then
  echo "OK: src/dofbot_urdf already present, skip."
else
  COPIED=0
  for cand in \
    "$HOME/dofbot_ws/src/dofbot_urdf" \
    "/home/yahboom/dofbot_ws/src/dofbot_urdf" \
    "$HOME/Dofbot/dofbot_urdf" ; do
    if [ -d "$cand" ]; then
      echo "Copying dofbot_urdf from $cand ..."
      cp -a "$cand" "$SRC/dofbot_urdf"
      COPIED=1
      break
    fi
  done
  if [ "$COPIED" = "0" ]; then
    echo "ERROR: src/dofbot_urdf missing."
    echo "Copy the Yahboom dofbot_urdf package (from robot image / SDK)"
    echo "into $SRC/dofbot_urdf, then re-run this script."
    echo "Driver/resources: https://drive.google.com/drive/folders/1N8DdsQJRkj8_7xfk3T-jFWvssnKH7QYD?usp=sharing"
    exit 1
  fi
fi

echo "=== 3/4 build ==="
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --packages-select pymoveit2 dofbot_urdf dofbot_moveit dofbot_tea_moveit chess_moveit_demo

echo "=== 4/4 done ==="
echo "source $WS/install/setup.bash"
echo "Chess base: ros2 launch chess_moveit_demo chess_base.launch.py"
echo "Chess sim : ros2 launch chess_moveit_demo chess_sim.launch.py"
echo "Tea       : ros2 launch dofbot_tea_moveit tea_backend.launch.py + ros2 launch dofbot_tea_moveit tea_task.launch.py"

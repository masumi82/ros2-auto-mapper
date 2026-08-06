#!/bin/bash
# Sealed-house autonomous mapping: full reset -> bringup -> verify -> explore.
# Run as a file so pkill patterns never match the invoking command line
# (running the same commands inline via `bash -c` would self-match and kill itself).
#
# Prerequisites:
#   - ROS 2 Humble + turtlebot3_gazebo + slam_toolbox + nav2_bringup
#   - auto_mapper package built in ~/ros2_ws (colcon build --packages-select auto_mapper)
#   - TURTLEBOT3_MODEL=burger
LOG=${LOG_DIR:-/tmp/auto_mapper_logs}
mkdir -p "$LOG"
REPO=$(cd "$(dirname "$0")/.." && pwd)
MAP_OUT=${MAP_OUT:-$HOME/ros2_ws/maps/house_sealed_final}

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
# NOTE: do NOT `set -u` — ROS setup.bash references unbound variables.

step() { echo "== $1 =="; }

step "0. 全停止"
pkill -9 -f gzserver; pkill -9 -f gzclient
pkill -9 -f component_container; pkill -9 -f rviz2
pkill -9 -f slam_toolbox; pkill -9 -f robot_state_publisher
pkill -9 -f static_transform_publisher; pkill -9 -f auto_explorer
pkill -9 -f bringup_launch; pkill -9 -f turtlebot3_house
pkill -9 -f lifecycle_manager; pkill -9 -f map_saver
sleep 4
echo "残存: $(pgrep -f 'gzserver|slam_toolbox|component_container|robot_state_publisher' | wc -l)"

step "1. Gazebo起動(家ワールド)"
# spawn INSIDE the house — the launch default (-2.0,-0.5) is the exterior courtyard!
setsid ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py \
  x_pose:=-3.0 y_pose:=2.0 > "$LOG/s_gz.log" 2>&1 < /dev/null &
for i in $(seq 1 60); do
  grep -qE "Successfully spawned|Spawn service failed" "$LOG/s_gz.log" 2>/dev/null && break
  sleep 5
done
grep -oE "Successfully spawned entity \[burger\]|Spawn service failed" "$LOG/s_gz.log" | head -1

step "2. 玄関封鎖壁"
# openings measured from a live SLAM map: front door = x[0.66,1.21], side gap = x[4.36,4.66]
timeout 70 ros2 run gazebo_ros spawn_entity.py -entity front_door_plug \
  -file "$REPO/worlds/door_plug.sdf" -x 0.94 -y -0.08 -z 0.5 2>&1 | tail -1
timeout 70 ros2 run gazebo_ros spawn_entity.py -entity side_gap_plug \
  -file "$REPO/worlds/door_plug.sdf" -x 4.51 -y -0.08 -z 0.5 2>&1 | tail -1

step "3. rsp停止(スラッシュTF源を排除)+TF橋"
pkill -9 -f robot_state_publisher; sleep 1
setsid ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.010 \
  --frame-id base_footprint --child-frame-id base_link \
  --ros-args -p use_sim_time:=true > /dev/null 2>&1 < /dev/null &
setsid ros2 run tf2_ros static_transform_publisher --x -0.032 --y 0 --z 0.172 \
  --frame-id base_link --child-frame-id base_scan \
  --ros-args -p use_sim_time:=true > /dev/null 2>&1 < /dev/null &
sleep 3

step "4. SLAM起動(async)"
setsid ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  > "$LOG/s_slam.log" 2>&1 < /dev/null &
sleep 12
echo "SLAMプロセス: $(pgrep -f async_slam | wc -l)"

step "5. Nav2起動(探索用パラメータ)"
# nav2_params_explore.yaml: track_unknown_space=false, inflation_radius=0.15
# (0.55 のままだと幅0.8mの東廊下が数学的に通行不能になる)
setsid ros2 launch nav2_bringup bringup_launch.py slam:=True \
  map:=$HOME/ros2_ws/maps/arena.yaml use_sim_time:=true \
  params_file:="$REPO/config/nav2_params_explore.yaml" \
  > "$LOG/s_nav2.log" 2>&1 < /dev/null &
for i in $(seq 1 60); do
  N=$(grep -c "Managed nodes are active" "$LOG/s_nav2.log" 2>/dev/null || echo 0)
  [ "$N" -ge 2 ] && break
  sleep 3
done
echo "Nav2 active行: $(grep -c 'Managed nodes are active' "$LOG/s_nav2.log" 2>/dev/null)"
# bringup内蔵のsync SLAMは重複するので停止(パス指定でasyncを誤射しない —
# "sync_slam" は "async_slam" の部分文字列である点に注意)
pkill -9 -f "/sync_slam"
sleep 2
echo "SLAM最終: async=$(pgrep -f async_slam | wc -l) sync=$(pgrep -f /sync_slam | wc -l)"

step "6. SLAM成長テスト(前進)"
python3 - <<'PYEOF'
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n=Node('growth'); counts=[]
n.create_subscription(OccupancyGrid,'/map',lambda m:counts.append(sum(1 for c in m.data if c>=0)),1)
pub=n.create_publisher(Twist,'/cmd_vel',10)
t=time.time()+6
while time.time()<t: rclpy.spin_once(n,timeout_sec=0.3)
before=counts[-1] if counts else 0
tw=Twist(); tw.linear.x=0.08
t=time.time()+14
while time.time()<t: pub.publish(tw); rclpy.spin_once(n,timeout_sec=0.1)
pub.publish(Twist())
t=time.time()+7
while time.time()<t: rclpy.spin_once(n,timeout_sec=0.3)
after=counts[-1] if counts else 0
print(f'GROWTH {before} -> {after} (+{after-before})')
raise SystemExit(0 if after-before>50 else 1)
PYEOF
if [ $? -ne 0 ]; then echo "SLAM成長NG — 中断"; exit 1; fi

step "7. 完全自動探索"
exec ros2 run auto_mapper auto_explorer --ros-args -p use_sim_time:=true \
  -p min_cluster:=4 -p max_goals:=60 \
  -p map_save_name:="$MAP_OUT"

#!/bin/bash
# Object patrol bringup: reset -> Gazebo(house) -> door plugs -> Nav2(known map) -> initial pose.
# Run as a file so kill patterns never match the invoking command line.
LOG=${LOG:-/tmp/object_patrol_logs}
mkdir -p "$LOG"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/ros2_ws/fastdds_wsl.xml
export TURTLEBOT3_MODEL=waffle_pi

MAP=${MAP:-$HOME/ros2_ws/maps/house_sealed_final.yaml}
WORLD=${WORLD:-}   # 空ならturtlebot3_houseランチのデフォルトワールド
X=${X:--3.0}; Y=${Y:-2.0}

step() { echo "== $1 =="; }

step "0. 全停止"
pkill -9 -f gzserver; pkill -9 -f gzclient
pkill -9 -f component_container; pkill -9 -f rviz2
pkill -9 -f robot_state_publisher; pkill -9 -f static_transform_publisher
pkill -9 -f bringup_launch; pkill -9 -f turtlebot3_house
pkill -9 -f navigation2.launch; pkill -9 -f lifecycle_manager
pkill -9 -f yolo_detector; pkill -9 -f object_mapper; pkill -9 -f patrol_node
sleep 4

step "1. Gazebo起動(家ワールド)"
if [ -n "$WORLD" ]; then
  setsid ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py \
    world:="$WORLD" x_pose:=$X y_pose:=$Y > "$LOG/gz.log" 2>&1 < /dev/null &
else
  setsid ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py \
    x_pose:=$X y_pose:=$Y > "$LOG/gz.log" 2>&1 < /dev/null &
fi
for i in $(seq 1 60); do
  grep -qE "Successfully spawned|Spawn service failed" "$LOG/gz.log" 2>/dev/null && break
  sleep 5
done
grep -oE "Successfully spawned entity \[[a-z_]*\]|Spawn service failed" "$LOG/gz.log" | head -1

step "2. 封鎖壁(玄関・側面)"
timeout 70 ros2 run gazebo_ros spawn_entity.py -entity front_door_plug \
  -file $HOME/ros2_ws/worlds/door_plug.sdf -x 0.94 -y -0.08 -z 0.5 2>&1 | tail -1
timeout 70 ros2 run gazebo_ros spawn_entity.py -entity side_gap_plug \
  -file $HOME/ros2_ws/worlds/door_plug.sdf -x 4.51 -y -0.08 -z 0.5 2>&1 | tail -1

step "3. Nav2起動(既知地図)"
setsid ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true map:="$MAP" > "$LOG/nav2.log" 2>&1 < /dev/null &
for i in $(seq 1 60); do
  [ "$(grep -c 'Creating bond timer' "$LOG/nav2.log" 2>/dev/null)" -ge 2 ] && break
  sleep 5
done
echo "Nav2 bond timers: $(grep -c 'Creating bond timer' "$LOG/nav2.log")"

step "4. AMCL初期位置"
timeout 30 ros2 run object_patrol set_initial_pose --ros-args -p x:=$X -p y:=$Y
echo "initial pose sent ($X, $Y)"

if [ "$PATROL" = "1" ]; then
  step "5. 検出・巡回ノード"
  setsid ~/ros2_ws/yolo_venv/bin/python -m object_patrol.yolo_detector \
    > "$LOG/yolo.log" 2>&1 < /dev/null &
  setsid ros2 run object_patrol object_mapper --ros-args -p map_yaml:="$MAP" \
    > "$LOG/mapper.log" 2>&1 < /dev/null &
  sleep 8
  ros2 run object_patrol patrol_node --ros-args \
    -p points_file:=$HOME/persol_ws/ros/ros2-auto-mapper/config/patrol_points.yaml \
    2>&1 | tee "$LOG/patrol.log"
  step "巡回完了"
fi
echo "READY"

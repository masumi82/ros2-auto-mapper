# object_patrol 実装計画

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 既知地図を巡回しながらYOLOで物体を発見し、カメラ+LiDARフュージョンで位置を地図に記録するロボットを作る。

**Architecture:** map_server+AMCL+Nav2の上に自作3ノード(patrol_node / yolo_detector / object_mapper)を載せる。位置推定の数学(方位角・距離抽出・重複統合)は純粋関数モジュールに分離しpytestでTDD。Gazebo統合はフェーズごとに手動検証。

**Tech Stack:** ROS 2 Humble / Gazebo Classic / TurtleBot3 waffle_pi / Nav2 / YOLOv8n(ultralytics, CPU) / OpenCV / pytest

## Global Constraints

- 設計書: `docs/specs/2026-08-17-object-patrol-design.md`(本計画の正)
- ブランチ: `feature/object-patrol`。コミットはすべてこのブランチへ
- 開発場所: リポジトリ `~/persol_ws/ros/ros2-auto-mapper/` 内にパッケージを作成し、`~/ros2_ws/src/` へシンボリックリンクしてビルド(直接コミット可能にするため)
- 起動スクリプトは必ず `export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/fastdds_wsl.xml` を含める(前回の教訓: export忘れは「手動○・スクリプト×」の症状を起こす)
- `export TURTLEBOT3_MODEL=waffle_pi`(カメラ搭載モデル。前回のburgerではない)
- YOLO信頼度しきい値 0.6 / 推論間引き 2〜3fps / 同一物体統合距離 0.5m / 確定に3フレーム観測
- Gazebo/長時間プロセスは `run_in_background` で起動し、pkill時は `[r]`ブラケットパターン+単独コマンド(前回の教訓)
- 各タスク完了時にコミット(メッセージ末尾に `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`)

---

### Task 1: パッケージ骨格とメッセージ定義

**Files:**
- Create: `object_patrol_msgs/package.xml`, `object_patrol_msgs/CMakeLists.txt`, `object_patrol_msgs/msg/DetectedObject.msg`, `object_patrol_msgs/msg/DetectedObjects.msg`
- Create: `object_patrol/package.xml`, `object_patrol/setup.py`, `object_patrol/setup.cfg`, `object_patrol/resource/object_patrol`, `object_patrol/object_patrol/__init__.py`

**Interfaces:**
- Produces: `object_patrol_msgs/msg/DetectedObject`(string class_name / float32 confidence / int32 x1,y1,x2,y2)、`object_patrol_msgs/msg/DetectedObjects`(std_msgs/Header header / DetectedObject[] objects)。以降の全タスクがこの型を使う
- Produces: ament_pythonパッケージ `object_patrol`(以降のノードはここに追加)

- [ ] **Step 1: メッセージパッケージを作成**

`object_patrol_msgs/msg/DetectedObject.msg`:
```
string class_name
float32 confidence
int32 x1
int32 y1
int32 x2
int32 y2
```

`object_patrol_msgs/msg/DetectedObjects.msg`:
```
std_msgs/Header header
DetectedObject[] objects
```

`object_patrol_msgs/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.8)
project(object_patrol_msgs)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  DEPENDENCIES std_msgs
)
ament_package()
```

`object_patrol_msgs/package.xml`(要点: `<buildtool_depend>ament_cmake</buildtool_depend>`, `<build_depend>rosidl_default_generators</build_depend>`, `<exec_depend>rosidl_default_runtime</exec_depend>`, `<depend>std_msgs</depend>`, `<member_of_group>rosidl_interface_packages</member_of_group>`)

- [ ] **Step 2: Pythonパッケージ骨格を作成**

`object_patrol/setup.py` の `entry_points` は後続タスクで追記していく。初期状態:
```python
from setuptools import setup
package_name = 'object_patrol'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': []},
)
```

- [ ] **Step 3: シンボリックリンクを張りビルド**

```bash
ln -s ~/persol_ws/ros/ros2-auto-mapper/object_patrol_msgs ~/ros2_ws/src/object_patrol_msgs
ln -s ~/persol_ws/ros/ros2-auto-mapper/object_patrol ~/ros2_ws/src/object_patrol
cd ~/ros2_ws && colcon build --packages-select object_patrol_msgs object_patrol --symlink-install
```
Expected: ビルド成功。`source install/setup.bash && ros2 interface show object_patrol_msgs/msg/DetectedObjects` で定義が表示される

- [ ] **Step 4: コミット**(`feat: add object_patrol package skeleton and messages`)

---

### Task 2: YOLO環境構築と静止認識テスト

**Files:**
- Create: `scripts/test_yolo_static.py`(スタンドアロン確認スクリプト)
- Create: `docs/images/yolo_static_test.png`(認識結果エビデンス)

**Interfaces:**
- Produces: venv `~/ros2_ws/yolo_venv`(--system-site-packagesでrclpyも見える)。以降yolo_detectorはこのvenvのpythonで実行
- Produces: 採用物体モデルのリスト(設計書§6の「5種類程度」をここで確定し、本計画Task 9のワールド定義に反映)

- [ ] **Step 1: venv作成とultralyticsインストール**

```bash
python3 -m venv --system-site-packages ~/ros2_ws/yolo_venv
~/ros2_ws/yolo_venv/bin/pip install ultralytics
```
失敗した場合のフォールバック: `uv venv --system-site-packages ~/ros2_ws/yolo_venv && uv pip install --python ~/ros2_ws/yolo_venv/bin/python ultralytics`(前回venv作成に失敗しuvで解決した実績あり)

- [ ] **Step 2: YOLOv8n単体で推論確認**

```bash
~/ros2_ws/yolo_venv/bin/python -c "
from ultralytics import YOLO
m = YOLO('yolov8n.pt')
r = m('https://ultralytics.com/images/bus.jpg')
print([m.names[int(c)] for c in r[0].boxes.cls])"
```
Expected: `['bus', 'person', ...]` のような出力(モデルDLと推論が動く)

- [ ] **Step 3: Gazeboで静止認識テスト**

waffle_piを空ワールドに置き、正面2mに候補モデルを1つずつスポーンして `/camera/image_raw` をYOLOにかける。候補: `person_standing`, `person_walking`(person)、`cafe_table`(dining table)、`fire_hydrant`(fire hydrant)、`stop_sign`(stop sign)、`bookshelf`(±)、TB3ハウス既設の家具。

`scripts/test_yolo_static.py`(要点):
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2, sys

class StaticTest(Node):
    def __init__(self):
        super().__init__('yolo_static_test')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 1)
    def cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        res = self.model(img, verbose=False)[0]
        for b in res.boxes:
            print(self.model.names[int(b.cls)], float(b.conf))
        cv2.imwrite(sys.argv[1] if len(sys.argv) > 1 else '/tmp/yolo_test.png', res.plot())
        raise SystemExit
rclpy.init(); rclpy.spin(StaticTest())
```
実行: Gazebo起動(FASTRTPS export・TURTLEBOT3_MODEL=waffle_pi)→ モデルをスポーン → `~/ros2_ws/yolo_venv/bin/python scripts/test_yolo_static.py`
Expected: 各候補モデルの認識可否と信頼度の一覧が得られる。**信頼度0.6以上で認識できたモデルを5種類前後選定**(CG感で全滅した場合はしきい値0.5に緩和を検討し、結果を記録)

- [ ] **Step 4: 結果を記録してコミット**(`feat: add YOLO static recognition test + adopted models list`。採用モデル一覧はコミットメッセージと本計画Task 9に記載)

---

### Task 3: フュージョン数学モジュール(TDD)

**Files:**
- Create: `object_patrol/object_patrol/fusion.py`
- Test: `object_patrol/test/test_fusion.py`

**Interfaces:**
- Produces: `pixel_to_bearing(px, image_width, hfov) -> float`(左が正のラジアン)、`scan_range_at(ranges, angle_min, angle_increment, bearing, window_deg=3.0) -> float | None`(±window内の有効値の中央値)、`bearing_range_to_xy(bearing, rng) -> (x, y)`(ロボット座標系、前方x+左y+)

- [ ] **Step 1: 失敗するテストを書く**

`object_patrol/test/test_fusion.py`:
```python
import math
import pytest
from object_patrol.fusion import pixel_to_bearing, scan_range_at, bearing_range_to_xy

HFOV = 1.085595  # waffle_pi camera horizontal FOV [rad]

def test_center_pixel_is_zero_bearing():
    assert pixel_to_bearing(320, 640, HFOV) == pytest.approx(0.0)

def test_left_edge_is_positive_half_fov():
    assert pixel_to_bearing(0, 640, HFOV) == pytest.approx(HFOV / 2, abs=1e-6)

def test_right_edge_is_negative_half_fov():
    assert pixel_to_bearing(640, 640, HFOV) == pytest.approx(-HFOV / 2, abs=1e-6)

def test_scan_range_median_ignores_inf():
    # 360本・1度刻み、bearing=0付近に [2.0, inf, 2.2] → 中央値2.1系
    ranges = [float('inf')] * 360
    ranges[359], ranges[0], ranges[1] = 2.0, float('inf'), 2.2
    r = scan_range_at(ranges, 0.0, math.radians(1), 0.0, window_deg=2.0)
    assert r == pytest.approx(2.2)  # 有効値[2.0,2.2]の中央値(上位側)

def test_scan_range_returns_none_when_all_invalid():
    assert scan_range_at([float('inf')] * 360, 0.0, math.radians(1), 0.0) is None

def test_scan_range_wraps_around_zero():
    # bearing=-2度(=358度側)でもラップして拾える
    ranges = [float('inf')] * 360
    ranges[358] = 1.5
    r = scan_range_at(ranges, 0.0, math.radians(1), math.radians(-2), window_deg=1.5)
    assert r == pytest.approx(1.5)

def test_bearing_range_to_xy_forward():
    x, y = bearing_range_to_xy(0.0, 2.0)
    assert (x, y) == (pytest.approx(2.0), pytest.approx(0.0))

def test_bearing_range_to_xy_left():
    x, y = bearing_range_to_xy(math.pi / 2, 3.0)
    assert x == pytest.approx(0.0, abs=1e-9) and y == pytest.approx(3.0)
```

- [ ] **Step 2: 失敗を確認** — `cd ~/persol_ws/ros/ros2-auto-mapper/object_patrol && python3 -m pytest test/test_fusion.py -v` → Expected: ImportError で全FAIL

- [ ] **Step 3: 実装**

`object_patrol/object_patrol/fusion.py`:
```python
import math


def pixel_to_bearing(px, image_width, hfov):
    """バウンディングボックス中心の横ピクセル → 方位角[rad](左が正)。"""
    fx = image_width / (2.0 * math.tan(hfov / 2.0))
    cx = image_width / 2.0
    return math.atan2(cx - px, fx)


def scan_range_at(ranges, angle_min, angle_increment, bearing, window_deg=3.0):
    """bearing±window内の有効距離の中央値。有効値なしならNone。"""
    window = math.radians(window_deg)
    vals = []
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_increment
        diff = math.atan2(math.sin(a - bearing), math.cos(a - bearing))
        if abs(diff) <= window and r is not None and math.isfinite(r) and r > 0.05:
            vals.append(r)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def bearing_range_to_xy(bearing, rng):
    """(方位角, 距離) → ロボット座標系の(x, y)。前方x+、左y+。"""
    return rng * math.cos(bearing), rng * math.sin(bearing)
```

注: `test_left_edge` はatan2式だと `atan(tan(HFOV/2)) == HFOV/2` で厳密に一致する。

- [ ] **Step 4: パス確認** — 同コマンド → Expected: 8 passed
- [ ] **Step 5: コミット**(`feat: add camera-LiDAR fusion math with tests`)

---

### Task 4: 物体レジストリ(重複統合・確定判定、TDD)

**Files:**
- Create: `object_patrol/object_patrol/registry.py`
- Test: `object_patrol/test/test_registry.py`

**Interfaces:**
- Produces: `ObjectRegistry(merge_dist=0.5, confirm_frames=3)`。`observe(class_name, x, y) -> dict`(統合or新規登録して返す)、`confirmed() -> list[dict]`(確定済みのみ)。dictキー: `class_name, x, y, count, confirmed`

- [ ] **Step 1: 失敗するテストを書く**

`object_patrol/test/test_registry.py`:
```python
import pytest
from object_patrol.registry import ObjectRegistry

def test_new_object_not_confirmed_until_3_frames():
    reg = ObjectRegistry(merge_dist=0.5, confirm_frames=3)
    reg.observe('person', 1.0, 2.0)
    reg.observe('person', 1.1, 2.0)
    assert reg.confirmed() == []
    reg.observe('person', 1.0, 2.1)
    assert len(reg.confirmed()) == 1

def test_nearby_same_class_merges_with_mean_position():
    reg = ObjectRegistry(confirm_frames=1)
    reg.observe('chair', 0.0, 0.0)
    o = reg.observe('chair', 0.4, 0.0)
    assert len(reg.confirmed()) == 1
    assert o['x'] == pytest.approx(0.2)

def test_far_same_class_registers_separately():
    reg = ObjectRegistry(merge_dist=0.5, confirm_frames=1)
    reg.observe('chair', 0.0, 0.0)
    reg.observe('chair', 2.0, 0.0)
    assert len(reg.confirmed()) == 2

def test_different_class_never_merges():
    reg = ObjectRegistry(confirm_frames=1)
    reg.observe('chair', 0.0, 0.0)
    reg.observe('bottle', 0.1, 0.0)
    assert len(reg.confirmed()) == 2
```

- [ ] **Step 2: 失敗を確認** — `python3 -m pytest test/test_registry.py -v` → ImportError で全FAIL
- [ ] **Step 3: 実装**

`object_patrol/object_patrol/registry.py`:
```python
import math


class ObjectRegistry:
    """発見物体の登録簿。近接同一クラスは統合し、複数フレーム観測で確定する。"""

    def __init__(self, merge_dist=0.5, confirm_frames=3):
        self.merge_dist = merge_dist
        self.confirm_frames = confirm_frames
        self._objects = []

    def observe(self, class_name, x, y):
        for o in self._objects:
            if o['class_name'] == class_name and math.hypot(o['x'] - x, o['y'] - y) <= self.merge_dist:
                n = o['count']
                o['x'] = (o['x'] * n + x) / (n + 1)
                o['y'] = (o['y'] * n + y) / (n + 1)
                o['count'] = n + 1
                o['confirmed'] = o['count'] >= self.confirm_frames
                return o
        o = {'class_name': class_name, 'x': x, 'y': y, 'count': 1,
             'confirmed': self.confirm_frames <= 1}
        self._objects.append(o)
        return o

    def confirmed(self):
        return [o for o in self._objects if o['confirmed']]
```

- [ ] **Step 4: パス確認** — 4 passed
- [ ] **Step 5: コミット**(`feat: add object registry with merge and confirmation logic`)

---

### Task 5: yolo_detectorノード

**Files:**
- Create: `object_patrol/object_patrol/yolo_detector.py`
- Modify: `object_patrol/setup.py`(entry_points追記)、`object_patrol/package.xml`(depend追記: rclpy, sensor_msgs, cv_bridge, object_patrol_msgs)

**Interfaces:**
- Consumes: `/camera/image_raw`(sensor_msgs/Image)
- Produces: `/detected_objects`(object_patrol_msgs/DetectedObjects、headerは入力画像のheaderをそのままコピー — object_mapperがTF時刻参照に使う)、`/detection_image`(sensor_msgs/Image、検出枠描画済み)
- パラメータ: `conf_threshold`(0.6) / `infer_interval_sec`(0.4 ≒2.5fps)

- [ ] **Step 1: 実装**

`object_patrol/object_patrol/yolo_detector.py`:
```python
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from object_patrol_msgs.msg import DetectedObject, DetectedObjects


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.declare_parameter('conf_threshold', 0.6)
        self.declare_parameter('infer_interval_sec', 0.4)
        self.conf = self.get_parameter('conf_threshold').value
        self.interval = self.get_parameter('infer_interval_sec').value
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.last_infer = 0.0
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 1)
        self.pub = self.create_publisher(DetectedObjects, '/detected_objects', 10)
        self.pub_img = self.create_publisher(Image, '/detection_image', 1)
        self.get_logger().info('yolo_detector ready')

    def cb(self, msg):
        now = time.monotonic()
        if now - self.last_infer < self.interval:
            return  # 間引き(WSL2のCPU負荷対策)
        self.last_infer = now
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        res = self.model(img, verbose=False)[0]
        out = DetectedObjects()
        out.header = msg.header  # TF時刻参照のため入力画像の時刻を引き継ぐ
        for b in res.boxes:
            if float(b.conf) < self.conf:
                continue
            d = DetectedObject()
            d.class_name = self.model.names[int(b.cls)]
            d.confidence = float(b.conf)
            x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
            d.x1, d.y1, d.x2, d.y2 = x1, y1, x2, y2
            out.objects.append(d)
        self.pub.publish(out)
        ann = self.bridge.cv2_to_imgmsg(res.plot(), 'bgr8')
        ann.header = msg.header
        self.pub_img.publish(ann)


def main():
    rclpy.init()
    rclpy.spin(YoloDetector())


if __name__ == '__main__':
    main()
```

setup.py entry_points に `'yolo_detector = object_patrol.yolo_detector:main'` を追加。

- [ ] **Step 2: ビルドして単体動作確認**

```bash
cd ~/ros2_ws && colcon build --packages-select object_patrol --symlink-install
```
Gazebo(waffle_pi + Task 2の採用モデル1個)を起動した状態で:
```bash
source ~/ros2_ws/install/setup.bash
~/ros2_ws/yolo_venv/bin/python -m object_patrol.yolo_detector  # venv実行(ultralyticsが必要なため)
```
別端末: `ros2 topic echo /detected_objects --once`
Expected: 配置モデルのclass_nameと信頼度が流れる。`ros2 topic hz /detected_objects` が約2.5Hz

- [ ] **Step 3: コミット**(`feat: add yolo_detector node`)

---

### Task 6: 既知地図での走行確認(map_server + AMCL + Nav2)

**Files:**
- Create: `scripts/run_object_patrol.sh`(統合起動スクリプト。このタスク時点ではGazebo+Nav2+initialposeまで)
- Create: `object_patrol/object_patrol/set_initial_pose.py`、setup.py entry_points追記

**Interfaces:**
- Consumes: `maps/` の前回生成地図(yaml+pgm)
- Produces: 起動スクリプト(以降のタスクで自作ノード起動を追記していく)。AMCLが収束しNav2が使える状態

- [ ] **Step 1: initialpose配信ノードを実装**

`object_patrol/object_patrol/set_initial_pose.py`:
```python
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class SetInitialPose(Node):
    """スポーン座標をAMCL初期位置として一定回数配信して終了する。"""

    def __init__(self):
        super().__init__('set_initial_pose')
        self.declare_parameter('x', -3.0)
        self.declare_parameter('y', 2.0)
        self.declare_parameter('yaw', 0.0)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.count = 0
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.get_parameter('x').value
        msg.pose.pose.position.y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.07
        self.pub.publish(msg)
        self.count += 1
        if self.count >= 5:
            raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(SetInitialPose())
    except SystemExit:
        pass
```

- [ ] **Step 2: 起動スクリプトを書く**

`scripts/run_object_patrol.sh`(骨子。前回の`run_sealed_mapping.sh`の構成を踏襲):
```bash
#!/bin/bash
set -e
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/fastdds_wsl.xml
export TURTLEBOT3_MODEL=waffle_pi
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
MAP=${MAP:-~/ros2_ws/maps/full_house_map.yaml}   # 前回地図の実ファイル名に合わせる
WORLD=${WORLD:-~/ros2_ws/worlds/object_house.world}  # Task 9で作成。それまではTB3ハウス
X=${X:--3.0}; Y=${Y:-2.0}

ros2 launch turtlebot3_gazebo empty_world.launch.py &   # 実際はworld指定起動に置換
sleep 15
# spawn(前回同様リトライ3回)
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$MAP &
sleep 20
ros2 run object_patrol set_initial_pose --ros-args -p x:=$X -p y:=$Y
# ここに後続タスクで yolo_detector / object_mapper / patrol_node 起動を追記
wait
```
注: Gazebo起動部は前回スクリプトの該当行(world指定・spawnリトライ)をコピーして合わせること。

- [ ] **Step 3: 走行確認**

スクリプト起動後、RVizでAMCLの自己位置が地図と一致していることを確認し:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```
Expected: ロボットが指定点まで走行しSUCCEEDED。AMCLパーティクルが収束している

- [ ] **Step 4: コミット**(`feat: add known-map navigation bringup (map_server+AMCL+Nav2)`)

---

### Task 7: patrol_node(巡回+見回し)

**Files:**
- Create: `object_patrol/object_patrol/patrol_node.py`、`config/patrol_points.yaml`、setup.py entry_points追記

**Interfaces:**
- Consumes: NavigateToPoseアクション(Nav2)、`config/patrol_points.yaml`
- Produces: 巡回完了時に `/object_mapper/finalize`(std_srvs/Trigger)を呼ぶ。`/cmd_vel` で360°回転

- [ ] **Step 1: waypoint定義**

`config/patrol_points.yaml`(座標は前回地図をRVizで見て各部屋中央に設定。以下は雛形、Task 9で実座標に調整):
```yaml
patrol_points:
  - {name: living,   x: -3.0, y: 2.0,  yaw: 0.0}
  - {name: kitchen,  x: -1.0, y: 3.0,  yaw: 0.0}
  - {name: east1,    x: 2.5,  y: 2.0,  yaw: 0.0}
  - {name: east2,    x: 4.0,  y: -1.0, yaw: 0.0}
  - {name: south,    x: 0.0,  y: -2.5, yaw: 0.0}
```

- [ ] **Step 2: 実装**

`object_patrol/object_patrol/patrol_node.py`:
```python
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class PatrolNode(Node):
    """waypointを順に巡回し、各点で360度見回す。完了後にobject_mapperへ集計を指示。"""

    SPIN_SPEED = 0.5  # rad/s

    def __init__(self):
        super().__init__('patrol_node')
        self.declare_parameter('points_file', 'patrol_points.yaml')
        with open(self.get_parameter('points_file').value) as f:
            self.points = yaml.safe_load(f)['patrol_points']
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.finalize_cli = self.create_client(Trigger, '/object_mapper/finalize')

    def run(self):
        self.nav.wait_for_server()
        for p in self.points:
            ok = self.goto(p) or self.goto(p)  # リトライ1回
            if not ok:
                self.get_logger().warn(f"skip waypoint {p['name']} (aborted twice)")
                continue
            self.get_logger().info(f"arrived {p['name']}, spinning")
            self.spin_around()
        self.get_logger().info('patrol finished, requesting finalize')
        if self.finalize_cli.wait_for_service(timeout_sec=10.0):
            fut = self.finalize_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            if fut.done():
                self.get_logger().info(f'finalize: {fut.result().message}')

    def goto(self, p):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(p['x'])
        goal.pose.pose.position.y = float(p['y'])
        goal.pose.pose.orientation.z = math.sin(p['yaw'] / 2)
        goal.pose.pose.orientation.w = math.cos(p['yaw'] / 2)
        fut = self.nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if not handle.accepted:
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().status == 4  # STATUS_SUCCEEDED

    def spin_around(self):
        """cmd_velで360度その場回転(検出のため低速)。"""
        twist = Twist()
        twist.angular.z = self.SPIN_SPEED
        duration = 2 * math.pi / self.SPIN_SPEED
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)
        self.cmd_pub.publish(Twist())  # 停止


def main():
    rclpy.init()
    node = PatrolNode()
    node.run()
    rclpy.shutdown()
```

- [ ] **Step 3: 動作確認** — Task 6の環境+patrol_node起動。Expected: 全waypointを順に走行し各点で一回転、ABORT時はスキップのログ。`/object_mapper/finalize`が未提供でもタイムアウトで正常終了すること
- [ ] **Step 4: 起動スクリプトにpatrol_node起動を追記(コメントアウト状態で用意、Task 9で有効化)し、コミット**(`feat: add patrol_node with waypoint loop and spin`)

---

### Task 8: object_mapperノード(フュージョン・記録・出力)

**Files:**
- Create: `object_patrol/object_patrol/object_mapper.py`、setup.py entry_points追記
- Modify: `object_patrol/package.xml`(depend追記: tf2_ros, tf2_geometry_msgs, visualization_msgs, std_srvs, nav_msgs)

**Interfaces:**
- Consumes: `/detected_objects`(Task 5)、`/scan`、TF(map←base_link)、`fusion.py`(Task 3)、`ObjectRegistry`(Task 4)
- Produces: `/object_markers`(visualization_msgs/MarkerArray)、`/object_mapper/finalize`(std_srvs/Trigger)→ `~/ros2_ws/object_report/report.md` と `object_map.png` を出力

- [ ] **Step 1: 実装**

`object_patrol/object_patrol/object_mapper.py`:
```python
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
import tf2_ros
import tf2_geometry_msgs  # PointStamped変換の登録に必要(import自体が登録処理)
from object_patrol_msgs.msg import DetectedObjects
from object_patrol.fusion import pixel_to_bearing, scan_range_at, bearing_range_to_xy
from object_patrol.registry import ObjectRegistry

HFOV = 1.085595
IMG_WIDTH = 640


class ObjectMapper(Node):
    def __init__(self):
        super().__init__('object_mapper')
        self.declare_parameter('report_dir', os.path.expanduser('~/ros2_ws/object_report'))
        self.registry = ObjectRegistry(merge_dist=0.5, confirm_frames=3)
        self.last_scan = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 5)
        self.create_subscription(DetectedObjects, '/detected_objects', self.det_cb, 5)
        self.marker_pub = self.create_publisher(MarkerArray, '/object_markers', 1)
        self.create_service(Trigger, '/object_mapper/finalize', self.finalize_cb)
        self.get_logger().info('object_mapper ready')

    def scan_cb(self, msg):
        self.last_scan = msg

    def det_cb(self, msg):
        if self.last_scan is None:
            return
        scan = self.last_scan
        for d in msg.objects:
            bearing = pixel_to_bearing((d.x1 + d.x2) / 2.0, IMG_WIDTH, HFOV)
            rng = scan_range_at(list(scan.ranges), scan.angle_min,
                                scan.angle_increment, bearing)
            if rng is None or rng > 5.0:
                continue  # 遠すぎる/測れない検出は捨てる(壁越し誤登録防止)
            x, y = bearing_range_to_xy(bearing, rng)
            pt = PointStamped()
            pt.header.frame_id = 'base_link'
            pt.header.stamp = msg.header.stamp  # 画像時刻のTFで変換(回転中のずれ防止)
            pt.point.x, pt.point.y = x, y
            try:
                mp = self.tf_buffer.transform(pt, 'map',
                                              timeout=Duration(seconds=0.2))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                continue
            o = self.registry.observe(d.class_name, mp.point.x, mp.point.y)
            if o['confirmed']:
                self.publish_markers()

    def publish_markers(self):
        arr = MarkerArray()
        for i, o in enumerate(self.registry.confirmed()):
            m = Marker()
            m.header.frame_id = 'map'
            m.id = i
            m.type = Marker.CYLINDER
            m.pose.position.x, m.pose.position.y = o['x'], o['y']
            m.pose.position.z = 0.3
            m.scale.x = m.scale.y = 0.3
            m.scale.z = 0.6
            m.color.r, m.color.g, m.color.a = 1.0, 0.3, 0.9
            arr.markers.append(m)
            t = Marker()
            t.header.frame_id = 'map'
            t.id = 100 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.text = o['class_name']
            t.pose.position.x, t.pose.position.y, t.pose.position.z = o['x'], o['y'], 0.9
            t.scale.z = 0.3
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            arr.markers.append(t)
        self.marker_pub.publish(arr)

    def finalize_cb(self, req, res):
        outdir = self.get_parameter('report_dir').value
        os.makedirs(outdir, exist_ok=True)
        objs = self.registry.confirmed()
        lines = ['# 物体探索レポート', '', f'発見物体: {len(objs)}個', '',
                 '| クラス | x [m] | y [m] | 観測回数 |', '|---|---|---|---|']
        for o in objs:
            lines.append(f"| {o['class_name']} | {o['x']:.2f} | {o['y']:.2f} | {o['count']} |")
        with open(os.path.join(outdir, 'report.md'), 'w') as f:
            f.write('\n'.join(lines) + '\n')
        self.draw_object_map(outdir, objs)
        res.success = True
        res.message = f'{len(objs)} objects reported to {outdir}'
        return res

    def draw_object_map(self, outdir, objs):
        """地図PGMに発見物体を描き込んだPNGを出力する。"""
        import cv2
        import yaml as pyyaml
        map_yaml = os.path.expanduser(os.environ.get('MAP', '~/ros2_ws/maps/full_house_map.yaml'))
        with open(map_yaml) as f:
            info = pyyaml.safe_load(f)
        img = cv2.imread(os.path.join(os.path.dirname(map_yaml), info['image']))
        resu = info['resolution']
        ox, oy = info['origin'][0], info['origin'][1]
        h = img.shape[0]
        for o in objs:
            px = int((o['x'] - ox) / resu)
            py = h - int((o['y'] - oy) / resu)
            cv2.circle(img, (px, py), 6, (0, 0, 255), -1)
            cv2.putText(img, o['class_name'], (px + 8, py + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        cv2.imwrite(os.path.join(outdir, 'object_map.png'), img)


def main():
    rclpy.init()
    rclpy.spin(ObjectMapper())
```

注意: 実装時に前回地図の実ファイル名・カメラ解像度(`ros2 topic echo /camera/image_raw --field width --once`)・カメラ光軸とbase_linkのオフセット(waffle_piのカメラは前方約7cm。bearing原点として無視できる誤差か確認し、必要なら`camera_rgb_frame`をTF起点にする)を確認して合わせること。

- [ ] **Step 2: 単体動作確認** — Task 6環境+yolo_detector+object_mapperを起動し、ロボット前方に物体を置いてRVizで`/object_markers`が物体の実位置(Gazebo上の配置座標)から0.5m以内に出ることを確認。`ros2 service call /object_mapper/finalize std_srvs/srv/Trigger` でreport.md/object_map.pngが生成されること
- [ ] **Step 3: コミット**(`feat: add object_mapper node with LiDAR fusion and reporting`)

---

### Task 9: 物体配置ワールドと統合実行・検証

**Files:**
- Create: `worlds/object_house.world`(TB3密閉ハウス+Task 2採用モデル6〜8個)
- Modify: `scripts/run_object_patrol.sh`(全ノード起動を有効化)、`config/patrol_points.yaml`(実座標調整)
- Create: `docs/evidence_object_patrol_*.png/mp4`、`docs/object_patrol_result.md`(検証結果)

**Interfaces:**
- Consumes: これまでの全成果物
- Produces: 検証済みシステム一式+エビデンス

- [ ] **Step 1: ワールド作成** — 前回の密閉ハウスworld(封鎖壁・ドアプラグ入り)をコピーし、Task 2の採用モデルを各部屋に計6〜8個 `<include>` で配置。**Task 2確定事項**: 採用モデルは person_standing(person 0.61-0.74) / person_walking(person 0.93) / fire_hydrant(fire hydrant 0.73) / stop_light(traffic light 0.91) / robocup_spl_ball(frisbee 0.65、CG誤分類だが安定のため正解クラス=frisbeeと定義)。**ワールドの<scene>にambient 0.9・shadows false を必ず設定**(暗いとfire_hydrantがtraffic light誤分類に落ちる等、全滅リスク)。不採用: stop_sign(メッシュ不解決)・chair/cafe_table(低視点で認識不可)・postbox/beer/oak_tree(弱)。**配置座標を一覧表(正解表)としてdocs/object_patrol_result.mdに記録**(LiDAR高さ16cmに実体がある置き方にする)
- [ ] **Step 2: waypoint実座標調整** — 地図をRVizで開き各部屋中央の座標を読んでpatrol_points.yamlを更新
- [ ] **Step 3: 統合実行** — `run_object_patrol.sh` で全系(Gazebo→Nav2→initialpose→yolo_detector→object_mapper→patrol_node)を起動し完走させる
- [ ] **Step 4: 検証** — report.mdの発見物体を正解表と突き合わせ:
  - 発見率 = 発見数/配置数 ≥ 80%
  - 位置誤差 = 各発見物体と正解座標の距離 ≤ 0.5m
  - 誤検出(正解表にない登録)の数を記録(目標0)
  - 結果(達成/未達とも)をdocs/object_patrol_result.mdに記録。未達なら原因分析して調整・再実行(しきい値・window_deg・waypoint追加。細足物体でLiDAR距離が壁距離になっているのが原因の場合は、設計書§4③の床平面仮定フォールバック — バウンディングボックス下端ピクセルとカメラ高さから距離を推定し、θ方向LiDAR距離と大きく食い違うときにそちらを採用 — をfusion.pyに追加実装する)
- [ ] **Step 5: エビデンス取得** — 前回のPowerShellキャプチャ+ffmpegパイプラインで、RViz(マーカー出現の様子)とGazeboの動画・静止画、最終object_map.pngを保存
- [ ] **Step 6: README更新とコミット** — README.mdにobject_patrolの節を追加し、`feat: add object world, integrated run and verification results` でコミット

---

## 実行メモ(全タスク共通)

- ビルド後は必ず `source ~/ros2_ws/install/setup.bash` を忘れない
- yolo_detectorだけはvenvのpythonで実行(`~/ros2_ws/yolo_venv/bin/python -m object_patrol.yolo_detector`)。他ノードは通常の`ros2 run`
- Gazebo/RVizが黒画面・不調になったらWSL再起動(前回の教訓: 長時間セッションのWSLg劣化)
- pytestはROS環境なしで実行可能(fusion.py/registry.pyは純粋Python)

# ソースコード読解ガイド — auto_explorer.py(396行)

ROS 2の主要概念(ノード・トピック・サービス・アクション・QoS・spin)が
1ファイルに全部登場する構成です。行番号は `auto_mapper/auto_mapper/auto_explorer.py` 基準。

## 1. 骨格

```
class AutoExplorer(Node)          # ROSノードの標準形: Nodeを継承
├── __init__       (L20-45)  ROSとの配線を全部宣言
├── on_map 等      (L62-72)  受信コールバック(貯めるだけ)
├── ray_goal       (L74-101) 頭脳②: センサ空間の探索(レーダー直読)
├── pick_goal      (L111-220)頭脳①: 地図空間の探索(フロンティア) ★最重要
├── navigate       (L222-237)Nav2アクションで移動
├── save_map       (L297-308)サービスで地図保存
└── explore        (L310-380)メインループ(指揮者)
main()             (L383-392)起動・終了の定型文
```

`main()` は `rclpy.init()` → ノード生成 → `explore()` → `destroy_node()/shutdown()` の
定型文。`setup.py` の `entry_points` がこの `main` を `ros2 run auto_mapper auto_explorer`
に結びつけている。

## 2. __init__ = 配線盤(ROS通信3方式が全部ある)

| 行 | コード | 方式 | 意味 |
|---|---|---|---|
| L22-32 | `declare_parameter(...)` | パラメータ | `ros2 run ... -p min_cluster:=4` で外から上書きできる設定値 |
| L38 | `create_subscription(OccupancyGrid,'/map',...,1)` | トピック購読 | SLAMの地図。深さ1=最新だけあればよい |
| L39 | `create_subscription(Odometry,'/odom',...,10)` | トピック購読 | 自己位置(車輪積算) |
| L41 | `create_subscription(LaserScan,'/scan', qos_profile_sensor_data)` | トピック購読+QoS | センサー用QoS(BEST_EFFORT)。「欠けてもいいから最新を速く」 |
| L43 | `ActionClient(self, NavigateToPose,'navigate_to_pose')` | **アクション** | 長い仕事(移動)の依頼。受理→経過→結果の3段階 |
| L44 | `create_client(SaveMap,'/slam_toolbox/save_map')` | **サービス** | 一問一答(保存して→OK) |
| L45 | `create_publisher(Twist,'/cmd_vel',10)` | トピック配信 | 放浪モードで車輪を直接回す |

覚え方: **トピック=垂れ流し / サービス=一問一答 / アクション=長い依頼**。

## 3. spinの使い方 — 「能動型」ノード

`rclpy.spin(node)`(永久受信待ち)は使わない。このノードは自分のループが主導権を持ち、
**必要な時だけ受信処理を回す**:

- `spin_for(sec)` (L103): 指定秒だけ `spin_once` を回す = その間だけコールバックが動く
- `spin_until_future_complete` (L229, L305): サービス/アクションの返事待ち
- navigate中も `spin_once` を回し続ける (L236) — 受信処理を止めるとNav2からの
  結果通知も受け取れないため

コールバック(L62-72)は `self.map_msg = m` のように**貯めるだけ**。判断は全部
`explore()` に集中させる(受信と思考の分離)。

## 4. OccupancyGrid の読み方(pick_goal, L111-220)

- 地図は**1次元配列** `data`。セル(cx,cy)へのアクセスは `data[cy * w + cx]` (L127)
- 値の意味: **-1=未知 / 0=床 / 100=壁**(0〜100は占有確率)
- セル→世界座標: `wx = origin.x + cx * resolution` (L198)。解像度0.05m/セル
- 世界→セルは逆算 (L210-211)

### フロンティア探索の流れ
1. **列挙** (L133-139): 床セルで上下左右に未知(-1)が隣接するもの = フロンティア
2. **クラスタリング** (L144-161): BFS(8近傍)で連結成分に分割、`min_cluster` 未満は
   ノイズとして捨てる(狭ドア縁の偽フロンティア対策)
3. **ゴール決定** (L167-219):
   - クラスタ重心に最も近い**境界セル**を選ぶ(重心そのものは未知空間に落ちることがある)
   - `clear()` でクリアランス(周囲±5セル=0.25mが床)を確認、ダメなら境界セルの
     周囲を渦巻き状に探索 (L185-195)
   - **0.7m押し込み** (L199-213): ゴールを境界の0.7m先(未知側)へ押す。
     Nav2は `track_unknown_space:=false` で未知へも計画できるので、ロボットが
     物理的にドアを越えて奥の部屋が見える。境界上ゴールだとドア枠で止まる
4. **スコア** (L217): `クラスタの大きさ / (1 + 距離)` — 大きくて近い縁を優先

## 5. ray_goal — センサ空間のフォールバック(L74-101)

地図上のフロンティアが尽きても、LiDARが**素通り**する方向(=まだ壁を見つけて
いない方向)が残ることがある。

- 無限大(無エコー)のレイは `range_max` (3.5m) に丸める (L86-87)
- 5レイ窓の**最小値**で平滑化(1本だけ長い=ノイズを弾く)し、最遠方向を選ぶ (L89-92)
- `min(最遠-0.5, 2.0)` m だけその方向へ前進するゴールを作る (L96-98)
- 最遠でも1.2m未満なら「見るべき方向なし」でNone (L93)

角度計算 (L95): `scan.angle_min + i*angle_increment` はロボット座標系なので
`+ self.yaw` で世界座標系に直す。yawはquaternionから逆算 (L68-69)。

## 6. navigate — アクションの教科書(L222-237)

```python
fut = self.nav.send_goal_async(goal)      # ① 依頼を送る
handle = fut.result()                     # ② 受理された? (accepted)
rf = handle.get_result_async()            # ③ 結果を待つ
rf.result().status                        # 4 = SUCCEEDED
```

ゴールは `frame_id='map'`(地図座標系)で送る。`orientation.w=1.0` は
「向きは問わない(正面向きでよい)」の定型。

## 7. explore — メインループ(L310-380)

```
ブートストラップ待ち (L322-324): /scanと/mapが届くまで最大60秒
  └ DDSの発見遅延で起動直後は何も届かない。届く前に判定すると即「完成」誤判定

for i in 1..max_goals:
    spin_for(3)                      # 最新の地図を取り込む
    goal = pick_goal(blacklist)      # ① フロンティア
    goal = goal or ray_goal(...)     # ② レーダー直読
    if goal is None:
        wanderバースト (L343-361)     # ③ 盲目前進×3
        成長<60 が2回連続 → 完成 → break
    navigate(goal)
    if status != 4 or 成長 < 30:      # 成長ベースブラックリスト (L377)
        blacklist.append(goal)
save_map()                           # サービスで保存 (L380)
```

wanderの存在理由 (L340-342のコメント): LiDAR射程内に反射物がない大空間は
SLAMが遠隔から描けない(無エコーのレイは破棄される)。**実際に走るしかない**。

## 8. 実機との関係

このファイルにGazebo専用のコードは1行もない。`/scan` `/odom` `/map` `/cmd_vel` と
NavigateToPose/SaveMapという**標準インターフェースだけ**に依存しているので、
実機TurtleBot3に載せ替えてもコードは無変更で動く(仮想と実機の境界は
Gazeboプラグインの一枚だけ)。

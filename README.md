# restaurant_delivery

ROS 2 (Humble) + Gazebo 環境向けの**ファミレス配膳ロボット自律走行・障害物飛び出し回避＆迂回制御シミュレーション**パッケージです。

---

## 1. 概要とデモシナリオ

レストランやファミレス内の配膳業務において、予測不能に駆け回る子どもやお客様、障害物による通路閉塞を安全に回避・迂回して配膳を完遂する安全・自律移動システムです。

### 2幕構成のストーリーボード
1. **第1幕: 駆け回る男の子の飛び出し緊急停止（0:00〜0:15）**
   - 厨房カウンター横（X=-1.2m）を出発後、通路横から駆け出す男の子（running_boy）をLiDAR（ワイド検知コーン左右±50°）が早期検知。
   - ロボットは安全マージンを保ち手前でスムーズに**緊急停止（EMERGENCY STOP）**し、男の子の通過を確認後に自動再発進します。
2. **第2幕: 女の子による最短通路閉塞と自律迂回・配膳完了（0:15〜1:10）**
   - 客席から降りて中央通路（X=2.8m）で横たわって通路を塞ぐ女の子（rolling_girl）を前方で検知。
   - 接近しすぎずに手前（X=1.3m）で安全停止し、自律的に**上側迂回ルート（Y=1.8m）**へ進路変更。
   - テーブルとの離隔マージンを確保しながら迂回路を抜け、目標客席テーブル（X=4.2m）へスムーズにアプローチして**配膳完了（オムライスの提供）**します。

---

## 2. 通信・アーキテクチャ設計

### ノード構成
```text
[ Gazebo Camera / LiDAR Sensor (/scan) ]
                │
                ▼
+---------------------+                      +-----------------------+
|  obstacle_monitor   | -- /emergency_stop ->|  delivery_controller  | --> [ /cmd_vel ]
| (ワイド検知・警報発報) | -- /obstacle_alert ->| (配膳制御・自律迂回)  |
+---------------------+                      +-----------------------+
                                                         ▲
                                                         │
                                                  [ /odom (台車位置) ]
```

### トピック一覧
| トピック名 | 型 | 送信元 | 受信先 | 説明 |
| :--- | :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo / LiDAR | `obstacle_monitor` | 周囲障害物との距離データ |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo / 台車 | `delivery_controller` | 現在の自己位置・姿勢情報 |
| `/emergency_stop` | `std_msgs/msg/Bool` | `obstacle_monitor` | `delivery_controller` | 緊急停止フラグ (飛び出し時: True) |
| `/obstacle_alert` | `std_msgs/msg/String` | `obstacle_monitor` | UI / システム | 音声アナウンス・ディスプレイ用警告文 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `delivery_controller` | Gazebo / 駆動輪 | ロボット台車への速度・角速度指令 |
| `/robot_status` | `std_msgs/msg/String` | `delivery_controller` | `debug_recorder` | ロボットの現在進行フェーズ情報 |

---

## 3. ディレクトリ構成
```text
restaurant_delivery/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── restaurant_delivery
├── urdf/
│   └── delivery_robot.urdf      # 差動二輪配膳台車モデル（LiDAR & カメラ & 差動プラグイン）
├── models/                      # 子どもモデル（SDF）
├── worlds/
│   └── restaurant.world         # レストラン環境（床面、客席テーブル、仕切り、厨房カウンター）
├── restaurant_delivery/
│   ├── __init__.py
│   ├── delivery_controller.py   # 配膳制御・一時停止・自律迂回ステートマシン
│   ├── obstacle_monitor.py      # 前方ワイド障害物監視・飛び出し警報発報ノード
│   ├── child_spawner.py         # 子どもの動的飛び出し・寝転がりアニメーション制御
│   └── debug_recorder.py        # 連番キャプチャ・コンタクトシート・GIF・MP4自動生成ノード
├── launch/
│   └── delivery_demo.launch.py  # Gazebo + ロボット + ノード一括起動Launch
├── debug_output/                # 実行ログ・コンタクトシート・動画出力先
└── README.md
```

---

## 4. ビルドと実行方法

### ワークスペースでのビルド
```bash
cd ~/pai_ws
colcon build --symlink-install --packages-select restaurant_delivery
source install/setup.bash
```

### Gazebo 統合シミュレーションの起動
Gazebo の起動、配膳ロボットのスポーン、客席テーブル配置、子どもの飛び出し、監視・配膳迂回制御およびレコーダーノードが一括起動します：
```bash
ros2 launch restaurant_delivery delivery_demo.launch.py
```

シミュレーション終了（Ctrl+C）時、`debug_output/` 内に以下が自動生成されます：
- `simulation_demo.mp4`: デモ動画（MP4形式）
- `simulation_debug.gif`: デモGIFアニメーション
- `contact_sheet.jpg`: シナリオ一覧コンタクトシート
- `execution_log.txt`: 実行ログ

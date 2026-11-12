# ROS 2 Humble 移植チェックシート (TODO 版)

このチェックシートは、現在の Jazzy ベース実装を ROS 2 Humble 環境へ移植するための作業項目を TODO 化したものです。

## 使い方

- 各項目を上から順に実施
- `- [ ]` を `- [x]` に変更して進捗管理
- 各フェーズの「完了条件」を満たしたら次へ進む

---

## Phase 0: 前提準備

- [ ] Ubuntu + ROS 2 Humble 環境を用意した
- [ ] 新規ワークスペース `~/ros2_ws_humble` を作成した
- [ ] `amr_navigator_autonomy` を `src/amr_navigator` として clone した

```bash
mkdir -p ~/ros2_ws_humble/src
cd ~/ros2_ws_humble/src
git clone -b develop https://github.com/megane312/amr_navigator_autonomy.git amr_navigator
```

完了条件:
- `~/ros2_ws_humble/src/amr_navigator` が存在する

---

## Phase 1: 依存パッケージ整合

### 1-1. 依存リポジトリ取得

- [ ] `FAST_LIO` を Humble 対応ブランチ/タグで取得
- [ ] `livox_ros_driver2` を Humble 対応ブランチ/タグで取得
- [ ] `livox_mid360_mapping` を Humble 対応ブランチ/タグで取得
- [ ] `om_modbus_master` / `om_msgs` を Humble 対応版で取得

### 1-2. rosdep 依存解決

- [ ] rosdep install がエラーなく完了

```bash
cd ~/ros2_ws_humble
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

完了条件:
- rosdep が unresolved dependency なしで完了

---

## Phase 2: 初回ビルドと差分抽出

- [ ] `colcon build --symlink-install` を実行
- [ ] ビルドエラーをファイル/行単位で記録
- [ ] エラーをカテゴリ分けした
  - [ ] CMake/ament 差分
  - [ ] Python 依存差分
  - [ ] API/メッセージ差分

```bash
cd ~/ros2_ws_humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

完了条件:
- ビルドエラーが一覧化され、対応順が決まっている

---

## Phase 3: Python 差分対応 (3.10)

- [ ] Python 実行バージョンを確認 (`python3 --version`)
- [ ] `scripts/*.py` の依存 import を確認
- [ ] Humble で不足する Python パッケージを補った
- [ ] `python3 -m py_compile` で主要スクリプト検証

対象スクリプト:

- [ ] `scripts/om_odom_tf.py`
- [ ] `scripts/joy_idshare_drive.py`
- [ ] `scripts/waypoint_mission.py`
- [ ] `scripts/waypoint_follower.py`
- [ ] `scripts/obstacle_stop_monitor.py`

```bash
python3 -m py_compile src/amr_navigator/scripts/om_odom_tf.py
python3 -m py_compile src/amr_navigator/scripts/waypoint_mission.py
python3 -m py_compile src/amr_navigator/scripts/waypoint_follower.py
python3 -m py_compile src/amr_navigator/scripts/obstacle_stop_monitor.py
```

完了条件:
- 主要 Python ノードが文法エラーなし

---

## Phase 4: RViz/Launch 差分対応

- [ ] RViz 設定ファイルのプラグイン名を確認
- [ ] `rviz_default_plugins/*` 前提で読み込みできることを確認
- [ ] launch 引数の型不一致を解消
- [ ] `ros2 launch` で即死しないことを確認

対象 launch:

- [ ] `launch/record_waypoints.launch.py`
- [ ] `launch/replay_waypoints.launch.py`
- [ ] `launch/follow_waypoints.launch.py`

完了条件:
- 3本の launch が起動し、主要ノードが生存

---

## Phase 5: 記録・再生の基本検証

### 5-1. 記録

- [ ] `record_waypoints.launch.py` 起動
- [ ] 手動走行後に `Saved waypoint #...` が出る
- [ ] `recorded_waypoints.txt` に行が追記される

### 5-2. 再生

- [ ] `replay_waypoints.launch.py` 起動
- [ ] `/waypoints` が publish される
- [ ] RViz に Path が表示される

完了条件:
- 1本以上の waypoint を記録でき、再生表示できる

---

## Phase 6: 自律追従 + 安全層検証

- [ ] `follow_waypoints.launch.py` 起動
- [ ] waypoint 追従が開始される
- [ ] 障害物投入で stop が立つ
- [ ] 障害物解除で同一 waypoint から再開する

確認トピック:

- [ ] `/waypoint_safety/stop`
- [ ] `/waypoint_safety/obstacle_count`
- [ ] `/waypoint_follower/status`
- [ ] `/waypoint_target_marker`

完了条件:
- 停止/再開の一連挙動が再現できる

---

## Phase 7: パラメータ・運用調整

- [ ] `topic_id` / `global_id` を実機に合わせた
- [ ] `expected_slave_id` の整合を確認した
- [ ] obstacle box (`forward_range`, `lateral_range`, `z_min`, `z_max`) を調整
- [ ] stop/resume ヒステリシス (`stop_count`, `resume_count`) を調整

完了条件:
- 現場環境で誤停止が許容範囲

---

## Phase 8: 受入れ判定

- [ ] コールドスタートから運用までの手順書化が完了
- [ ] 記録→再生→追従→停止/再開の全手順を1回通した
- [ ] ログ/既知課題/暫定回避策を記録した

完了条件:
- Humble 環境での運用可否を明確に判定できる

---

## 既知の注意点

- Ctrl-C 時に `rcl_shutdown already called` 系の traceback が出ることがある
- LiDAR/モータ関連ノードの終了時に ASan ログが出る場合がある
- これらは「運転中の機能不全」とは分けて評価すること

---

## 参考ドキュメント

- `docs/new_pc_quickstart_checklist.md`
- `docs/autonomous_navigation_deployment_guide.md`
- `docs/manual_drive_setup.md`

# sz_indoor_fsm 使用说明

`sz_indoor_fsm` 提供一套 ROS1 Python 协同任务状态机，面向 GCS、MASTER、SLAVE 三种角色。

核心文件：

- `scripts/fsm.py`：实际任务状态机。
- `scripts/fsm_monitor.py`：Rich 终端 GCS 操作台。
- `scripts/fsm_auto_test.py`：自动/半自动集成测试驱动。
- `srv/JsonCommand.srv`：所有 FSM service 使用的 JSON 命令接口。
- `README_traj_router.md`：给 traj_router 同事看的 service/JSON 对接说明。
- `launch/coop_fsm.launch`：单节点 FSM/monitor 启动模板。
- `launch/coop_fsm_auto_test.launch`：本地测试流程。

## 状态流程

GCS:

```text
PREPARE
  -> WAIT_TAKEOFF
  -> TAKEOFF_RUNNING
  -> TRAJ_FOLLOWING
  -> LAND
  -> PREPARE
```

`STOP` 仍保留为兼容状态：如果 traj_router 主动上报 `stop`，GCS 可以进入 `STOP`。
但正常联调流程不再等待 `stop`；在 `TRAJ_FOLLOWING` 中按降落键或触发降落 RC 即可直接进入 `LAND`。

MASTER / SLAVE:

```text
PREPARE
  -> TAKEOFF_RUNNING
  -> ACHIEVE
  -> WAIT
  -> TRAJ_FOLLOWING
  -> LAND
  -> PREPARE
```

任意角色收到 ABORT 后进入：

```text
ABORT
```

`ABORT` 默认不自动复位，避免强制降落流程被误触发后又回到普通任务。

## 状态切换条件与调用链

主循环入口是 `CoopFSM.spin()`，每个周期按下面顺序执行：

1. `_keyboard_commands()` 读取键盘输入。
2. `_manual_abort_requested()` 检查 operator/键盘/RC 的 ABORT。
3. `_run_state_action_hook()` 执行当前状态的 `on_tick_*()` 钩子。
4. GCS 执行 `_tick_gcs(keyboard_commands)`，UAV 执行 `_tick_uav()`。
5. `_publish_status()` 发布状态 JSON。

### GCS 状态切换

GCS 的主动切换逻辑集中在 `_tick_gcs()`：

| 当前状态 | 切换条件 | 下一个状态 | 关键函数调用 |
| --- | --- | --- | --- |
| `PREPARE` | GCS 不做本地 `quadrotor_state` 自检，进入等待 UAV 准备状态 | `WAIT_TAKEOFF` | `_set_state(WAIT_TAKEOFF, "gcs_skip_prepare_check")` |
| `WAIT_TAKEOFF` | `_all_prepared()` 为真，且 `_manual_takeoff_requested(keyboard_commands)` 为真 | `TAKEOFF_RUNNING` | `_start_takeoff_from_gcs()` |
| `TAKEOFF_RUNNING` | `_all_achieved()` 为真 | `TRAJ_FOLLOWING` | `_start_traj_following_from_gcs()` |
| `TRAJ_FOLLOWING` | `_manual_land_requested(keyboard_commands)` 为真 | `LAND` | `_start_land_from_gcs()` |
| `TRAJ_FOLLOWING` | GCS 的 event service 收到 `stop`，且当前状态仍是 `TRAJ_FOLLOWING` | `STOP` | `_event_service_cb()` -> `_set_state(STOP, "traj_router_stop")` |
| `STOP` | `_manual_land_requested(keyboard_commands)` 为真 | `LAND` | `_start_land_from_gcs()` |
| `LAND` | `now_sec() - land_start_time >= land_reset_delay` | `PREPARE` | `_reset_for_next_round("land_reset_delay")` |

`_start_takeoff_from_gcs()` 会先 `_set_state(TAKEOFF_RUNNING, "manual_takeoff_confirmed")`，
再调用 `_send_to_traj_router("takeoff", payload)`。如果 `send_takeoff_started_to_uavs`
为真，还会 `_broadcast_to_uavs("takeoff_started", payload)`。

`_start_traj_following_from_gcs()` 会调用：

```python
payload = {"participants": self.participants, "traj_type": self.traj_type}
_send_to_traj_router("traj_following", payload)
_broadcast_to_uavs("traj_following", payload)
_set_state(TRAJ_FOLLOWING, "all_uavs_achieved")
```

`traj_type` 默认是 `1`，可通过 launch 参数修改为 `2`、`3` 等。traj_router 可据此选择不同主轨迹。

`_start_land_from_gcs()` 会调用：

```python
_send_to_traj_router("land", payload)
_broadcast_to_uavs("land", payload)
_set_state(LAND, "manual_land_confirmed")
```

### UAV 状态切换

MASTER 和 SLAVE 共用 `_tick_uav()`：

| 当前状态 | 切换条件 | 下一个状态 | 关键函数调用 |
| --- | --- | --- | --- |
| `PREPARE` | `_health_ok()` 为真，且 `_send_gcs_event("prepared", ...)` 成功 | `TAKEOFF_RUNNING` | `_set_state(TAKEOFF_RUNNING, "self_check_passed")` |
| `TAKEOFF_RUNNING` | `_takeoff_reached()` 为真 | `ACHIEVE` | `_set_state(ACHIEVE, "takeoff_reached")` |
| `ACHIEVE` | `_send_gcs_event("achieve", ...)` 成功 | `WAIT` | `_set_state(WAIT, "achieve_reported")` |
| `WAIT` | GCS 下发 `traj_following` 命令 | `TRAJ_FOLLOWING` | `_command_service_cb()` -> `_set_state(TRAJ_FOLLOWING, "gcs_traj_following")` |
| `TRAJ_FOLLOWING` | GCS 下发 `land` 命令 | `LAND` | `_command_service_cb()` -> `_set_state(LAND, "gcs_land")` |
| `LAND` | `now_sec() - land_start_time >= land_reset_delay` | `PREPARE` | `_reset_for_next_round("land_reset_delay")` |

`_health_ok()` 同时要求：

- 最近收到过 `quadrotor_state`，且未超过 `health_timeout`。
- `_health_rate() >= health_min_rate_hz`。
- `_uav_state_msg_valid(msg)` 通过。

`_takeoff_reached()` 默认接受两个条件之一：

- 当前高度 `data[2]` 与 `takeoff_height` 的差值不超过 `takeoff_reached_tolerance`。
- `takeoff_started` 后经过 `takeoff_duration + takeoff_timeout_margin`。

UAV 收到 GCS 的 `takeoff_started` 命令时，`_command_service_cb()` 会更新
`takeoff_started`、`takeoff_start_time`、`takeoff_height` 和 `takeoff_duration`。
如果当前状态是 `PREPARE` 或 `TAKEOFF_RUNNING`，还会调用：

```python
_set_state(TAKEOFF_RUNNING, "gcs_takeoff_started")
```

### prepared / achieve / stop 事件

这些事件从 UAV 或 traj_router 通过 `/gcs/fsm/event` 进入 GCS，对应回调是
`_event_service_cb()`：

| event | 作用 | 是否立即切状态 |
| --- | --- | --- |
| `prepared` | 将 `uav_id` 加入 `prepared_uavs` | 不立即切换；下一次 `_tick_gcs()` 用 `_all_prepared()` 判断 |
| `achieve` | 将 `uav_id` 加入 `achieved_uavs` | 不立即切换；下一次 `_tick_gcs()` 用 `_all_achieved()` 判断 |
| `stop` | 保存 `last_stop_payload` | 兼容旧流程；如果 GCS 正在 `TRAJ_FOLLOWING`，立即切到 `STOP` |
| `abort` | 进入 ABORT 流程 | 立即调用 `_enter_abort(...)` |

### 操作员、键盘和 RC 输入

GCS 的手动起飞确认来自 `_manual_takeoff_requested(keyboard_commands)`：

- Rich monitor 调 `/gcs/fsm/operator` 发送 `takeoff`，置位 `operator_takeoff_requested`。
- 本地键盘按 `key_takeoff`，默认是 `t`。
- RC 起飞通道出现上升沿：`_rc_rising_edge("takeoff")`。

GCS 的手动降落确认来自 `_manual_land_requested(keyboard_commands)`：

- Rich monitor 发送 `land`，置位 `operator_land_requested`。
- 本地键盘按 `key_land`，默认是 `l`。
- RC 降落通道出现上升沿：`_rc_rising_edge("land")`。

当前逻辑中，GCS 在 `TRAJ_FOLLOWING` 或 `STOP` 状态收到手动降落确认，都会调用
`_start_land_from_gcs()`：先向 traj_router 发送 `land` service，再向所有 UAV FSM
广播 `land`，最后 GCS 自身进入 `LAND`。

ABORT 来自 `_manual_abort_requested(keyboard_commands)` 或外部 service：

- Rich monitor 发送 `abort`，置位 `operator_abort_requested`。
- 本地键盘按 `key_abort`，默认是 `a`。
- RC 中止通道满足阈值：`_abort_active()`。
- GCS event service 收到 `abort`。
- UAV command service 收到 `abort`。

### ABORT 与 reset

进入 ABORT 的统一入口是 `_enter_abort(reason, payload)`。它会先调用：

```python
_set_state(ABORT, reason)
```

`_set_state()` 在进入 `ABORT` 时会调用 `force_land(reason, {})`。`force_land()` 只对
UAV 生效，GCS 会直接返回。

如果当前节点是 GCS，`_enter_abort()` 还会调用：

```python
_send_to_traj_router("abort", abort_payload)
_broadcast_to_uavs("abort", abort_payload)
```

如果当前节点是 UAV，`_enter_abort()` 会调用：

```python
_send_gcs_event("abort", {"reason": reason})
```

`ABORT` 不会被定时器自动清除。需要重新开始时走 reset：

- GCS operator service 收到 `reset`：`_operator_service_cb()` -> `_reset_for_next_round("operator_reset")`。
- UAV command service 收到 `reset`：`_command_service_cb()` -> `_reset_for_next_round("remote_reset")`。

`_reset_for_next_round()` 会清空 `prepared_uavs`、`achieved_uavs`、`prepared_sent`、
`achieve_sent`、`takeoff_started`、`last_stop_payload`、`abort_reason` 等单轮任务状态，
然后调用：

```python
_set_state(PREPARE, reason)
```

## 按角色划分的职责

这套 FSM 的核心原则是：GCS 做全局编排和人工确认，UAV FSM 做本机状态判断，
traj_router 做轨迹分发和轨迹类型选择。调试 mock 只模拟接口，不代表真实飞控逻辑。

### GCS

GCS 负责全队任务编排，主要逻辑在 `CoopFSM._tick_gcs()`：

- 维护 `participants`、`prepared_uavs`、`achieved_uavs`。
- 判断是否全部 UAV 已 prepared：`_all_prepared()`。
- 判断是否全部 UAV 已 achieve：`_all_achieved()`。
- 接收 Rich monitor / 键盘 / RC 的起飞、降落、ABORT、reset 操作。
- 起飞确认满足后调用 `_start_takeoff_from_gcs()`。
- 所有 UAV achieve 后调用 `_start_traj_following_from_gcs()`。
- 运行轨迹期间收到降落触发后调用 `_start_land_from_gcs()`。
- ABORT 时调用 `_enter_abort()`，并向 traj_router 和所有 UAV 广播 `abort`。

GCS 不判断单机位置是否健康，也不计算轨迹点。GCS 切换到运行轨迹时，会把
`traj_type` 一起发给 traj_router：

```python
payload = {"participants": self.participants, "traj_type": self.traj_type}
_send_to_traj_router("traj_following", payload)
```

### MASTER / SLAVE UAV

MASTER 和 SLAVE 的单机 FSM 逻辑基本一致，主要逻辑在 `CoopFSM._tick_uav()`。
`master_id` 目前主要用于 GCS 侧默认监听哪一路 RC 输入，不改变单机 FSM 的判断方式。

- 监听本机 `/<uav_id>/quadrotor_state`。
- 检查状态话题是否新鲜、频率是否足够、数据格式是否有效：`_health_ok()`。
- 健康检查通过后向 GCS 上报 `prepared`。
- 收到 GCS `takeoff_started` 后进入 `TAKEOFF`。
- 判断本机是否到达起飞高度：`_takeoff_reached()`。
- 起飞完成后向 GCS 上报 `achieve`，并进入 `WAIT`。
- 收到 GCS `traj_following` 后进入 `TRAJ_FOLLOWING`。
- 收到 GCS `land` 后进入 `LAND`。
- 收到 ABORT 后进入 `ABORT`，并调用 `force_land()` 作为强制降落 hook。

当前 `quadrotor_state` 使用 `std_msgs/Float64MultiArray`，数据顺序是：

```text
[px, py, pz, vx, vy, vz, qw, qx, qy, qz]
```

速度、加速度和真实控制动作不由 FSM 自己计算。真实飞行接口应该接到
`trigger_takeoff()`、`trigger_traj_following()`、`trigger_land()`、`force_land()` 这些 hook。

### traj_router

traj_router 负责轨迹侧执行，主要入口是 `/traj_router/command`：

- 收到 `takeoff` 后执行起飞轨迹相关逻辑。
- 收到 `traj_following` 后读取 `traj_type`，选择并发布对应轨迹。
- 收到 `land` 后执行降落轨迹相关逻辑。
- 收到 `abort` 后停止当前轨迹或进入安全处理。

FSM 不直接选择轨迹文件或轨迹 topic 的细节，只把任务意图和 `traj_type` 发给
traj_router。正常流程中，进入 `TRAJ_FOLLOWING` 后不再等待 `stop` 指令；降落由 GCS
侧的键盘、RC 或 Rich monitor 操作触发。

### Rich monitor / operator

Rich monitor 是人工操作入口，不直接控制 UAV 或 traj_router。它只向 GCS 的
`/gcs/fsm/operator` 发送命令：

- `takeoff`：允许 GCS 从 `WAIT_TAKEOFF` 开始起飞。
- `land`：允许 GCS 在运行轨迹期间进入降落。
- `abort`：触发 GCS ABORT 流程。
- `reset`：清理本轮状态，回到 `PREPARE`。

### debug mock

debug mock 用于本地联调 GCS 和 traj_router。它模拟 UAV 侧接口：

- 发布 `/uav1/quadrotor_state` 到 `/uav4/quadrotor_state`。
- 提供 `/<uav_id>/fsm/command`，接收 GCS 广播的命令。
- 可自动发送 `prepared` 和 `achieve` 给 GCS。
- 可监听 GCS status，在 GCS reset 后清空 mock 的单轮状态。
- 可通过 `mock_publish_traj_flags:=true` 发布 `/<uav_id>/traj_generation_flag`。
- `mock_achieve_delay` 会影响从 `TAKEOFF` 到 `TRAJ_FOLLOWING` 的速度。

如果 debug launch 一进入 `TAKEOFF` 很快就进入 `TRAJ_FOLLOWING`，通常是因为
`mock_auto_achieve:=true` 且 `mock_achieve_delay` 比较短。这是 mock 行为，不代表真实无人机已经完成起飞判断。

### 判断归属速查

| 判断 / 操作 | 完成角色 | 主要函数 |
| --- | --- | --- |
| 单机状态话题是否健康 | UAV FSM | `_health_ok()` |
| 单机状态数据格式是否有效 | UAV FSM | `_uav_state_msg_valid()` |
| 单机是否到达起飞高度 | UAV FSM | `_takeoff_reached()` |
| 是否所有 UAV 已 prepared | GCS | `_all_prepared()` |
| 是否允许起飞 | GCS | `_manual_takeoff_requested()` |
| 是否所有 UAV 已 achieve | GCS | `_all_achieved()` |
| 是否进入轨迹跟踪 | GCS | `_start_traj_following_from_gcs()` |
| 选择第几条轨迹 | traj_router | `traj_type` 对应的处理逻辑 |
| 运行轨迹期间是否降落 | GCS | `_manual_land_requested()` |
| ABORT 广播 | GCS | `_enter_abort()` |
| 单机强制降落 hook | UAV FSM | `force_land()` |

## Service 与 Topic

### JsonCommand.srv

```srv
string json
---
bool success
string message
```

### 主要 service

- `/gcs/fsm/event`
  - UAV/traj_router -> GCS
  - 事件：`prepared`、`achieve`、`stop`、`abort`

- `/gcs/fsm/operator`
  - Rich 操作台 -> GCS
  - 命令：`takeoff`、`land`、`abort`、`reset`

- `/<uav_id>/fsm/command`
  - GCS -> UAV
  - 命令：`takeoff_started`、`traj_following`、`land`、`abort`、`reset`

- `/traj_router/command`
  - GCS -> traj_router
  - 命令：`takeoff`、`traj_following`、`land`、`abort`

### 主要 topic

- `/gcs/fsm/status`、`/<uav_id>/fsm/status`
  - `std_msgs/String`
  - JSON 状态，用于 Rich 操作台和 rosbag。

- `/gcs/fsm/service_audit`、`/<uav_id>/fsm/service_audit`
  - `std_msgs/String`
  - 每次 JSON service 收/发的审计记录。ROS1 rosbag 不能直接录 service，所以复盘 service 请录这些 topic。

- `/<uav_id>/quadrotor_state`
  - `std_msgs/Float64MultiArray`
  - UAV 自检和起飞高度判断输入。格式为 `[px, py, pz, vx, vy, vz, qw, qx, qy, qz]`。
    可用 `uav_state_topic:=...` 改成你的真实话题。

## 常用启动

默认起飞参数已经统一为：

```text
takeoff_height = 1.0 m
takeoff_duration = 10.0 s
```

### 实飞 GCS 侧

GCS 侧推荐启动 `sz_indoor_launch/launch/coop_gcs.launch`，它会启动 GCS FSM、
Rich 操作台、traj_router 和 trajectory_node：

```bash
source /home/fran/sz_indoor_transport_ws/devel/setup.zsh
roslaunch sz_indoor_launch coop_gcs.launch \
  self_id:=gcs \
  master_id:=uav1 \
  participants:=uav1,uav2,uav3,uav4 \
  run_id:=coop_lift_test_001 \
  traj_type:=1 \
  takeoff_height:=1.0 \
  takeoff_duration:=10.0 \
  launch_monitor:=true \
  node_output:=log
```

GCS 侧要特别注意：

- `participants` 必须写全本轮参与的 UAV，GCS 会等待这些 UAV 上报 `prepared/achieve`。
- `master_id` 是 GCS 默认读取 RC 的 UAV，例如 `/uav1/mavros/rc/in`。
- `run_id` 必须和所有 UAV 侧一致，否则 service/event 会被拒绝。
- `traj_type` 只需要在 GCS 侧重点配置，GCS 进入轨迹跟随时会发给 traj_router。
- `takeoff_height/takeoff_duration` 默认就是 `1.0/10.0`；如果手动覆盖，GCS 和 UAV 侧保持一致。
- 如果需要看报错，不要让 Rich 刷掉终端输出，可加 `monitor_screen:=false` 或把各节点 `output:=log`。

### 实飞 UAV 侧

每台 UAV 启动 `sz_indoor_launch/launch/coop_uav.launch`。master 例子：

```bash
source /home/fran/sz_indoor_transport_ws/devel/setup.zsh
roslaunch sz_indoor_launch coop_uav.launch \
  uav_id:=uav1 \
  role:=master \
  master_id:=uav1 \
  run_id:=coop_lift_test_001 \
  fcu_url:=/dev/ttyUSB0:1000000 \
  tgt_system:=1 \
  tgt_component:=1 \
  simulation:=false \
  launch_controller:=true \
  takeoff_height:=1.0 \
  takeoff_duration:=10.0
```

slave 例子：

```bash
source /home/fran/sz_indoor_transport_ws/devel/setup.zsh
roslaunch sz_indoor_launch coop_uav.launch \
  uav_id:=uav2 \
  role:=slave \
  master_id:=uav1 \
  run_id:=coop_lift_test_001 \
  fcu_url:=/dev/ttyUSB0:1000000 \
  tgt_system:=1 \
  tgt_component:=1 \
  simulation:=false \
  launch_controller:=true \
  takeoff_height:=1.0 \
  takeoff_duration:=10.0
```

UAV 侧要特别注意：

- `uav_id` 会决定命名空间和话题名，例如 `/uav2/quadrotor_state`、`/uav2/trajectory`。
- `role` 只能是 `master/slave`；只有 GCS 用 `participants`。
- `master_id` 要和 GCS 保持一致，slave 也要填同一个 master。
- `run_id` 必须和 GCS 一致。
- `fcu_url/tgt_system/tgt_component` 要按每台飞控实际连接配置。
- `simulation:=false` 会启动 MAVROS；仿真或只跑逻辑时再设为 `true`。
- `launch_controller:=true` 会同时启动 MAVROS、controller、observer、UAV FSM。
- controller 默认 `auto_offboard:=true`、`auto_arm:=true`、`keep_offboard:=true`，会持续发 setpoint 并保持 OFFBOARD；上桨前务必确认参数和安全流程。
- `uav_state_topic` 默认是 `/<uav_id>/quadrotor_state`，由 observer 发布，FSM 自检依赖它。
- `controller_file_path` 默认使用 `myparams_20260323.json`，换机或换参数时要显式指定。

### GCS + Rich 操作台

```bash
source /home/fran/sz_indoor_transport_ws/devel/setup.zsh
roslaunch sz_indoor_fsm coop_fsm.launch \
  role:=gcs \
  self_id:=gcs \
  master_id:=uav0 \
  participants:=uav0,uav1 \
  node_output:=log \
  launch_monitor:=true
```

Rich 操作台按键：

```text
t  起飞确认
l  降落确认
a  ABORT
r  reset
```

### MASTER

```bash
roslaunch sz_indoor_fsm coop_fsm.launch \
  role:=master \
  self_id:=uav0 \
  master_id:=uav0 \
  node_output:=log \
  launch_monitor:=false
```

### SLAVE

```bash
roslaunch sz_indoor_fsm coop_fsm.launch \
  role:=slave \
  self_id:=uav1 \
  master_id:=uav0 \
  node_output:=log \
  launch_monitor:=false
```

### GCS + traj_router 联调

没有真实 UAV 端时，可以用 `mock_multi_uav.py` 模拟多机端：

```bash
roslaunch sz_indoor_launch debug_gcs_traj_router.launch
```

默认会启动：

- `gcs_coop_fsm`
- `fsm_monitor`
- `traj_router`
- `formation_trajectory`
- `mock_multi_uav`

默认参与者是 `uav1,uav2,uav3,uav4`。mock 会发布：

- `/uav1/quadrotor_state`
- `/uav2/quadrotor_state`
- `/uav3/quadrotor_state`
- `/uav4/quadrotor_state`

这些话题类型是 `std_msgs/Float64MultiArray`，数据格式和 `observer.cpp` 保持一致：
`[px, py, pz, vx, vy, vz, qw, qx, qy, qz]`。mock 中速度为 0，姿态为单位四元数。
四台无人机的默认位置分布在半径 `mock_initial_radius` 的圆上，因此每台位置都不同；
默认 `mock_initial_radius=1.0`、`mock_initial_z=0.0`。

`/<uav_id>/traj_generation_flag` 默认不发布。如果 trajectory_node 或旧联调链路需要这些
flag，可启动时加 `mock_publish_traj_flags:=true`。

联调流程：

1. 等 `GCS Info` 里 `prepared all=True`。
2. 在 Rich 操作台按 `t`，GCS 会向 traj_router 发送 `takeoff`，并向 mock UAV 发送 `takeoff_started`。
3. mock UAV 自动上报 `achieve` 后，GCS 会进入 `TRAJ_FOLLOWING`，并向 traj_router/UAV 发送 `traj_following`。
   发送给 traj_router 的 JSON 中会包含 `traj_type`。
4. 需要降落时，在 Rich 操作台按 `l`，或触发配置的降落 RC 通道。
5. GCS 会直接调用 `_start_land_from_gcs()`：向 traj_router 发送 `land`，向所有 UAV 发送 `land`，并进入 `LAND`。
6. GCS 自动或手动 reset 后，mock 会监听 `/gcs/fsm/status`，看到 GCS 回到新一轮
   `PREPARE/WAIT_TAKEOFF` 且 prepared/achieve 已清空时，也重置自己的 `prepared`、
   `achieve`、高度和模式锁存。

`debug_gcs_traj_router.launch` 已把 GCS、traj_router、trajectory_node、mock UAV 的参数都暴露在顶层。
常用调试参数示例：

```bash
roslaunch sz_indoor_launch debug_gcs_traj_router.launch \
  participants:=uav1,uav2,uav3,uav4 \
  traj_type:=1 \
  trajectory_omega:=0.2 \
  mock_auto_stop:=false \
  mock_publish_traj_flags:=false \
  mock_sync_reset_from_gcs_status:=true \
  rc_land_channel:=5 \
  rc_land_threshold:=1800
```

## 自动测试

半自动测试，起飞/降落由 Rich 操作台按键完成：

```bash
roslaunch sz_indoor_fsm coop_fsm_auto_test.launch launch_monitor:=true
```

流程：

1. 等 `GCS Info` 里 `prepared all=True`。
2. 按 `t`。
3. GCS 进入 `TRAJ_FOLLOWING` 后按 `l`。
4. 测试脚本看到 `LAND -> PREPARE/WAIT_TAKEOFF` 后 clean exit。

全自动回归测试：

```bash
roslaunch sz_indoor_fsm coop_fsm_auto_test.launch \
  launch_monitor:=false \
  manual_operator:=false
```

全自动测试默认也走直接降落路径：进入 `TRAJ_FOLLOWING` 并运行 `follow_duration` 后，
测试节点会打降落 RC。需要回测旧的 `stop -> STOP -> land` 路径时，可加
`auto_send_stop:=true`。

## launch 参数说明

`coop_fsm.launch` 里每个 arg 已经逐项写了 XML 注释。最常改的是：

- `role`：`gcs/master/slave`。
- `self_id`：当前节点 ID。
- `master_id`：master UAV ID，GCS 默认从它的 MAVROS RC topic 读遥控器。
- `participants`：只给 GCS 使用的 UAV 列表。GCS 用它等待 prepared/achieve，并向所有 UAV 广播状态命令；MASTER/SLAVE 不需要知道这个名单。
- `run_id`：本轮实验 ID，所有节点必须一致。
- `takeoff_height`：起飞目标高度，默认 `1.0` m。
- `takeoff_duration`：发给 traj_router 的起飞时间，同时用于超时判据，默认 `10.0` s。
- `uav_state_topic`：`quadrotor_state` 输入话题，默认 `/<self_id>/quadrotor_state`。
- `health_min_rate_hz`：`quadrotor_state` 最低频率。
- `traj_router_service`：中央轨迹路由服务。
- `gcs_event_service`：GCS 接收 prepared/achieve/stop 的服务。
- `operator_service`：Rich 操作台控制 GCS 的服务。
- `rc_*`：起飞/降落/ABORT 遥控器通道和阈值。
- `node_output`：建议 `log`，避免污染 Rich 终端。
- `launch_monitor`：是否随 launch 启动 Rich 操作台。

## 在哪里加自己的状态操作

推荐只改 `scripts/fsm.py` 里的 hook，不要直接把业务代码塞进 `_tick_gcs()` / `_tick_uav()`。

### 状态进入时执行一次

在这些函数里加代码：

```python
def on_enter_traj_following(self, old_state, reason):
    # 例如：切控制器到轨迹跟踪模式
    pass

def on_enter_land(self, old_state, reason):
    # 例如：通知执行器准备降落
    pass
```

所有可用入口 hook：

- `on_enter_prepare`
- `on_enter_wait_takeoff`
- `on_enter_takeoff_running`
- `on_enter_achieve`
- `on_enter_wait`
- `on_enter_traj_following`
- `on_enter_stop`
- `on_enter_land`
- `on_enter_abort`

### 在某个状态内周期执行

在这些函数里加代码：

```python
def on_tick_wait(self):
    # 例如：持续发布 hold 指令
    pass
```

所有可用周期 hook：

- `on_tick_prepare`
- `on_tick_wait_takeoff`
- `on_tick_takeoff_running`
- `on_tick_achieve`
- `on_tick_wait`
- `on_tick_traj_following`
- `on_tick_stop`
- `on_tick_land`
- `on_tick_abort`

### 自定义 quadrotor_state 有效性

改这里：

```python
def _uav_state_msg_valid(self, msg: Float64MultiArray):
    ...
```

默认检查 `data` 至少包含 10 个数，位置/速度是否为有限数值；可选检查姿态四元数。

### 自定义起飞完成判据

改这里：

```python
def _takeoff_reached(self) -> bool:
    ...
```

默认是高度接近 `takeoff_height` 或超过 `takeoff_duration + takeoff_timeout_margin`。

### 接真实强制降落接口

改这里：

```python
def force_land(self, reason: str, payload: Dict[str, Any]):
    ...
```

这个函数只在 UAV 进入 `ABORT` 时执行。GCS 的 ABORT 行为是广播 `abort` 给 traj_router 和所有 UAV。

## rosbag 建议

ROS1 rosbag 不能直接录 service。建议录这些 topic：

```bash
rosbag record \
  /gcs/fsm/status /uav0/fsm/status /uav1/fsm/status \
  /gcs/fsm/service_audit /uav0/fsm/service_audit /uav1/fsm/service_audit
```

这样状态、service JSON payload、成功/失败结果都能复盘。

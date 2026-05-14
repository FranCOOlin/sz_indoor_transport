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
  -> STOP
  -> LAND
  -> PREPARE
```

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
| `PREPARE` | GCS 不做本地 UAVState 自检，进入等待 UAV 准备状态 | `WAIT_TAKEOFF` | `_set_state(WAIT_TAKEOFF, "gcs_skip_prepare_check")` |
| `WAIT_TAKEOFF` | `_all_prepared()` 为真，且 `_manual_takeoff_requested(keyboard_commands)` 为真 | `TAKEOFF_RUNNING` | `_start_takeoff_from_gcs()` |
| `TAKEOFF_RUNNING` | `_all_achieved()` 为真 | `TRAJ_FOLLOWING` | `_start_traj_following_from_gcs()` |
| `TRAJ_FOLLOWING` | GCS 的 event service 收到 `stop`，且当前状态仍是 `TRAJ_FOLLOWING` | `STOP` | `_event_service_cb()` -> `_set_state(STOP, "traj_router_stop")` |
| `STOP` | `_manual_land_requested(keyboard_commands)` 为真 | `LAND` | `_start_land_from_gcs()` |
| `LAND` | `now_sec() - land_start_time >= land_reset_delay` | `PREPARE` | `_reset_for_next_round("land_reset_delay")` |

`_start_takeoff_from_gcs()` 会先 `_set_state(TAKEOFF_RUNNING, "manual_takeoff_confirmed")`，
再调用 `_send_to_traj_router("takeoff", payload)`。如果 `send_takeoff_started_to_uavs`
为真，还会 `_broadcast_to_uavs("takeoff_started", payload)`。

`_start_traj_following_from_gcs()` 会调用：

```python
_send_to_traj_router("traj_following", payload)
_broadcast_to_uavs("traj_following", payload)
_set_state(TRAJ_FOLLOWING, "all_uavs_achieved")
```

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

- 最近收到过 `UAVState`，且未超过 `health_timeout`。
- `_health_rate() >= health_min_rate_hz`。
- `_uav_state_msg_valid(msg)` 通过。

`_takeoff_reached()` 默认接受两个条件之一：

- 当前高度 `position.z` 与 `takeoff_height` 的差值不超过 `takeoff_reached_tolerance`。
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
| `stop` | 保存 `last_stop_payload` | 如果 GCS 正在 `TRAJ_FOLLOWING`，立即切到 `STOP` |
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

- `/<uav_id>/quadrotor_feedback`
  - `sz_indoor_controller/UAVState`
  - UAV 自检和起飞高度判断输入。可用 `uav_state_topic:=...` 改成你的真实话题。

## 常用启动

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

## 自动测试

半自动测试，起飞/降落由 Rich 操作台按键完成：

```bash
roslaunch sz_indoor_fsm coop_fsm_auto_test.launch launch_monitor:=true
```

流程：

1. 等 `GCS Info` 里 `prepared all=True`。
2. 按 `t`。
3. 等 GCS 到 `STOP`。
4. 按 `l`。
5. 测试脚本看到 `LAND -> PREPARE/WAIT_TAKEOFF` 后 clean exit。

全自动回归测试：

```bash
roslaunch sz_indoor_fsm coop_fsm_auto_test.launch \
  launch_monitor:=false \
  manual_operator:=false
```

## launch 参数说明

`coop_fsm.launch` 里每个 arg 已经逐项写了 XML 注释。最常改的是：

- `role`：`gcs/master/slave`。
- `self_id`：当前节点 ID。
- `master_id`：master UAV ID，GCS 默认从它的 MAVROS RC topic 读遥控器。
- `participants`：只给 GCS 使用的 UAV 列表。GCS 用它等待 prepared/achieve，并向所有 UAV 广播状态命令；MASTER/SLAVE 不需要知道这个名单。
- `run_id`：本轮实验 ID，所有节点必须一致。
- `takeoff_height`：起飞目标高度。
- `takeoff_duration`：发给 traj_router 的起飞时间，同时用于超时判据。
- `uav_state_topic`：UAVState 输入话题。
- `health_min_rate_hz`：UAVState 最低频率。
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

### 自定义 UAVState 有效性

改这里：

```python
def _uav_state_msg_valid(self, msg: UAVState):
    ...
```

默认检查 `position`、`velocity` 是否为有限数值；可选检查姿态四元数。

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

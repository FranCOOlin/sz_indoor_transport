# sz_indoor_fsm 使用说明

`sz_indoor_fsm` 提供一套 ROS1 Python 协同任务状态机，面向 GCS、MASTER、SLAVE 三种角色。

核心文件：

- `scripts/fsm.py`：实际任务状态机。
- `scripts/fsm_monitor.py`：Rich 终端 GCS 操作台。
- `scripts/fsm_auto_test.py`：自动/半自动集成测试驱动。
- `srv/JsonCommand.srv`：所有 FSM service 使用的 JSON 命令接口。
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
  participants:=uav0,uav1 \
  node_output:=log \
  launch_monitor:=false
```

### SLAVE

```bash
roslaunch sz_indoor_fsm coop_fsm.launch \
  role:=slave \
  self_id:=uav1 \
  master_id:=uav0 \
  participants:=uav0,uav1 \
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
- `participants`：GCS 等待 prepared/achieve 的 UAV 列表。
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

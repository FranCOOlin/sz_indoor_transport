# traj_router 对接说明

`traj_router` 只需要和 FSM 通过一个通用 JSON service 对接：

- `traj_router` 提供 `/traj_router/command`
- GCS FSM 调用 `/traj_router/command`
- `traj_router` 在主轨迹完成、异常中止等时机调用 `/gcs/fsm/event` 通知 GCS

所有 service 都使用同一个类型：

```srv
# sz_indoor_fsm/srv/JsonCommand.srv
string json
---
bool success
string message
```

`json` 字段是一个 JSON object 编码后的字符串。返回值里：

- `success=true`：命令已被接收，FSM 会继续往下走。
- `success=false`：命令被拒绝，`message` 里写明原因。

## 1. traj_router 需要提供的 service

### `/traj_router/command`

类型：

```text
sz_indoor_fsm/JsonCommand
```

方向：

```text
GCS FSM -> traj_router
```

GCS 会向这个 service 发送四类命令：

| cmd | 时机 | traj_router 建议动作 |
| --- | --- | --- |
| `takeoff` | GCS 收到所有 UAV 的 `prepared`，并收到起飞确认后 | 规划并开始起飞轨迹 |
| `traj_following` | 所有 UAV 起飞完成并上报 `achieve` 后 | 切换/确认进入主任务轨迹阶段 |
| `land` | 主轨迹完成后，GCS 收到降落确认 | 规划并开始正常降落轨迹 |
| `abort` | GCS 或 UAV 触发紧急中止 | 立即停止正常规划，进入你们定义的应急处理 |

所有 GCS 发来的 JSON 都至少包含：

```json
{
  "cmd": "takeoff",
  "event": "takeoff",
  "run_id": "coop_lift_test_001",
  "source": "gcs",
  "stamp": 1778737200.123
}
```

字段说明：

- `cmd` / `event`：命令名。两者通常相同，解析时优先看 `cmd` 即可。
- `run_id`：本轮实验 ID。traj_router 回调 GCS 时必须原样带回。
- `source`：发送方，GCS 发给 traj_router 时通常是 `gcs`。
- `stamp`：发送时刻，单位秒，来自 ROS time。

### `takeoff` payload

GCS 发送：

```json
{
  "cmd": "takeoff",
  "event": "takeoff",
  "run_id": "coop_lift_test_001",
  "source": "gcs",
  "stamp": 1778737200.123,
  "height": 3.0,
  "duration": 6.0,
  "participants": ["uav0", "uav1"]
}
```

字段说明：

- `height`：目标起飞高度，单位 m。
- `duration`：期望起飞时间，单位 s。
- `participants`：参与本轮任务的 UAV ID 列表。

建议行为：

1. 检查 `height > 0`、`duration > 0`。
2. 保存 `run_id`。
3. 开始生成/发布起飞轨迹。
4. 返回 `success=true`。

注意：UAV 是否认为起飞完成，由 UAV FSM 根据自身 `UAVState` 高度或超时判断，并向 GCS 上报 `achieve`。traj_router 不需要上报起飞完成。

### `traj_following` payload

GCS 发送：

```json
{
  "cmd": "traj_following",
  "event": "traj_following",
  "run_id": "coop_lift_test_001",
  "source": "gcs",
  "stamp": 1778737210.456,
  "participants": ["uav0", "uav1"]
}
```

建议行为：

1. 切换到主轨迹阶段。
2. 开始或确认持续发布主任务轨迹。
3. 主轨迹完成后，调用 `/gcs/fsm/event` 发送 `stop`。

### `land` payload

GCS 发送：

```json
{
  "cmd": "land",
  "event": "land",
  "run_id": "coop_lift_test_001",
  "source": "gcs",
  "stamp": 1778737220.789,
  "participants": ["uav0", "uav1"]
}
```

建议行为：

1. 停止主轨迹阶段。
2. 规划/发布正常降落轨迹。
3. 返回 `success=true`。

当前 FSM 不要求 traj_router 回报 nominal landing done。GCS 和 UAV FSM 会在 `LAND` 状态保持一段时间后自动回到下一轮 `PREPARE`。

### `abort` payload

GCS 发送：

```json
{
  "cmd": "abort",
  "event": "abort",
  "run_id": "coop_lift_test_001",
  "source": "gcs",
  "stamp": 1778737225.000,
  "reason": "manual_abort"
}
```

建议行为：

1. 停止正常轨迹输出。
2. 进入你们定义的应急策略。
3. 返回 `success=true`。

真正的 UAV 强制降落接口目前留在 UAV FSM 的 `force_land()` 里，traj_router 这里只需要停止/切换轨迹侧行为。

## 2. traj_router 需要调用的 GCS service

### `/gcs/fsm/event`

类型：

```text
sz_indoor_fsm/JsonCommand
```

方向：

```text
traj_router -> GCS FSM
```

traj_router 主要需要发送两个事件：

| event | 时机 | GCS 行为 |
| --- | --- | --- |
| `stop` | 主轨迹完成 | 如果 GCS 当前在 `TRAJ_FOLLOWING`，切换到 `STOP`，等待操作员确认降落 |
| `abort` | traj_router 自己发现无法继续安全执行 | GCS 进入 `ABORT`，并广播 abort 给 UAV |

### 主轨迹完成：发送 `stop`

```json
{
  "event": "stop",
  "cmd": "stop",
  "run_id": "coop_lift_test_001",
  "source": "traj_router",
  "stamp": 1778737220.000,
  "reason": "trajectory_finished"
}
```

`reason` 可自由填写，GCS 会保存到 `last_stop_payload`，方便调试和 rosbag 复盘。

### 轨迹侧异常：发送 `abort`

```json
{
  "event": "abort",
  "cmd": "abort",
  "run_id": "coop_lift_test_001",
  "source": "traj_router",
  "stamp": 1778737221.000,
  "reason": "planner_failed"
}
```

## 3. Python 示例：实现 `/traj_router/command`

下面是一个最小可运行 service server 示例，演示如何接收 `JsonCommand`、解析 JSON、分发不同命令。

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json

import rospy

from sz_indoor_fsm.srv import JsonCommand, JsonCommandResponse


class TrajRouterBridge:
    def __init__(self):
        self.run_id = ""
        self.active_mode = "idle"
        self.command_service = rospy.get_param("~command_service", "/traj_router/command")

        self.server = rospy.Service(
            self.command_service,
            JsonCommand,
            self.handle_command,
        )
        rospy.loginfo("[traj_router] command service ready: %s", self.command_service)

    def handle_command(self, req):
        try:
            payload = json.loads(req.json)
        except Exception as exc:
            return JsonCommandResponse(False, "invalid json: %s" % exc)

        if not isinstance(payload, dict):
            return JsonCommandResponse(False, "json must be an object")

        cmd = str(payload.get("cmd", payload.get("event", ""))).lower().strip()
        run_id = str(payload.get("run_id", "")).strip()
        source = str(payload.get("source", "")).strip()

        if not run_id:
            return JsonCommandResponse(False, "missing run_id")

        # 第一次收到命令时锁定 run_id。后续可以按需拒绝旧实验残留命令。
        if not self.run_id:
            self.run_id = run_id
        elif run_id != self.run_id:
            return JsonCommandResponse(False, "run_id mismatch")

        rospy.loginfo("[traj_router] cmd=%s run_id=%s source=%s payload=%s",
                      cmd, run_id, source, payload)

        if cmd == "takeoff":
            return self.handle_takeoff(payload)
        if cmd == "traj_following":
            return self.handle_traj_following(payload)
        if cmd == "land":
            return self.handle_land(payload)
        if cmd == "abort":
            return self.handle_abort(payload)

        return JsonCommandResponse(False, "unknown cmd: %s" % cmd)

    def handle_takeoff(self, payload):
        height = float(payload.get("height", 0.0))
        duration = float(payload.get("duration", 0.0))
        participants = payload.get("participants", [])

        if height <= 0.0:
            return JsonCommandResponse(False, "height must be positive")
        if duration <= 0.0:
            return JsonCommandResponse(False, "duration must be positive")

        # TODO: 在这里接你们真正的起飞轨迹规划/发布逻辑。
        # 示例：
        # self.router.plan_takeoff(height=height, duration=duration, uavs=participants)
        self.active_mode = "takeoff"

        rospy.loginfo("[traj_router] takeoff accepted height=%.2f duration=%.2f uavs=%s",
                      height, duration, participants)
        return JsonCommandResponse(True, "takeoff accepted")

    def handle_traj_following(self, payload):
        participants = payload.get("participants", [])

        # TODO: 在这里切到主轨迹模式。
        # self.router.start_main_trajectory(uavs=participants)
        self.active_mode = "traj_following"

        rospy.loginfo("[traj_router] traj_following accepted uavs=%s", participants)
        return JsonCommandResponse(True, "traj_following accepted")

    def handle_land(self, payload):
        participants = payload.get("participants", [])

        # TODO: 在这里接正常降落轨迹。
        # self.router.plan_land(uavs=participants)
        self.active_mode = "land"

        rospy.loginfo("[traj_router] land accepted uavs=%s", participants)
        return JsonCommandResponse(True, "land accepted")

    def handle_abort(self, payload):
        reason = str(payload.get("reason", "abort"))

        # TODO: 在这里停止正常轨迹，切换到你们的应急轨迹/空输出/安全输出。
        # self.router.abort(reason=reason)
        self.active_mode = "abort"

        rospy.logwarn("[traj_router] abort accepted reason=%s", reason)
        return JsonCommandResponse(True, "abort accepted")


def main():
    rospy.init_node("traj_router_bridge")
    TrajRouterBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
```

## 4. Python 示例：主轨迹结束后通知 GCS

当 traj_router 判断主任务轨迹完成时，调用 `/gcs/fsm/event`：

```python
import json

import rospy

from sz_indoor_fsm.srv import JsonCommand


def call_gcs_event(event, run_id, reason=""):
    service_name = "/gcs/fsm/event"
    payload = {
        "event": event,
        "cmd": event,
        "run_id": run_id,
        "source": "traj_router",
        "stamp": rospy.Time.now().to_sec(),
        "reason": reason,
    }

    rospy.wait_for_service(service_name, timeout=1.0)
    proxy = rospy.ServiceProxy(service_name, JsonCommand)
    resp = proxy(json.dumps(payload, sort_keys=True, separators=(",", ":")))

    if not resp.success:
        rospy.logwarn("[traj_router] GCS rejected event=%s message=%s",
                      event, resp.message)
    return resp.success


# 主轨迹完成时：
call_gcs_event("stop", run_id="coop_lift_test_001", reason="trajectory_finished")

# traj_router 自己发现异常时：
call_gcs_event("abort", run_id="coop_lift_test_001", reason="planner_failed")
```

建议 traj_router 保存最近一次从 GCS 收到的 `run_id`，回调 GCS 时使用同一个 `run_id`。如果 `run_id` 不一致，GCS 会拒绝该事件，避免旧进程或旧实验残留消息污染当前流程。

## 5. 快速手动测试

启动 FSM 自动测试环境时，测试脚本已经提供了一个 fake `/traj_router/command`。如果要测试真实 traj_router，把 fake 测试节点关掉，或者单独启动 GCS/UAV FSM，然后启动你的 traj_router。

也可以直接用 `rosservice call` 模拟 GCS 发来的命令：

```bash
rosservice call /traj_router/command "json: '{\"cmd\":\"takeoff\",\"event\":\"takeoff\",\"run_id\":\"coop_lift_test_001\",\"source\":\"gcs\",\"stamp\":0.0,\"height\":3.0,\"duration\":6.0,\"participants\":[\"uav0\",\"uav1\"]}'"
```

模拟 traj_router 通知 GCS 主轨迹完成：

```bash
rosservice call /gcs/fsm/event "json: '{\"event\":\"stop\",\"cmd\":\"stop\",\"run_id\":\"coop_lift_test_001\",\"source\":\"traj_router\",\"stamp\":0.0,\"reason\":\"manual_test\"}'"
```

## 6. rosbag 复盘建议

ROS1 `rosbag` 不能直接录 service 调用。FSM 会把每次 JSON service 收/发镜像到 audit topic：

```bash
rosbag record \
  /gcs/fsm/status /uav0/fsm/status /uav1/fsm/status \
  /gcs/fsm/service_audit /uav0/fsm/service_audit /uav1/fsm/service_audit
```

`service_audit` 里的 `payload` 字段就是实际 service JSON，包含 `cmd/event/run_id/source/stamp` 和业务参数。

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
协同吊运任务有限状态机。

本节点有意把所有高层任务决策集中在一个 Python 文件中，便于在飞行测试迭代时检查和修改。

角色
----
GCS:
    中央协调器。收集各 UAV FSM 上报的 prepared/achieve 事件，
    向 traj_router 和各 UAV 发送 JSON 服务命令，并等待操作员确认起飞/降落。

MASTER / SLAVE:
    单机任务 FSM。每架 UAV 检查自己的 UAVState 数据流，向 GCS 上报
    prepared/achieve，接收 GCS 下发的状态切换命令，并拥有本地 ABORT 强制降落钩子。

自定义行为的添加位置
--------------------
最安全的扩展点位于 CoopFSM 靠近文件底部的位置：

    1. on_enter_<state>() hooks
       状态切换后立即调用一次。
       适合放置一次性操作，例如切换控制器模式、使能子系统、
       锁存轨迹 id，或通知其他节点。

    2. on_tick_<state>() hooks
       FSM 保持在该状态期间，每个控制周期都会调用。
       适合放置与状态相关的周期性工作，例如发布悬停命令、
       检查外部条件，或喂 watchdog。

    3. force_land()
       为 ABORT 强制降落预留的接口。当前版本只打印日志；
       请在这里接入真实的降落、kill 或 RTL 接口。

核心状态转移逻辑位于 _tick_gcs() 和 _tick_uav()。尽量让这些方法只负责决定
*何时切换状态*。除非副作用本身就是状态转移命令，否则请把副作用放到上面的钩子中。
"""

import json
import math
import select
import sys
import termios
import threading
import tty
from collections import deque
from typing import Any, Dict, List, Optional

import rospy
from mavros_msgs.msg import RCIn
from std_msgs.msg import String

from sz_indoor_controller.msg import UAVState
from sz_indoor_fsm.srv import JsonCommand, JsonCommandResponse


PREPARE = "PREPARE"
# 仅 GCS 使用的等待状态：GCS 已跳过自身检查，正在等待 UAV 上报 prepared，
# 随后等待操作员确认起飞。
WAIT_TAKEOFF = "WAIT_TAKEOFF"
TAKEOFF_RUNNING = "TAKEOFF_RUNNING"
ACHIEVE = "ACHIEVE"
WAIT = "WAIT"
TRAJ_FOLLOWING = "TRAJ_FOLLOWING"
STOP = "STOP"
LAND = "LAND"
ABORT = "ABORT"

ROLE_GCS = "gcs"
ROLE_MASTER = "master"
ROLE_SLAVE = "slave"


def now_sec() -> float:
    """返回以秒为单位的 ROS 时间。封装这一层便于测试或替身实现。"""
    return rospy.Time.now().to_sec()


def parse_csv(value: Any) -> List[str]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(v).strip() for v in value if str(v).strip()]
    return [part.strip() for part in str(value).split(",") if part.strip()]


def finite(values: List[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def json_dumps(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def json_loads(text: str) -> Optional[Dict[str, Any]]:
    try:
        payload = json.loads(text)
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


class KeyboardReader(threading.Thread):
    """小型非阻塞 stdin 读取线程。

    仅当 FSM 本身在交互式终端中启动时使用。正常 GCS 运行时，
    Rich 监视器会改为调用 operator 服务，从而把 FSM 日志和键盘处理分离。
    """

    def __init__(self):
        super().__init__(daemon=True)
        self.enabled = bool(sys.stdin and sys.stdin.isatty())
        self._lock = threading.Lock()
        self._keys = deque()
        self._stop = False
        self._old_term = None

    def run(self):
        if not self.enabled:
            return
        try:
            self._old_term = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            while not rospy.is_shutdown() and not self._stop:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue
                ch = sys.stdin.read(1)
                with self._lock:
                    self._keys.append(ch)
        finally:
            if self._old_term is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)

    def pop_all(self) -> List[str]:
        with self._lock:
            keys = list(self._keys)
            self._keys.clear()
        return keys

    def stop(self):
        self._stop = True


class CoopFSM:
    """感知角色的协同 FSM。

    这里有意暂不拆分为多个子类。大多数飞行测试修改都需要并排查看 GCS/UAV 行为，
    目前按角色分支的复杂度仍然足够小，放在一个文件中更容易阅读。
    """

    def __init__(self):
        # ------------------------------------------------------------------
        # 身份与运行范围
        # ------------------------------------------------------------------
        # run_id 会放入所有 JSON 服务载荷中，防止新启动的 FSM 误接收旧测试轮次的过期事件。
        self.role = str(rospy.get_param("~role", ROLE_SLAVE)).lower().strip()
        if self.role not in [ROLE_GCS, ROLE_MASTER, ROLE_SLAVE]:
            raise RuntimeError("~role must be one of: gcs, master, slave")

        self.is_gcs = self.role == ROLE_GCS
        self.self_id = str(
            rospy.get_param("~self_id", "gcs" if self.is_gcs else "uav0")
        ).strip()
        self.master_id = str(rospy.get_param("~master_id", "uav0")).strip()
        self.run_id = str(rospy.get_param("~run_id", "coop_lift_test_001")).strip()
        self.topic_prefix = f"/{self.self_id.strip('/')}"

        # 只有 GCS 持有全局 UAV 列表。GCS 使用该列表判断是否所有飞机都已上报
        # prepared/achieve，并向所有 UAV FSM 节点广播高层命令。单个 UAV FSM 应保持本地化：
        # 只需要 self_id/role 以及 GCS 服务地址。
        if self.is_gcs:
            self.participants = parse_csv(rospy.get_param("~participants", ""))
            if not self.participants:
                self.participants = [self.master_id]
            self.participants = [p for p in self.participants if p and p != "gcs"]
        else:
            self.participants = []

        # ------------------------------------------------------------------
        # 循环与状态发布频率
        # ------------------------------------------------------------------
        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 30.0))
        self.status_rate_hz = float(rospy.get_param("~status_rate_hz", 5.0))
        self.status_pub_period = 1.0 / max(self.status_rate_hz, 1e-6)
        self.last_status_pub_time = 0.0

        # ------------------------------------------------------------------
        # 任务时序与阈值
        # ------------------------------------------------------------------
        # takeoff_duration 会发送给 traj_router；当高度反馈不可用或延迟时，
        # 也会作为 achieve 的兜底超时时间。
        self.takeoff_height = float(rospy.get_param("~takeoff_height", 3.0))
        self.takeoff_duration = float(rospy.get_param("~takeoff_duration", 6.0))
        self.takeoff_reached_tolerance = float(
            rospy.get_param("~takeoff_reached_tolerance", 0.25)
        )
        self.takeoff_timeout_margin = float(
            rospy.get_param("~takeoff_timeout_margin", 2.0)
        )
        self.land_reset_delay = float(rospy.get_param("~land_reset_delay", 5.0))
        self.service_timeout = float(rospy.get_param("~service_timeout", 0.3))

        # ------------------------------------------------------------------
        # 基于 sz_indoor_controller/UAVState 的 UAV 健康检查
        # ------------------------------------------------------------------
        # 话题按节点 id 划分作用域。GCS 自然位于 /gcs 下，
        # UAV 使用 /uav0、/uav1 等前缀，这样话题列表更容易浏览。
        # 如果真实估计器在其他位置发布 UAVState，请覆盖 ~uav_state_topic。
        self.health_topic = str(
            rospy.get_param("~uav_state_topic", f"{self.topic_prefix}/quadrotor_feedback")
        )
        self.health_min_rate_hz = float(rospy.get_param("~health_min_rate_hz", 20.0))
        self.health_timeout = float(rospy.get_param("~health_timeout", 1.0))
        self.health_window_sec = float(rospy.get_param("~health_window_sec", 2.0))
        self.require_attitude_valid = bool(
            rospy.get_param("~require_attitude_valid", False)
        )
        self.attitude_norm_tolerance = float(
            rospy.get_param("~attitude_norm_tolerance", 0.25)
        )

        # ------------------------------------------------------------------
        # JSON 服务接口
        # ------------------------------------------------------------------
        # gcs_event_service:
        #   UAV/traj_router -> GCS 的事件入口，例如 prepared/achieve/stop。
        # operator_service:
        #   Rich 监视器/终端 UI -> GCS 操作员命令。
        # command_service:
        #   GCS -> 当前 UAV 的命令入口，例如 traj_following/land/abort。
        self.gcs_event_service = str(
            rospy.get_param("~gcs_event_service", "/gcs/fsm/event")
        )
        self.operator_service = str(
            rospy.get_param(
                "~operator_service",
                "/gcs/fsm/operator" if self.is_gcs else f"/{self.self_id}/fsm/operator",
            )
        )
        self.command_service = str(
            rospy.get_param("~command_service", f"/{self.self_id}/fsm/command")
        )
        self.traj_router_service = str(
            rospy.get_param("~traj_router_service", "/traj_router/command")
        )
        self.uav_command_service_template = str(
            rospy.get_param("~uav_command_service_template", "/{uav_id}/fsm/command")
        )
        self.send_takeoff_started_to_uavs = bool(
            rospy.get_param("~send_takeoff_started_to_uavs", True)
        )

        # ------------------------------------------------------------------
        # 可观测性与回放相关话题
        # ------------------------------------------------------------------
        # status_topic 供 Rich 仪表盘使用。service_audit_topic 会把服务请求/响应
        # 镜像为 rosbag 可记录的 String 消息，因为 ROS1 rosbag 不能直接记录服务调用。
        self.status_topic = str(
            rospy.get_param(
                "~status_topic",
                f"{self.topic_prefix}/fsm/status",
            )
        )
        self.service_audit_topic = str(
            rospy.get_param(
                "~service_audit_topic",
                f"{self.topic_prefix}/fsm/service_audit",
            )
        )
        self.rc_topic = str(
            rospy.get_param(
                "~rc_in_topic",
                f"/{self.master_id}/mavros/rc/in"
                if self.is_gcs
                else f"{self.topic_prefix}/mavros/rc/in",
            )
        )

        # ------------------------------------------------------------------
        # 本地键盘与 master 遥控输入
        # ------------------------------------------------------------------
        # RC 通道号从 1 开始，与遥控器标注一致。
        # 内部 _rc_value() 会将其转换为从 0 开始的数组索引。
        self.enable_keyboard = bool(rospy.get_param("~enable_keyboard", True))
        self.key_takeoff = str(rospy.get_param("~key_takeoff", "t"))
        self.key_land = str(rospy.get_param("~key_land", "l"))
        self.key_abort = str(rospy.get_param("~key_abort", "a"))

        self.rc_takeoff_channel = int(rospy.get_param("~rc_takeoff_channel", 12))
        self.rc_takeoff_threshold = int(rospy.get_param("~rc_takeoff_threshold", 1500))
        self.rc_takeoff_direction = str(
            rospy.get_param("~rc_takeoff_direction", "above")
        ).lower().strip()
        self.rc_land_channel = int(rospy.get_param("~rc_land_channel", 5))
        self.rc_land_threshold = int(rospy.get_param("~rc_land_threshold", 1800))
        self.rc_land_direction = str(
            rospy.get_param("~rc_land_direction", "above")
        ).lower().strip()
        self.rc_abort_channel = int(rospy.get_param("~rc_abort_channel", 11))
        self.rc_abort_threshold = int(rospy.get_param("~rc_abort_threshold", 1800))
        self.rc_abort_direction = str(
            rospy.get_param("~rc_abort_direction", "above")
        ).lower().strip()

        # ------------------------------------------------------------------
        # 运行时状态
        # ------------------------------------------------------------------
        self.state = PREPARE
        self.prev_state = ""
        self.state_enter_time = now_sec()
        self.takeoff_start_time = 0.0
        self.takeoff_started = False
        self.land_start_time = 0.0
        self.abort_reason = ""

        self.prepared_uavs = set()
        self.achieved_uavs = set()
        self.prepared_sent = False
        self.achieve_sent = False
        self.last_stop_payload = {}
        self.operator_takeoff_requested = False
        self.operator_land_requested = False
        self.operator_abort_requested = False

        self.rc_channels = []
        self.last_rc_time = 0.0
        self.rc_takeoff_last = False
        self.rc_land_last = False

        self.last_uav_state_time = 0.0
        self.uav_state_stamps = deque()
        self.current_uav_state = None
        self.current_uav_state_valid = False
        self.current_uav_state_invalid_reason = "no_message"

        # ------------------------------------------------------------------
        # ROS 发布者、订阅者和服务
        # ------------------------------------------------------------------
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10)
        self.service_audit_pub = rospy.Publisher(
            self.service_audit_topic, String, queue_size=50
        )
        rospy.Subscriber(self.rc_topic, RCIn, self._rc_cb, queue_size=10)

        if self.is_gcs:
            self.event_srv = rospy.Service(
                self.gcs_event_service, JsonCommand, self._event_service_cb
            )
            self.operator_srv = rospy.Service(
                self.operator_service, JsonCommand, self._operator_service_cb
            )
        else:
            self.command_srv = rospy.Service(
                self.command_service, JsonCommand, self._command_service_cb
            )
            rospy.Subscriber(self.health_topic, UAVState, self._uav_state_cb, queue_size=50)

        self.keyboard = KeyboardReader() if self.enable_keyboard else None
        if self.keyboard:
            self.keyboard.start()
            if not self.keyboard.enabled:
                rospy.loginfo("[%s FSM] keyboard disabled: stdin is not a TTY", self.self_id)

        rospy.loginfo(
            "[%s FSM] role=%s state=%s participants=%s status=%s operator=%s",
            self.self_id,
            self.role,
            self.state,
            ",".join(self.participants),
            self.status_topic,
            self.operator_service if self.is_gcs else "",
        )

    def _set_state(self, new_state: str, reason: str = ""):
        """切换 FSM 状态，并执行进入状态时的副作用。

        将状态转移的簿记集中在这里，确保每次转移都会一致地更新状态持续时间、
        prev_state、日志、计时器和扩展钩子。
        """
        if new_state == self.state:
            return
        old_state = self.state
        self.prev_state = old_state
        self.state = new_state
        self.state_enter_time = now_sec()
        rospy.loginfo(
            "[%s FSM] %s -> %s%s",
            self.self_id,
            old_state,
            new_state,
            f" reason={reason}" if reason else "",
        )
        if new_state == TAKEOFF_RUNNING and old_state != TAKEOFF_RUNNING:
            if not self.is_gcs and self.takeoff_started:
                self.takeoff_start_time = now_sec()
        elif new_state == LAND:
            self.land_start_time = now_sec()
        elif new_state == ABORT:
            self.force_land(reason or "abort", {})
        self._on_enter_state(new_state, old_state, reason)

    def _rc_cb(self, msg: RCIn):
        self.last_rc_time = now_sec()
        self.rc_channels = list(msg.channels)

    def _uav_state_cb(self, msg: UAVState):
        t = now_sec()
        self.last_uav_state_time = t
        self.uav_state_stamps.append(t)
        while self.uav_state_stamps and t - self.uav_state_stamps[0] > self.health_window_sec:
            self.uav_state_stamps.popleft()
        self.current_uav_state = msg
        self.current_uav_state_valid, self.current_uav_state_invalid_reason = (
            self._uav_state_msg_valid(msg)
        )

    def _uav_state_msg_valid(self, msg: UAVState):
        """校验 UAVState 的语义内容。

        当前默认检查：
        - 位置和速度都是有限数值
        - 可选的姿态四元数是有限数值，且接近单位模长

        如果后续向 UAVState 添加字段，请在本函数中扩展对应的有效性规则。
        """
        position_values = [msg.position.x, msg.position.y, msg.position.z]
        velocity_values = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
        if not finite(position_values + velocity_values):
            return False, "position_or_velocity_not_finite"

        if self.require_attitude_valid:
            q = msg.attitude
            quat_values = [q.x, q.y, q.z, q.w]
            if not finite(quat_values):
                return False, "attitude_not_finite"
            norm = math.sqrt(sum(float(v) * float(v) for v in quat_values))
            if abs(norm - 1.0) > self.attitude_norm_tolerance:
                return False, "attitude_norm_invalid"

        return True, "ok"

    def _health_rate(self) -> float:
        if len(self.uav_state_stamps) < 2:
            return 0.0
        span = self.uav_state_stamps[-1] - self.uav_state_stamps[0]
        if span <= 1e-6:
            return 0.0
        return float(len(self.uav_state_stamps) - 1) / span

    def _health_ok(self) -> bool:
        t = now_sec()
        fresh = self.last_uav_state_time > 0.0 and t - self.last_uav_state_time <= self.health_timeout
        return fresh and self._health_rate() >= self.health_min_rate_hz and self.current_uav_state_valid

    def _event_service_cb(self, req):
        """GCS 接收外部任务事件的服务回调。

        期望的 event 取值：
        - prepared：某架 UAV 已通过自检，可进入起飞阶段。
        - achieve：某架 UAV 已满足起飞高度/时间条件。
        - stop：traj_router 上报主轨迹已完成。
        - abort：任意节点上报紧急中止。
        """
        payload = json_loads(req.json)
        if payload is None:
            self._audit_service("in", self.gcs_event_service, {}, False, "invalid json object")
            return JsonCommandResponse(False, "invalid json object")
        if not self._run_id_matches(payload):
            self._audit_service("in", self.gcs_event_service, payload, False, "run_id mismatch")
            return JsonCommandResponse(False, "run_id mismatch")

        event = str(payload.get("event", payload.get("cmd", ""))).lower().strip()
        uav_id = str(payload.get("uav_id", payload.get("self_id", ""))).strip()
        self._audit_service("in", self.gcs_event_service, payload, True, "accepted")

        if event == "prepared":
            if uav_id:
                self.prepared_uavs.add(uav_id)
            return JsonCommandResponse(True, "prepared accepted")
        if event == "achieve":
            if uav_id:
                self.achieved_uavs.add(uav_id)
            return JsonCommandResponse(True, "achieve accepted")
        if event == "stop":
            self.last_stop_payload = payload
            if self.state == TRAJ_FOLLOWING:
                self._set_state(STOP, "traj_router_stop")
            return JsonCommandResponse(True, "stop accepted")
        if event == "abort":
            self._enter_abort(f"remote_abort:{uav_id or 'unknown'}", payload)
            return JsonCommandResponse(True, "abort accepted")

        return JsonCommandResponse(False, f"unknown event: {event}")

    def _command_service_cb(self, req):
        """UAV 接收 GCS 命令的服务回调。"""
        payload = json_loads(req.json)
        if payload is None:
            self._audit_service("in", self.command_service, {}, False, "invalid json object")
            return JsonCommandResponse(False, "invalid json object")
        if not self._run_id_matches(payload):
            self._audit_service("in", self.command_service, payload, False, "run_id mismatch")
            return JsonCommandResponse(False, "run_id mismatch")

        cmd = str(payload.get("cmd", payload.get("event", ""))).lower().strip()
        self._audit_service("in", self.command_service, payload, True, "accepted")
        if cmd == "takeoff_started":
            self.takeoff_started = True
            self.takeoff_start_time = now_sec()
            self.takeoff_height = float(payload.get("height", self.takeoff_height))
            self.takeoff_duration = float(payload.get("duration", self.takeoff_duration))
            if self.state in [PREPARE, TAKEOFF_RUNNING]:
                self._set_state(TAKEOFF_RUNNING, "gcs_takeoff_started")
            return JsonCommandResponse(True, "takeoff_started accepted")
        if cmd == "traj_following":
            self._set_state(TRAJ_FOLLOWING, "gcs_traj_following")
            return JsonCommandResponse(True, "traj_following accepted")
        if cmd == "land":
            self._set_state(LAND, "gcs_land")
            return JsonCommandResponse(True, "land accepted")
        if cmd == "abort":
            self._enter_abort("gcs_abort", payload)
            return JsonCommandResponse(True, "abort accepted")
        if cmd == "reset":
            self._reset_for_next_round("remote_reset")
            return JsonCommandResponse(True, "reset accepted")

        return JsonCommandResponse(False, f"unknown cmd: {cmd}")

    def _operator_service_cb(self, req):
        """GCS 操作员命令服务。

        用户按下 t/l/a/r 时，Rich 终端监视器会调用该服务。
        将操作员输入保持为服务，可避免仪表盘渲染与任务转移逻辑混在一起。
        """
        payload = json_loads(req.json)
        if payload is None:
            self._audit_service("in", self.operator_service, {}, False, "invalid json object")
            return JsonCommandResponse(False, "invalid json object")
        if not self._run_id_matches(payload):
            self._audit_service("in", self.operator_service, payload, False, "run_id mismatch")
            return JsonCommandResponse(False, "run_id mismatch")

        cmd = str(payload.get("cmd", payload.get("event", ""))).lower().strip()
        self._audit_service("in", self.operator_service, payload, True, "accepted")
        if cmd == "takeoff":
            self.operator_takeoff_requested = True
            return JsonCommandResponse(True, "takeoff requested")
        if cmd == "land":
            self.operator_land_requested = True
            return JsonCommandResponse(True, "land requested")
        if cmd == "abort":
            self.operator_abort_requested = True
            return JsonCommandResponse(True, "abort requested")
        if cmd == "reset":
            self._reset_for_next_round("operator_reset")
            return JsonCommandResponse(True, "reset requested")

        return JsonCommandResponse(False, f"unknown operator cmd: {cmd}")

    def _run_id_matches(self, payload: Dict[str, Any]) -> bool:
        incoming = str(payload.get("run_id", self.run_id)).strip()
        return not incoming or incoming == self.run_id

    def _all_prepared(self) -> bool:
        return set(self.participants).issubset(self.prepared_uavs)

    def _all_achieved(self) -> bool:
        return set(self.participants).issubset(self.achieved_uavs)

    def _make_payload(self, cmd: str, extra: Optional[Dict[str, Any]] = None):
        payload = {
            "cmd": cmd,
            "event": cmd,
            "run_id": self.run_id,
            "source": self.self_id,
            "stamp": now_sec(),
        }
        if extra:
            payload.update(extra)
        return payload

    def _call_json_service(self, service_name: str, payload: Dict[str, Any]) -> bool:
        """调用 JsonCommand 服务，并将结果镜像到 service_audit。"""
        try:
            rospy.wait_for_service(service_name, timeout=self.service_timeout)
            resp = rospy.ServiceProxy(service_name, JsonCommand)(json_dumps(payload))
        except Exception as exc:
            self._audit_service("out", service_name, payload, False, str(exc))
            rospy.logwarn(
                "[%s FSM] json service call failed service=%s cmd=%s error=%s",
                self.self_id,
                service_name,
                payload.get("cmd", payload.get("event", "")),
                exc,
            )
            return False

        self._audit_service("out", service_name, payload, bool(resp.success), resp.message)
        if not resp.success:
            rospy.logwarn(
                "[%s FSM] json service rejected service=%s message=%s payload=%s",
                self.self_id,
                service_name,
                resp.message,
                payload,
            )
        return bool(resp.success)

    def _audit_service(
        self,
        direction: str,
        service_name: str,
        payload: Dict[str, Any],
        success: Optional[bool],
        message: str,
    ):
        """为每次 JSON 服务调用发布 rosbag 友好的审计记录。"""
        audit = {
            "stamp": now_sec(),
            "run_id": self.run_id,
            "self_id": self.self_id,
            "role": self.role,
            "state": self.state,
            "direction": direction,
            "service": service_name,
            "success": success,
            "message": message,
            "payload": payload,
        }
        self.service_audit_pub.publish(String(json_dumps(audit)))

    def _send_gcs_event(self, event: str, extra: Optional[Dict[str, Any]] = None) -> bool:
        """向 GCS 发送一条 UAV/traj 事件。"""
        payload = self._make_payload(
            event,
            {
                "event": event,
                "uav_id": self.self_id,
                "role": self.role,
                **(extra or {}),
            },
        )
        return self._call_json_service(self.gcs_event_service, payload)

    def _send_to_traj_router(self, cmd: str, extra: Optional[Dict[str, Any]] = None) -> bool:
        """向中央 traj_router 服务发送一条高层命令。"""
        return self._call_json_service(self.traj_router_service, self._make_payload(cmd, extra))

    def _send_to_uav(self, uav_id: str, cmd: str, extra: Optional[Dict[str, Any]] = None) -> bool:
        """向指定 UAV FSM 发送一条命令。"""
        service = self.uav_command_service_template.format(uav_id=uav_id)
        payload = self._make_payload(cmd, {"target": uav_id, **(extra or {})})
        return self._call_json_service(service, payload)

    def _broadcast_to_uavs(self, cmd: str, extra: Optional[Dict[str, Any]] = None):
        """向 ~participants 中列出的每个参与者发送同一条命令。"""
        for uav_id in self.participants:
            self._send_to_uav(uav_id, cmd, extra)

    def _rc_value(self, channel: int) -> Optional[int]:
        if channel <= 0:
            return None
        index = channel - 1
        if index < 0 or index >= len(self.rc_channels):
            return None
        return int(self.rc_channels[index])

    def _rc_matches(self, channel: int, threshold: int, direction: str) -> bool:
        value = self._rc_value(channel)
        if value is None:
            return False
        if direction == "below":
            return value <= threshold
        return value >= threshold

    def _rc_rising_edge(self, name: str) -> bool:
        if name == "takeoff":
            active = self._rc_matches(
                self.rc_takeoff_channel,
                self.rc_takeoff_threshold,
                self.rc_takeoff_direction,
            )
            rising = active and not self.rc_takeoff_last
            self.rc_takeoff_last = active
            return rising
        if name == "land":
            active = self._rc_matches(
                self.rc_land_channel,
                self.rc_land_threshold,
                self.rc_land_direction,
            )
            rising = active and not self.rc_land_last
            self.rc_land_last = active
            return rising
        return False

    def _abort_active(self) -> bool:
        return self._rc_matches(
            self.rc_abort_channel,
            self.rc_abort_threshold,
            self.rc_abort_direction,
        )

    def _keyboard_commands(self) -> List[str]:
        if not self.keyboard:
            return []
        commands = []
        for key in self.keyboard.pop_all():
            if key == self.key_takeoff:
                commands.append("takeoff")
            elif key == self.key_land:
                commands.append("land")
            elif key == self.key_abort:
                commands.append("abort")
        return commands

    def _manual_takeoff_requested(self, keyboard_commands: List[str]) -> bool:
        if self.operator_takeoff_requested:
            self.operator_takeoff_requested = False
            return True
        return "takeoff" in keyboard_commands or self._rc_rising_edge("takeoff")

    def _manual_land_requested(self, keyboard_commands: List[str]) -> bool:
        if self.operator_land_requested:
            self.operator_land_requested = False
            return True
        return "land" in keyboard_commands or self._rc_rising_edge("land")

    def _manual_abort_requested(self, keyboard_commands: List[str]) -> bool:
        if self.operator_abort_requested:
            self.operator_abort_requested = False
            return True
        return "abort" in keyboard_commands or self._abort_active()

    def _enter_abort(self, reason: str, payload: Optional[Dict[str, Any]] = None):
        """锁存 ABORT，并将其扇出到所有相关节点。

        ABORT 被有意设计为粘滞状态；它不会在计时器结束后自行复位。
        这样可以防止真实紧急降落流程意外重新使能正常任务流程。
        """
        if self.state == ABORT:
            return
        self.abort_reason = reason
        self._set_state(ABORT, reason)
        if self.is_gcs:
            abort_payload = {"reason": reason, **(payload or {})}
            self._send_to_traj_router("abort", abort_payload)
            self._broadcast_to_uavs("abort", abort_payload)
        else:
            self._send_gcs_event("abort", {"reason": reason})

    def force_land(self, reason: str, payload: Dict[str, Any]):
        """为 UAV 角色预留的强制降落钩子。

        在这里放置真实的强制降落操作，例如：
        - 调用 MAVROS 模式/降落服务
        - 发布控制器紧急命令
        - 调用底层安全节点

        本地 UAV FSM 进入 ABORT 时会调用该函数。GCS 不在这里执行本地降落；
        它会向 traj_router/UAV 广播 abort。
        """
        if self.is_gcs:
            return
        rospy.logerr(
            "[%s FSM] ABORT force_land hook called reason=%s payload=%s. TODO: connect real forced landing interface here.",
            self.self_id,
            reason,
            payload,
        )

    # ==================================================================
    # 用户扩展钩子
    # ==================================================================
    # 这些钩子在基础实现中有意保持空操作。它们是推荐放置实验专用代码的位置：
    #
    #   - on_enter_*：进入某个状态时执行的一次性工作。
    #   - on_tick_* ：FSM 保持在某个状态期间执行的周期性工作。
    #
    # 示例：
    #   - 进入 TRAJ_FOLLOWING 时切换控制器模式
    #   - 在 WAIT 中每个周期发布悬停命令
    #   - 进入 LAND 时发送硬件触发
    #
    # 将状态转移决策保留在 _tick_gcs/_tick_uav 中。副作用放在这里。

    def _on_enter_state(self, new_state: str, old_state: str, reason: str):
        """分发状态进入钩子。

        如有需要，可在这里为所有钩子添加统一日志/指标。
        实际与具体状态相关的用户代码，请编辑下面的 on_enter_<state>() 方法。
        """
        handler = {
            PREPARE: self.on_enter_prepare,
            WAIT_TAKEOFF: self.on_enter_wait_takeoff,
            TAKEOFF_RUNNING: self.on_enter_takeoff_running,
            ACHIEVE: self.on_enter_achieve,
            WAIT: self.on_enter_wait,
            TRAJ_FOLLOWING: self.on_enter_traj_following,
            STOP: self.on_enter_stop,
            LAND: self.on_enter_land,
            ABORT: self.on_enter_abort,
        }.get(new_state)
        if handler:
            handler(old_state, reason)

    def _run_state_action_hook(self):
        """运行当前状态对应的周期性钩子。"""
        handler = {
            PREPARE: self.on_tick_prepare,
            WAIT_TAKEOFF: self.on_tick_wait_takeoff,
            TAKEOFF_RUNNING: self.on_tick_takeoff_running,
            ACHIEVE: self.on_tick_achieve,
            WAIT: self.on_tick_wait,
            TRAJ_FOLLOWING: self.on_tick_traj_following,
            STOP: self.on_tick_stop,
            LAND: self.on_tick_land,
            ABORT: self.on_tick_abort,
        }.get(self.state)
        if handler:
            handler()

    def on_enter_prepare(self, old_state: str, reason: str):
        """钩子：进入 PREPARE。

        在这里添加一次性复位工作。常见扩展包括：清除本地控制器状态、
        复位规划器，或在自检前准备传感器。
        """

    def on_enter_wait_takeoff(self, old_state: str, reason: str):
        """钩子：GCS 进入 WAIT_TAKEOFF。

        如果希望外部显示器或蜂鸣器提示“正在等待所有检查/准备起飞”，
        可在这里添加 GCS 侧 UI/通知工作。
        """

    def on_enter_takeoff_running(self, old_state: str, reason: str):
        """钩子：进入 TAKEOFF_RUNNING。

        UAV 注意：在 GCS 发送 takeoff_started 前，该状态表示“已准备并等待，
        无需本地控制动作”。takeoff_started 之后，预期由 traj_router 生成实际起飞轨迹。
        """

    def on_enter_achieve(self, old_state: str, reason: str):
        """钩子：UAV 进入 ACHIEVE。

        如果 UAV 必须在起飞完成且上报 achieve 给 GCS 前精确锁存某些状态，
        可在这里添加一次性操作。
        """

    def on_enter_wait(self, old_state: str, reason: str):
        """钩子：UAV 上报 ACHIEVE 后进入 WAIT。

        如果控制器在等待 GCS 下发 TRAJ_FOLLOWING 期间需要切换到保持/空闲模式，
        可在这里添加相关设置。
        """

    def on_enter_traj_following(self, old_state: str, reason: str):
        """钩子：进入 TRAJ_FOLLOWING。

        这里是将本地控制器切换到跟踪模式的自然位置。当前 FSM 只切换状态；
        不会直接向控制器发送命令。
        """

    def on_enter_stop(self, old_state: str, reason: str):
        """钩子：GCS 进入 STOP。

        可在这里添加 GCS 侧“等待降落确认”的一次性行为。
        """

    def on_enter_land(self, old_state: str, reason: str):
        """钩子：进入 LAND。

        如果正常降落需要调用接口，可在这里添加一次性降落设置。
        ABORT 的强制降落逻辑属于 force_land()。
        """

    def on_enter_abort(self, old_state: str, reason: str):
        """钩子：进入 ABORT。

        可在这里添加额外报警/日志工作。本地强制降落钩子是 force_land()，
        它已经由 _set_state() 调用。
        """

    def on_tick_prepare(self):
        """钩子：处于 PREPARE 时的周期性工作。"""

    def on_tick_wait_takeoff(self):
        """钩子：等待 prepared 与起飞确认时的 GCS 周期性工作。"""

    def on_tick_takeoff_running(self):
        """钩子：处于 TAKEOFF_RUNNING 时的周期性工作。"""

    def on_tick_achieve(self):
        """钩子：处于 ACHIEVE 时的周期性工作。"""

    def on_tick_wait(self):
        """钩子：等待 TRAJ_FOLLOWING 时的 UAV 周期性工作。"""

    def on_tick_traj_following(self):
        """钩子：处于 TRAJ_FOLLOWING 时的周期性工作。"""

    def on_tick_stop(self):
        """钩子：等待降落确认时的 GCS 周期性工作。"""

    def on_tick_land(self):
        """钩子：处于 LAND 时的周期性工作。"""

    def on_tick_abort(self):
        """钩子：处于 ABORT 时的周期性工作。"""

    def _start_takeoff_from_gcs(self):
        """所有 UAV 准备完成后，GCS 启动起飞阶段。"""
        payload = {
            "height": self.takeoff_height,
            "duration": self.takeoff_duration,
            "participants": self.participants,
        }
        self.takeoff_start_time = now_sec()
        self.takeoff_started = True
        self._set_state(TAKEOFF_RUNNING, "manual_takeoff_confirmed")
        self._send_to_traj_router("takeoff", payload)
        if self.send_takeoff_started_to_uavs:
            self._broadcast_to_uavs("takeoff_started", payload)

    def _start_traj_following_from_gcs(self):
        """所有 UAV achieve 后，GCS 启动主轨迹跟踪阶段。"""
        payload = {"participants": self.participants}
        self._send_to_traj_router("traj_following", payload)
        self._broadcast_to_uavs("traj_following", payload)
        self._set_state(TRAJ_FOLLOWING, "all_uavs_achieved")

    def _start_land_from_gcs(self):
        """STOP 后且操作员确认后，GCS 启动正常降落。"""
        payload = {"participants": self.participants}
        self._send_to_traj_router("land", payload)
        self._broadcast_to_uavs("land", payload)
        self._set_state(LAND, "manual_land_confirmed")

    def _takeoff_reached(self) -> bool:
        """UAV 起飞完成条件。

        默认行为接受以下任一条件：
        - 高度与 takeoff_height 的差值不超过 takeoff_reached_tolerance，或
        - takeoff_started 后经过 takeoff_duration + takeoff_timeout_margin。

        如果真实飞行器使用其他高度符号约定，或有更好的“起飞完成”信号，
        请修改本函数。
        """
        height_reached = False
        if self.current_uav_state is not None:
            z = float(self.current_uav_state.position.z)
            height_reached = abs(z - self.takeoff_height) <= self.takeoff_reached_tolerance
        timed_out = (
            self.takeoff_started
            and self.takeoff_start_time > 0.0
            and now_sec() - self.takeoff_start_time
            >= self.takeoff_duration + self.takeoff_timeout_margin
        )
        return height_reached or timed_out

    def _reset_for_next_round(self, reason: str):
        """返回 PREPARE 前，清除单轮任务锁存状态。"""
        self.prepared_uavs.clear()
        self.achieved_uavs.clear()
        self.prepared_sent = False
        self.achieve_sent = False
        self.takeoff_started = False
        self.takeoff_start_time = 0.0
        self.land_start_time = 0.0
        self.last_stop_payload = {}
        self.abort_reason = ""
        self.rc_takeoff_last = self._rc_matches(
            self.rc_takeoff_channel,
            self.rc_takeoff_threshold,
            self.rc_takeoff_direction,
        )
        self.rc_land_last = self._rc_matches(
            self.rc_land_channel,
            self.rc_land_threshold,
            self.rc_land_direction,
        )
        self._set_state(PREPARE, reason)

    def _tick_gcs(self, keyboard_commands: List[str]):
        """GCS 状态转移逻辑。

        让本方法专注于判断 GCS *何时* 应该切换状态。
        一次性/周期性副作用请添加到上面的钩子中。
        """
        if self.state == PREPARE:
            self._set_state(WAIT_TAKEOFF, "gcs_skip_prepare_check")
            return

        if self.state == WAIT_TAKEOFF:
            if self._all_prepared() and self._manual_takeoff_requested(keyboard_commands):
                self._start_takeoff_from_gcs()
            return

        if self.state == TAKEOFF_RUNNING:
            if self._all_achieved():
                self._start_traj_following_from_gcs()
            return

        if self.state == STOP:
            if self._manual_land_requested(keyboard_commands):
                self._start_land_from_gcs()
            return

        if self.state == LAND:
            if now_sec() - self.land_start_time >= self.land_reset_delay:
                self._reset_for_next_round("land_reset_delay")
            return

    def _tick_uav(self):
        """MASTER/SLAVE 状态转移逻辑。

        让本方法专注于判断 UAV *何时* 应该切换状态。
        控制器或硬件命令请添加到 on_enter_* / on_tick_* 钩子中。
        """
        if self.state == PREPARE:
            if self._health_ok():
                if not self.prepared_sent:
                    self.prepared_sent = self._send_gcs_event(
                        "prepared",
                        {
                            "health_rate_hz": self._health_rate(),
                            "uav_state_topic": self.health_topic,
                        },
                    )
                if self.prepared_sent:
                    self._set_state(TAKEOFF_RUNNING, "self_check_passed")
            return

        if self.state == TAKEOFF_RUNNING:
            if self._takeoff_reached():
                self._set_state(ACHIEVE, "takeoff_reached")
            return

        if self.state == ACHIEVE:
            if not self.achieve_sent:
                self.achieve_sent = self._send_gcs_event(
                    "achieve",
                    {
                        "height": self.current_uav_state.position.z
                        if self.current_uav_state is not None
                        else None
                    },
                )
            if self.achieve_sent:
                self._set_state(WAIT, "achieve_reported")
            return

        if self.state == LAND:
            if now_sec() - self.land_start_time >= self.land_reset_delay:
                self._reset_for_next_round("land_reset_delay")
            return

    def _status_payload(self) -> Dict[str, Any]:
        """构建发布到 status_topic 的仪表盘状态 JSON。"""
        t = now_sec()
        z = None
        if self.current_uav_state is not None:
            z = float(self.current_uav_state.position.z)
        return {
            "stamp": t,
            "run_id": self.run_id,
            "self_id": self.self_id,
            "role": self.role,
            "state": self.state,
            "prev_state": self.prev_state,
            "state_age": t - self.state_enter_time,
            "participants": self.participants,
            "prepared": sorted(self.prepared_uavs),
            "achieved": sorted(self.achieved_uavs),
            "all_prepared": self._all_prepared() if self.is_gcs else None,
            "all_achieved": self._all_achieved() if self.is_gcs else None,
            "health": {
                "topic": self.health_topic if not self.is_gcs else "",
                "ok": self._health_ok() if not self.is_gcs else True,
                "rate_hz": self._health_rate() if not self.is_gcs else None,
                "fresh": (
                    now_sec() - self.last_uav_state_time <= self.health_timeout
                    if self.last_uav_state_time > 0.0
                    else False
                )
                if not self.is_gcs
                else True,
                "valid": self.current_uav_state_valid if not self.is_gcs else True,
                "reason": self.current_uav_state_invalid_reason,
                "z": z,
            },
            "takeoff": {
                "height": self.takeoff_height,
                "duration": self.takeoff_duration,
                "started": self.takeoff_started,
                "elapsed": t - self.takeoff_start_time if self.takeoff_start_time > 0.0 else 0.0,
            },
            "rc": {
                "topic": self.rc_topic,
                "fresh": self.last_rc_time > 0.0 and t - self.last_rc_time <= 1.0,
                "takeoff_value": self._rc_value(self.rc_takeoff_channel),
                "land_value": self._rc_value(self.rc_land_channel),
                "abort_value": self._rc_value(self.rc_abort_channel),
            },
            "abort_reason": self.abort_reason,
        }

    def _publish_status(self):
        """按 status_rate_hz 发布仪表盘状态。"""
        t = now_sec()
        if t - self.last_status_pub_time < self.status_pub_period:
            return
        self.last_status_pub_time = t
        self.status_pub.publish(String(json_dumps(self._status_payload())))

    def spin(self):
        """FSM 主循环。

        每个周期的执行顺序：
        1. 读取操作员/键盘/RC 中止输入。
        2. 运行当前状态的用户周期性钩子。
        3. 运行按角色区分的状态转移逻辑。
        4. 发布供 Rich/rosbag 使用的状态。
        """
        rate = rospy.Rate(self.control_rate_hz)
        try:
            while not rospy.is_shutdown():
                keyboard_commands = self._keyboard_commands()
                if self.state != ABORT and self._manual_abort_requested(keyboard_commands):
                    self._enter_abort("manual_abort", {})

                self._run_state_action_hook()

                if self.state != ABORT:
                    if self.is_gcs:
                        self._tick_gcs(keyboard_commands)
                    else:
                        self._tick_uav()

                self._publish_status()
                rate.sleep()
        finally:
            if self.keyboard:
                self.keyboard.stop()


def main():
    rospy.init_node("coop_fsm")
    node = CoopFSM()
    node.spin()


if __name__ == "__main__":
    main()

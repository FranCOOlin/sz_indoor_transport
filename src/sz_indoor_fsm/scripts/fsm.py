#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Cooperative lift task finite-state machine.

This node intentionally keeps all high-level mission decisions in one Python
file so it is easy to inspect and modify during flight-test iteration.

Roles
-----
GCS:
    Central coordinator. It collects prepared/achieve events from UAV FSMs,
    sends JSON service commands to traj_router and UAVs, and waits for operator
    takeoff/land confirmation.

MASTER / SLAVE:
    Per-UAV task FSM. Each UAV checks its own UAVState stream, reports
    prepared/achieve to GCS, accepts state-change commands from GCS, and owns
    the local forced-landing hook for ABORT.

Where to add custom behavior
----------------------------
The safest extension points are near the bottom of CoopFSM:

    1. on_enter_<state>() hooks
       Called once immediately after a state transition.
       Use these for one-shot operations such as switching a controller mode,
       arming a subsystem, latching a trajectory id, or notifying another node.

    2. on_tick_<state>() hooks
       Called every control tick while the FSM remains in that state.
       Use these for periodic state-specific work such as publishing a hold
       command, checking an external condition, or feeding a watchdog.

    3. force_land()
       The reserved interface for ABORT forced landing. The current version
       only logs; wire your real landing/kill/RTL interface there.

Core transition logic is in _tick_gcs() and _tick_uav(). Try to keep those
methods focused on deciding *when to change state*. Put side effects in the
hooks above unless the side effect is itself the transition command.
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
# GCS-only wait state: GCS has skipped its own checks and is waiting for UAVs
# to report prepared, then operator confirmation to start takeoff.
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
    """Return ROS time as seconds. Keeping this wrapped makes tests/stubs easier."""
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
    """Small non-blocking stdin reader.

    This is used only when the FSM itself is launched in an interactive
    terminal. In normal GCS operation the Rich monitor calls the operator
    service instead, so FSM logging and keyboard handling can be separated.
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
    """Role-aware cooperative FSM.

    The class is deliberately not split into many subclasses yet. Most flight
    test changes need to see GCS/UAV behavior side-by-side, and role branching is
    still small enough to keep readable in one file.
    """

    def __init__(self):
        # ------------------------------------------------------------------
        # Identity and run scoping
        # ------------------------------------------------------------------
        # run_id is included in all JSON service payloads. It prevents stale
        # events from an old test run being accepted by a newly launched FSM.
        self.role = str(rospy.get_param("~role", ROLE_SLAVE)).lower().strip()
        if self.role not in [ROLE_GCS, ROLE_MASTER, ROLE_SLAVE]:
            raise RuntimeError("~role must be one of: gcs, master, slave")

        self.is_gcs = self.role == ROLE_GCS
        self.self_id = str(
            rospy.get_param("~self_id", "gcs" if self.is_gcs else "uav0")
        ).strip()
        self.master_id = str(rospy.get_param("~master_id", "uav0")).strip()
        self.run_id = str(rospy.get_param("~run_id", "coop_lift_test_001")).strip()

        self.participants = parse_csv(rospy.get_param("~participants", ""))
        if self.is_gcs and not self.participants:
            self.participants = [self.master_id]
        self.participants = [p for p in self.participants if p and p != "gcs"]

        # ------------------------------------------------------------------
        # Loop and status publishing rates
        # ------------------------------------------------------------------
        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 30.0))
        self.status_rate_hz = float(rospy.get_param("~status_rate_hz", 5.0))
        self.status_pub_period = 1.0 / max(self.status_rate_hz, 1e-6)
        self.last_status_pub_time = 0.0

        # ------------------------------------------------------------------
        # Mission timing and thresholds
        # ------------------------------------------------------------------
        # takeoff_duration is sent to traj_router and also used as a fallback
        # achieve timeout if height feedback is unavailable or delayed.
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
        # UAV health check from sz_indoor_controller/UAVState
        # ------------------------------------------------------------------
        # The default topic follows the simulator/observer convention found in
        # this workspace. Override ~uav_state_topic if your real estimator
        # publishes UAVState somewhere else.
        self.health_topic = str(
            rospy.get_param("~uav_state_topic", f"/{self.self_id}/quadrotor_feedback")
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
        # JSON service interfaces
        # ------------------------------------------------------------------
        # gcs_event_service:
        #   UAV/traj_router -> GCS event ingress, e.g. prepared/achieve/stop.
        # operator_service:
        #   Rich monitor / terminal UI -> GCS operator commands.
        # command_service:
        #   GCS -> this UAV command ingress, e.g. traj_following/land/abort.
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
        # Topics for observability and replay
        # ------------------------------------------------------------------
        # status_topic feeds the Rich dashboard. service_audit_topic mirrors
        # service requests/responses into rosbag-recordable String messages,
        # because ROS1 rosbag cannot directly record service calls.
        self.status_topic = str(
            rospy.get_param(
                "~status_topic",
                "/gcs/fsm/status" if self.is_gcs else f"/{self.self_id}/fsm/status",
            )
        )
        self.service_audit_topic = str(
            rospy.get_param(
                "~service_audit_topic",
                "/gcs/fsm/service_audit"
                if self.is_gcs
                else f"/{self.self_id}/fsm/service_audit",
            )
        )
        self.rc_topic = str(
            rospy.get_param(
                "~rc_in_topic",
                f"/{self.master_id}/mavros/rc/in"
                if self.is_gcs
                else f"/{self.self_id}/mavros/rc/in",
            )
        )

        # ------------------------------------------------------------------
        # Local keyboard and master RC input
        # ------------------------------------------------------------------
        # RC channel numbers are 1-based, matching RC transmitter labels.
        # Internally _rc_value() converts them to zero-based array indices.
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
        # Runtime state
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
        # ROS publishers, subscribers, and services
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
        """Change FSM state and run state-entry side effects.

        Keep transition bookkeeping here so every transition updates age,
        prev_state, logs, timers, and extension hooks consistently.
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
        """Validate the semantic content of UAVState.

        Current default checks:
        - position and velocity are finite numbers
        - optional attitude quaternion is finite and near unit norm

        If you later add fields to UAVState, extend this function with the
        additional validity rules. This is the place that corresponds to the
        TODO you mentioned before.
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
        """GCS service callback for external mission events.

        Expected event values:
        - prepared: a UAV passed self-check and is ready for the takeoff phase.
        - achieve: a UAV reached takeoff height/time condition.
        - stop: traj_router reports the main trajectory is finished.
        - abort: any node reports an emergency abort.
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
        """UAV service callback for commands sent by GCS."""
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
        """GCS operator command service.

        The Rich terminal monitor calls this service when the user presses
        t/l/a/r. Keeping operator input as a service avoids mixing dashboard
        rendering with mission transition logic.
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
        """Call a JsonCommand service and mirror the result to service_audit."""
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
        """Publish a rosbag-friendly audit record for each JSON service call."""
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
        """Send one UAV/traj event to GCS."""
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
        """Send one high-level command to the central traj_router service."""
        return self._call_json_service(self.traj_router_service, self._make_payload(cmd, extra))

    def _send_to_uav(self, uav_id: str, cmd: str, extra: Optional[Dict[str, Any]] = None) -> bool:
        """Send one command to a specific UAV FSM."""
        service = self.uav_command_service_template.format(uav_id=uav_id)
        payload = self._make_payload(cmd, {"target": uav_id, **(extra or {})})
        return self._call_json_service(service, payload)

    def _broadcast_to_uavs(self, cmd: str, extra: Optional[Dict[str, Any]] = None):
        """Send the same command to every participant listed in ~participants."""
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
        """Latch ABORT and fan it out to all relevant nodes.

        ABORT is intentionally sticky; it does not reset itself after a timer.
        This prevents a real emergency landing path from accidentally rearming
        the nominal mission flow.
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
        """Reserved forced-landing hook for UAV roles.

        Put the real forced landing operation here, for example:
        - call a MAVROS mode/land service
        - publish a controller emergency command
        - call your low-level safety node

        This function is called when the local UAV FSM enters ABORT. GCS does
        not execute local landing here; it broadcasts abort to traj_router/UAVs.
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
    # User extension hooks
    # ==================================================================
    # These hooks are intentionally no-op in the base implementation. They
    # are the recommended places for experiment-specific code:
    #
    #   - on_enter_*: one-shot work when a state is entered.
    #   - on_tick_* : periodic work while the FSM remains in a state.
    #
    # Examples:
    #   - switch a controller mode when entering TRAJ_FOLLOWING
    #   - publish a hold command every tick in WAIT
    #   - send a hardware trigger when entering LAND
    #
    # Keep transition decisions in _tick_gcs/_tick_uav. Keep side effects here.

    def _on_enter_state(self, new_state: str, old_state: str, reason: str):
        """Dispatch state-entry hooks.

        Add shared logging/metrics around hooks here if needed. For actual
        state-specific user code, edit the on_enter_<state>() methods below.
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
        """Run the periodic hook for the current state."""
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
        """Hook: entered PREPARE.

        Add one-shot reset work here. Typical additions: clear local controller
        state, reset a planner, or prepare sensors before self-check.
        """

    def on_enter_wait_takeoff(self, old_state: str, reason: str):
        """Hook: GCS entered WAIT_TAKEOFF.

        Add GCS-side UI/notification work here if you want an external display
        or buzzer to show "all checks pending / ready for takeoff".
        """

    def on_enter_takeoff_running(self, old_state: str, reason: str):
        """Hook: entered TAKEOFF_RUNNING.

        UAV note: before GCS sends takeoff_started, this state means "prepared
        and waiting, no local control action required". After takeoff_started,
        traj_router is expected to generate the actual takeoff trajectory.
        """

    def on_enter_achieve(self, old_state: str, reason: str):
        """Hook: UAV entered ACHIEVE.

        Add one-shot operations here if the UAV must latch any state exactly at
        takeoff completion before reporting achieve to GCS.
        """

    def on_enter_wait(self, old_state: str, reason: str):
        """Hook: UAV entered WAIT after reporting ACHIEVE.

        Add hold/idle setup here if your controller needs a mode switch while
        waiting for GCS to command TRAJ_FOLLOWING.
        """

    def on_enter_traj_following(self, old_state: str, reason: str):
        """Hook: entered TRAJ_FOLLOWING.

        This is the natural place to switch your local controller into tracking
        mode. The current FSM only changes state; it does not command the
        controller directly.
        """

    def on_enter_stop(self, old_state: str, reason: str):
        """Hook: GCS entered STOP.

        Add one-shot GCS-side "waiting for land confirmation" behavior here.
        """

    def on_enter_land(self, old_state: str, reason: str):
        """Hook: entered LAND.

        Add one-shot land setup here if nominal landing needs an interface call.
        Forced landing for ABORT belongs in force_land().
        """

    def on_enter_abort(self, old_state: str, reason: str):
        """Hook: entered ABORT.

        Add extra alarm/logging work here. The local forced landing hook is
        force_land(), which has already been called by _set_state().
        """

    def on_tick_prepare(self):
        """Hook: periodic work while in PREPARE."""

    def on_tick_wait_takeoff(self):
        """Hook: periodic GCS work while waiting for prepared + takeoff confirm."""

    def on_tick_takeoff_running(self):
        """Hook: periodic work while in TAKEOFF_RUNNING."""

    def on_tick_achieve(self):
        """Hook: periodic work while in ACHIEVE."""

    def on_tick_wait(self):
        """Hook: periodic UAV work while waiting for TRAJ_FOLLOWING."""

    def on_tick_traj_following(self):
        """Hook: periodic work while in TRAJ_FOLLOWING."""

    def on_tick_stop(self):
        """Hook: periodic GCS work while waiting for land confirmation."""

    def on_tick_land(self):
        """Hook: periodic work while in LAND."""

    def on_tick_abort(self):
        """Hook: periodic work while in ABORT."""

    def _start_takeoff_from_gcs(self):
        """GCS starts the takeoff phase after all UAVs are prepared."""
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
        """GCS starts the main trajectory-following phase after all UAVs achieve."""
        payload = {"participants": self.participants}
        self._send_to_traj_router("traj_following", payload)
        self._broadcast_to_uavs("traj_following", payload)
        self._set_state(TRAJ_FOLLOWING, "all_uavs_achieved")

    def _start_land_from_gcs(self):
        """GCS starts nominal landing after STOP and operator confirmation."""
        payload = {"participants": self.participants}
        self._send_to_traj_router("land", payload)
        self._broadcast_to_uavs("land", payload)
        self._set_state(LAND, "manual_land_confirmed")

    def _takeoff_reached(self) -> bool:
        """UAV takeoff completion condition.

        Default behavior accepts either:
        - height is within takeoff_reached_tolerance of takeoff_height, or
        - takeoff_duration + takeoff_timeout_margin elapsed after takeoff_started.

        Modify this function if your real vehicle uses another altitude sign
        convention or a better "takeoff complete" signal.
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
        """Clear per-round latches before returning to PREPARE."""
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
        """GCS transition logic.

        Keep this method focused on *when* the GCS should change state. Add
        one-shot/per-tick side effects in the hooks above.
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
        """MASTER/SLAVE transition logic.

        Keep this method focused on *when* the UAV should change state. Add
        controller or hardware commands in on_enter_* / on_tick_* hooks.
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
        """Build dashboard status JSON published on status_topic."""
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
        """Publish dashboard status at status_rate_hz."""
        t = now_sec()
        if t - self.last_status_pub_time < self.status_pub_period:
            return
        self.last_status_pub_time = t
        self.status_pub.publish(String(json_dumps(self._status_payload())))

    def spin(self):
        """Main FSM loop.

        Ordering per cycle:
        1. Read operator/keyboard/RC abort input.
        2. Run the user periodic hook for the current state.
        3. Run role-specific transition logic.
        4. Publish status for Rich/rosbag.
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

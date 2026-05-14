#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from typing import Any, Dict, List

import rospy
from std_msgs.msg import Bool, Float64MultiArray, String

from sz_indoor_controller.msg import UAVState
from sz_indoor_fsm.srv import JsonCommand, JsonCommandResponse


def now_sec() -> float:
    return rospy.Time.now().to_sec()


def parse_csv(value: Any) -> List[str]:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(v).strip() for v in value if str(v).strip()]
    return [part.strip() for part in str(value).split(",") if part.strip()]


def json_dumps(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


def json_loads(text: str) -> Dict[str, Any]:
    try:
        payload = json.loads(text)
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


class MockMultiUAV:
    def __init__(self):
        rospy.init_node("mock_multi_uav")

        self.run_id = str(rospy.get_param("~run_id", "coop_lift_test_001"))
        self.participants = parse_csv(
            rospy.get_param("~participants", "uav170,uav171,uav173,uav174")
        )
        self.gcs_event_service = str(
            rospy.get_param("~gcs_event_service", "/gcs/fsm/event")
        )
        self.command_service_template = str(
            rospy.get_param("~command_service_template", "/{uav_id}/fsm/command")
        )
        self.feedback_topic_template = str(
            rospy.get_param("~feedback_topic_template", "/{uav_id}/quadrotor_feedback")
        )
        self.state_topic_template = str(
            rospy.get_param("~state_topic_template", "/{uav_id}/quadrotor_state")
        )
        self.flag_topic_template = str(
            rospy.get_param("~flag_topic_template", "/{uav_id}/traj_generation_flag")
        )
        self.trajectory_request_topic = str(
            rospy.get_param("~trajectory_request_topic", "/trajectory_request")
        )
        self.rewrite_request_from = str(
            rospy.get_param("~rewrite_request_from", "main_trajectory")
        )
        self.rewrite_request_to = str(rospy.get_param("~rewrite_request_to", "trajectory1"))
        self.bridge_trajectory_request = bool(
            rospy.get_param("~bridge_trajectory_request", True)
        )
        self.auto_prepared = bool(rospy.get_param("~auto_prepared", True))
        self.auto_achieve = bool(rospy.get_param("~auto_achieve", True))
        self.auto_stop = bool(rospy.get_param("~auto_stop", True))
        self.prepared_delay = float(rospy.get_param("~prepared_delay", 1.0))
        self.achieve_delay = float(rospy.get_param("~achieve_delay", 1.0))
        self.follow_duration = float(rospy.get_param("~follow_duration", 8.0))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
        self.flag_rate_hz = float(rospy.get_param("~flag_rate_hz", 5.0))
        self.takeoff_height = float(rospy.get_param("~takeoff_height", 3.0))
        self.takeoff_duration = float(rospy.get_param("~takeoff_duration", 6.0))
        self.land_duration = float(rospy.get_param("~land_duration", 5.0))

        self.started_time = now_sec()
        self.takeoff_started_time = 0.0
        self.follow_started_time = 0.0
        self.land_started_time = 0.0
        self.mode = "idle"
        self.stop_sent = False
        self.last_flag_time = 0.0

        self.sent_prepared = {uav_id: False for uav_id in self.participants}
        self.sent_achieve = {uav_id: False for uav_id in self.participants}
        self.altitudes = {uav_id: 0.0 for uav_id in self.participants}
        self.land_start_altitudes = dict(self.altitudes)
        self.positions = self._initial_positions()

        self.feedback_pubs = {}
        self.state_pubs = {}
        self.flag_pubs = {}
        self.command_srvs = []
        for uav_id in self.participants:
            self.feedback_pubs[uav_id] = rospy.Publisher(
                self.feedback_topic_template.format(uav_id=uav_id),
                UAVState,
                queue_size=10,
            )
            self.state_pubs[uav_id] = rospy.Publisher(
                self.state_topic_template.format(uav_id=uav_id),
                Float64MultiArray,
                queue_size=10,
            )
            self.flag_pubs[uav_id] = rospy.Publisher(
                self.flag_topic_template.format(uav_id=uav_id),
                Bool,
                queue_size=10,
            )
            service_name = self.command_service_template.format(uav_id=uav_id)
            self.command_srvs.append(
                rospy.Service(
                    service_name,
                    JsonCommand,
                    lambda req, uid=uav_id: self._command_cb(uid, req),
                )
            )

        self.request_pub = rospy.Publisher(
            self.trajectory_request_topic, String, queue_size=10
        )
        rospy.Subscriber(
            self.trajectory_request_topic,
            String,
            self._trajectory_request_cb,
            queue_size=10,
        )
        rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1e-6)), self._timer_cb)

        rospy.loginfo(
            "[mock_multi_uav] participants=%s gcs_event=%s command_template=%s",
            ",".join(self.participants),
            self.gcs_event_service,
            self.command_service_template,
        )

    def _initial_positions(self) -> Dict[str, List[float]]:
        positions = {}
        count = max(len(self.participants), 1)
        radius = 1.0
        for index, uav_id in enumerate(self.participants):
            angle = 2.0 * math.pi * float(index) / float(count)
            positions[uav_id] = [radius * math.cos(angle), radius * math.sin(angle), 0.0]
        return positions

    def _make_event_payload(self, event: str, uav_id: str = "", extra=None):
        payload = {
            "cmd": event,
            "event": event,
            "run_id": self.run_id,
            "source": "mock_multi_uav",
            "stamp": now_sec(),
        }
        if uav_id:
            payload["uav_id"] = uav_id
            payload["self_id"] = uav_id
        if extra:
            payload.update(extra)
        return payload

    def _send_gcs_event(self, event: str, uav_id: str = "", extra=None) -> bool:
        payload = self._make_event_payload(event, uav_id, extra)
        try:
            rospy.wait_for_service(self.gcs_event_service, timeout=0.2)
            resp = rospy.ServiceProxy(self.gcs_event_service, JsonCommand)(
                json_dumps(payload)
            )
            if not resp.success:
                rospy.logwarn(
                    "[mock_multi_uav] GCS rejected event=%s uav=%s message=%s",
                    event,
                    uav_id,
                    resp.message,
                )
            return bool(resp.success)
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, "[mock_multi_uav] cannot send event=%s: %s", event, exc
            )
            return False

    def _command_cb(self, uav_id: str, req):
        payload = json_loads(req.json)
        if str(payload.get("run_id", self.run_id)) != self.run_id:
            return JsonCommandResponse(False, "run_id mismatch")

        cmd = str(payload.get("cmd", payload.get("event", ""))).lower().strip()
        rospy.loginfo("[mock_multi_uav] %s got cmd=%s", uav_id, cmd)

        if cmd == "takeoff_started":
            self.mode = "takeoff"
            self.takeoff_started_time = now_sec()
            self.stop_sent = False
            self.takeoff_height = float(payload.get("height", self.takeoff_height))
            self.takeoff_duration = max(
                float(payload.get("duration", self.takeoff_duration)), 0.1
            )
            return JsonCommandResponse(True, "mock takeoff_started accepted")

        if cmd == "traj_following":
            self.mode = "follow"
            self.follow_started_time = now_sec()
            self.stop_sent = False
            self._publish_ready_flags(force=True)
            return JsonCommandResponse(True, "mock traj_following accepted")

        if cmd == "land":
            self.mode = "land"
            self.land_started_time = now_sec()
            self.land_start_altitudes = dict(self.altitudes)
            return JsonCommandResponse(True, "mock land accepted")

        if cmd == "abort":
            self.mode = "abort"
            return JsonCommandResponse(True, "mock abort accepted")

        if cmd == "reset":
            self._reset()
            return JsonCommandResponse(True, "mock reset accepted")

        return JsonCommandResponse(False, f"unknown cmd: {cmd}")

    def _reset(self):
        self.mode = "idle"
        self.stop_sent = False
        self.takeoff_started_time = 0.0
        self.follow_started_time = 0.0
        self.land_started_time = 0.0
        self.sent_prepared = {uav_id: False for uav_id in self.participants}
        self.sent_achieve = {uav_id: False for uav_id in self.participants}
        self.altitudes = {uav_id: 0.0 for uav_id in self.participants}
        self.land_start_altitudes = dict(self.altitudes)
        self.started_time = now_sec()

    def _trajectory_request_cb(self, msg: String):
        if not self.bridge_trajectory_request:
            return
        if msg.data != self.rewrite_request_from:
            return

        rospy.loginfo(
            "[mock_multi_uav] bridge trajectory request %s -> %s",
            self.rewrite_request_from,
            self.rewrite_request_to,
        )
        self._publish_ready_flags(force=True)
        bridged = String()
        bridged.data = self.rewrite_request_to
        self.request_pub.publish(bridged)

    def _publish_ready_flags(self, force=False):
        t = now_sec()
        if not force and t - self.last_flag_time < 1.0 / max(self.flag_rate_hz, 1e-6):
            return
        self.last_flag_time = t
        msg = Bool()
        msg.data = True
        for pub in self.flag_pubs.values():
            pub.publish(msg)

    def _update_altitudes(self):
        t = now_sec()
        if self.mode == "takeoff":
            alpha = min(
                max((t - self.takeoff_started_time) / self.takeoff_duration, 0.0), 1.0
            )
            for uav_id in self.participants:
                self.altitudes[uav_id] = self.takeoff_height * alpha
        elif self.mode == "land":
            alpha = min(max((t - self.land_started_time) / self.land_duration, 0.0), 1.0)
            for uav_id in self.participants:
                self.altitudes[uav_id] = self.land_start_altitudes[uav_id] * (1.0 - alpha)

    def _publish_states(self):
        for uav_id in self.participants:
            x, y, _ = self.positions[uav_id]
            z = self.altitudes[uav_id]

            feedback = UAVState()
            feedback.position.x = x
            feedback.position.y = y
            feedback.position.z = z
            feedback.attitude.w = 1.0
            self.feedback_pubs[uav_id].publish(feedback)

            state = Float64MultiArray()
            state.data = [x, y, z]
            self.state_pubs[uav_id].publish(state)

    def _drive_events(self):
        t = now_sec()
        if self.auto_prepared and t - self.started_time >= self.prepared_delay:
            for uav_id in self.participants:
                if not self.sent_prepared[uav_id]:
                    if self._send_gcs_event("prepared", uav_id):
                        self.sent_prepared[uav_id] = True

        if (
            self.auto_achieve
            and self.mode == "takeoff"
            and t - self.takeoff_started_time >= self.achieve_delay
        ):
            for uav_id in self.participants:
                if not self.sent_achieve[uav_id]:
                    if self._send_gcs_event(
                        "achieve", uav_id, {"height": self.altitudes[uav_id]}
                    ):
                        self.sent_achieve[uav_id] = True

        if (
            self.auto_stop
            and self.mode == "follow"
            and not self.stop_sent
            and t - self.follow_started_time >= self.follow_duration
        ):
            if self._send_gcs_event("stop", "", {"reason": "mock_follow_complete"}):
                self.stop_sent = True

    def _timer_cb(self, _event):
        self._update_altitudes()
        self._publish_states()
        if self.mode in ["follow", "takeoff"]:
            self._publish_ready_flags()
        self._drive_events()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        MockMultiUAV().run()
    except rospy.ROSInterruptException:
        pass

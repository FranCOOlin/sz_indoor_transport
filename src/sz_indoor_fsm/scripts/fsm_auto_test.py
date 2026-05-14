#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import sys
from typing import Any, Dict, List

import rospy
from mavros_msgs.msg import RCIn
from std_msgs.msg import String

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


class FSMAutoTest:
    def __init__(self):
        self.run_id = str(rospy.get_param("~run_id", "coop_lift_test_001"))
        self.participants = parse_csv(rospy.get_param("~participants", "uav0,uav1"))
        self.state_topic_template = str(
            rospy.get_param("~state_topic_template", "/{uav_id}/quadrotor_feedback")
        )
        self.status_topic = str(rospy.get_param("~gcs_status_topic", "/gcs/fsm/status"))
        self.gcs_event_service = str(
            rospy.get_param("~gcs_event_service", "/gcs/fsm/event")
        )
        self.traj_router_service = str(
            rospy.get_param("~traj_router_service", "/traj_router/command")
        )
        self.rc_topic = str(rospy.get_param("~rc_topic", "/uav0/mavros/rc/in"))

        self.state_pub_rate_hz = float(rospy.get_param("~state_pub_rate_hz", 30.0))
        self.rc_pub_rate_hz = float(rospy.get_param("~rc_pub_rate_hz", 20.0))
        self.takeoff_trigger_delay = float(rospy.get_param("~takeoff_trigger_delay", 0.5))
        self.rc_pulse_sec = float(rospy.get_param("~rc_pulse_sec", 1.0))
        self.follow_duration = float(rospy.get_param("~follow_duration", 2.0))
        self.land_duration = float(rospy.get_param("~land_duration", 2.0))
        self.test_timeout = float(rospy.get_param("~test_timeout", 60.0))
        self.manual_operator = bool(rospy.get_param("~manual_operator", False))

        self.rc_takeoff_channel = int(rospy.get_param("~rc_takeoff_channel", 12))
        self.rc_takeoff_high = int(rospy.get_param("~rc_takeoff_high", 1900))
        self.rc_land_channel = int(rospy.get_param("~rc_land_channel", 5))
        self.rc_land_high = int(rospy.get_param("~rc_land_high", 1900))
        self.rc_low = int(rospy.get_param("~rc_low", 1000))

        self.state_pubs = {
            uav_id: rospy.Publisher(
                self.state_topic_template.format(uav_id=uav_id),
                UAVState,
                queue_size=10,
            )
            for uav_id in self.participants
        }
        self.rc_pub = rospy.Publisher(self.rc_topic, RCIn, queue_size=10)
        self.traj_srv = rospy.Service(
            self.traj_router_service, JsonCommand, self._traj_router_cb
        )
        rospy.Subscriber(self.status_topic, String, self._gcs_status_cb, queue_size=20)

        self.gcs_status = {}
        self.gcs_state_history = []
        self.altitudes = {uav_id: 0.0 for uav_id in self.participants}
        self.mode = "idle"
        self.takeoff_start_time = 0.0
        self.takeoff_height = 0.0
        self.takeoff_duration = 1.0
        self.follow_start_time = 0.0
        self.land_start_time = 0.0
        self.land_start_altitudes = dict(self.altitudes)

        self.takeoff_pulse_until = 0.0
        self.land_pulse_until = 0.0
        self.takeoff_triggered = False
        self.land_triggered = False
        self.stop_sent = False
        self.seen_land = False
        self.started_time = now_sec()
        self.last_state_pub_time = 0.0
        self.last_rc_pub_time = 0.0

        rospy.loginfo(
            "[fsm_auto_test] participants=%s traj_service=%s status=%s manual_operator=%s",
            ",".join(self.participants),
            self.traj_router_service,
            self.status_topic,
            self.manual_operator,
        )

    def _traj_router_cb(self, req):
        payload = json_loads(req.json)
        if payload.get("run_id", self.run_id) != self.run_id:
            return JsonCommandResponse(False, "run_id mismatch")
        cmd = str(payload.get("cmd", "")).lower().strip()

        if cmd == "takeoff":
            self.mode = "takeoff"
            self.takeoff_start_time = now_sec()
            self.takeoff_height = float(payload.get("height", 3.0))
            self.takeoff_duration = max(float(payload.get("duration", 6.0)), 0.1)
            rospy.loginfo(
                "[fsm_auto_test] fake traj_router got takeoff height=%.2f duration=%.2f",
                self.takeoff_height,
                self.takeoff_duration,
            )
            return JsonCommandResponse(True, "fake takeoff accepted")

        if cmd == "traj_following":
            self.mode = "follow"
            self.follow_start_time = now_sec()
            self.stop_sent = False
            rospy.loginfo("[fsm_auto_test] fake traj_router got traj_following")
            return JsonCommandResponse(True, "fake traj_following accepted")

        if cmd == "land":
            self.mode = "land"
            self.land_start_time = now_sec()
            self.land_start_altitudes = dict(self.altitudes)
            rospy.loginfo("[fsm_auto_test] fake traj_router got land")
            return JsonCommandResponse(True, "fake land accepted")

        if cmd == "abort":
            self.mode = "abort"
            rospy.logwarn("[fsm_auto_test] fake traj_router got abort")
            return JsonCommandResponse(True, "fake abort accepted")

        return JsonCommandResponse(False, f"unknown cmd {cmd}")

    def _gcs_status_cb(self, msg: String):
        payload = json_loads(msg.data)
        if not payload:
            return
        self.gcs_status = payload
        state = str(payload.get("state", ""))
        if not self.gcs_state_history or self.gcs_state_history[-1] != state:
            self.gcs_state_history.append(state)
            rospy.loginfo("[fsm_auto_test] GCS state -> %s", state)
        if state == "LAND":
            self.seen_land = True

    def _call_gcs_event(self, event: str, extra=None):
        payload = {
            "cmd": event,
            "event": event,
            "run_id": self.run_id,
            "source": "fsm_auto_test",
            "stamp": now_sec(),
        }
        if extra:
            payload.update(extra)
        try:
            rospy.wait_for_service(self.gcs_event_service, timeout=1.0)
            resp = rospy.ServiceProxy(self.gcs_event_service, JsonCommand)(
                json_dumps(payload)
            )
            return bool(resp.success)
        except Exception as exc:
            rospy.logwarn("[fsm_auto_test] gcs event call failed: %s", exc)
            return False

    def _publish_states(self):
        t = now_sec()
        if t - self.last_state_pub_time < 1.0 / max(self.state_pub_rate_hz, 1e-6):
            return
        self.last_state_pub_time = t

        if self.mode == "takeoff":
            alpha = min(max((t - self.takeoff_start_time) / self.takeoff_duration, 0.0), 1.0)
            for uav_id in self.participants:
                self.altitudes[uav_id] = self.takeoff_height * alpha
        elif self.mode == "land":
            alpha = min(max((t - self.land_start_time) / self.land_duration, 0.0), 1.0)
            for uav_id in self.participants:
                self.altitudes[uav_id] = self.land_start_altitudes[uav_id] * (1.0 - alpha)

        for uav_id, pub in self.state_pubs.items():
            msg = UAVState()
            msg.position.z = self.altitudes[uav_id]
            msg.attitude.w = 1.0
            pub.publish(msg)

    def _publish_rc(self):
        t = now_sec()
        if t - self.last_rc_pub_time < 1.0 / max(self.rc_pub_rate_hz, 1e-6):
            return
        self.last_rc_pub_time = t
        msg = RCIn()
        msg.channels = [self.rc_low] * 16
        if t < self.takeoff_pulse_until:
            msg.channels[self.rc_takeoff_channel - 1] = self.rc_takeoff_high
        if t < self.land_pulse_until:
            msg.channels[self.rc_land_channel - 1] = self.rc_land_high
        msg.rssi = 255
        self.rc_pub.publish(msg)

    def _drive_test(self):
        t = now_sec()
        gcs_state = str(self.gcs_status.get("state", ""))
        all_prepared = bool(self.gcs_status.get("all_prepared", False))

        if (
            not self.manual_operator
            and
            not self.takeoff_triggered
            and gcs_state == "WAIT_TAKEOFF"
            and all_prepared
            and t - safe_float(self.gcs_status.get("stamp", t)) >= 0.0
            and t - self.started_time >= self.takeoff_trigger_delay
        ):
            self.takeoff_triggered = True
            self.takeoff_pulse_until = t + self.rc_pulse_sec
            rospy.loginfo("[fsm_auto_test] pulsing takeoff RC channel")

        if self.mode == "follow" and not self.stop_sent:
            if t - self.follow_start_time >= self.follow_duration:
                if self._call_gcs_event("stop", {"reason": "auto_test_follow_done"}):
                    self.stop_sent = True
                    rospy.loginfo("[fsm_auto_test] sent stop event")

        if not self.manual_operator and not self.land_triggered and gcs_state == "STOP":
            self.land_triggered = True
            self.land_pulse_until = t + self.rc_pulse_sec
            rospy.loginfo("[fsm_auto_test] pulsing land RC channel")

        if self.manual_operator and gcs_state == "LAND":
            self.land_triggered = True

        if self.land_triggered and self.seen_land and gcs_state in ["PREPARE", "WAIT_TAKEOFF"]:
            rospy.loginfo(
                "[fsm_auto_test] PASS state_history=%s",
                " -> ".join(self.gcs_state_history),
            )
            sys.exit(0)

        if t - self.started_time > self.test_timeout:
            rospy.logerr(
                "[fsm_auto_test] FAIL timeout state=%s history=%s",
                gcs_state,
                " -> ".join(self.gcs_state_history),
            )
            sys.exit(2)

    def spin(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self._publish_states()
            self._publish_rc()
            self._drive_test()
            rate.sleep()


def safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return default


def main():
    rospy.init_node("fsm_auto_test")
    FSMAutoTest().spin()


if __name__ == "__main__":
    main()

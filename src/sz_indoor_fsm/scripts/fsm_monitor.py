#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import select
import sys
import termios
import threading
import tty
from collections import deque
from typing import Any, Dict

import rospy
from std_msgs.msg import String

from sz_indoor_fsm.srv import JsonCommand

try:
    from rich.console import Console, Group
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
except ImportError as exc:
    raise SystemExit(
        "fsm_monitor.py needs the Python package 'rich'. Install it with: pip3 install rich"
    ) from exc


def parse_csv(value: Any):
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [str(v).strip() for v in value if str(v).strip()]
    return [part.strip() for part in str(value).split(",") if part.strip()]


def safe_float(value, default=0.0):
    try:
        return float(value)
    except Exception:
        return default


def json_dumps(payload: Dict[str, Any]) -> str:
    return json.dumps(payload, sort_keys=True, separators=(",", ":"))


class KeyboardReader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.enabled = bool(sys.stdin and sys.stdin.isatty())
        self._lock = threading.Lock()
        self._keys = []
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

    def pop_all(self):
        with self._lock:
            keys = list(self._keys)
            self._keys = []
        return keys

    def stop(self):
        self._stop = True


class FSMMonitor:
    def __init__(self):
        self.console = Console()
        self.screen = bool(rospy.get_param("~screen", False))
        self.refresh_hz = float(rospy.get_param("~refresh_hz", 4.0))
        self.run_id = str(rospy.get_param("~run_id", "coop_lift_test_001"))
        self.operator_service = str(
            rospy.get_param("~operator_service", "/gcs/fsm/operator")
        )
        self.enable_keyboard = bool(rospy.get_param("~enable_keyboard", True))
        self.max_events = int(rospy.get_param("~max_events", 8))
        participants = parse_csv(rospy.get_param("~participants", ""))
        include_gcs = bool(rospy.get_param("~include_gcs", True))

        topics = parse_csv(rospy.get_param("~status_topics", ""))
        if not topics:
            if include_gcs:
                topics.append("/gcs/fsm/status")
            topics.extend([f"/{uav_id}/fsm/status" for uav_id in participants])

        audit_topics = parse_csv(rospy.get_param("~audit_topics", ""))
        if not audit_topics:
            if include_gcs:
                audit_topics.append("/gcs/fsm/service_audit")
            audit_topics.extend([f"/{uav_id}/fsm/service_audit" for uav_id in participants])

        self.rows: Dict[str, Dict[str, Any]] = {}
        self.topic_by_name: Dict[str, str] = {}
        self.last_state_by_name: Dict[str, str] = {}
        self.events = deque(maxlen=max(self.max_events, 1))
        self.last_command = "keys: t takeoff | l land | a abort | r reset"
        self.keyboard = KeyboardReader() if self.enable_keyboard else None
        if self.keyboard:
            self.keyboard.start()
        for topic in topics:
            rospy.Subscriber(topic, String, self._status_cb, callback_args=topic, queue_size=20)
        for topic in audit_topics:
            rospy.Subscriber(topic, String, self._audit_cb, callback_args=topic, queue_size=50)

        rospy.loginfo("[fsm_monitor] watching topics: %s", ", ".join(topics))
        rospy.loginfo("[fsm_monitor] watching audit topics: %s", ", ".join(audit_topics))

    def _status_cb(self, msg: String, topic: str):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        name = str(payload.get("self_id", topic)).strip() or topic
        payload["_topic"] = topic
        payload["_recv_time"] = rospy.Time.now().to_sec()
        self.rows[name] = payload
        self.topic_by_name[name] = topic
        state = str(payload.get("state", ""))
        old_state = self.last_state_by_name.get(name)
        if state and old_state != state:
            self.last_state_by_name[name] = state
            self._add_event(f"{name} -> {state}")

    def _audit_cb(self, msg: String, topic: str):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        node = str(payload.get("self_id", topic))
        direction = str(payload.get("direction", "?"))
        service = str(payload.get("service", ""))
        success = payload.get("success", None)
        body = payload.get("payload", {}) or {}
        cmd = str(body.get("cmd", body.get("event", "")))
        marker = "ok" if success else "fail"
        service_leaf = service.rstrip("/").split("/")[-1] if service else "service"
        self._add_event(f"{node} {direction} {service_leaf} {cmd} {marker}")

    def _add_event(self, text: str):
        stamp = rospy.Time.now().to_sec()
        self.events.append((stamp, text))

    def _style_state(self, state: str) -> str:
        if state == "ABORT":
            return f"[bold red]{state}[/bold red]"
        if state == "LAND":
            return f"[yellow]{state}[/yellow]"
        if state in ["TRAJ_FOLLOWING", "TAKEOFF_RUNNING"]:
            return f"[green]{state}[/green]"
        if state in ["WAIT_TAKEOFF", "WAIT", "STOP"]:
            return f"[cyan]{state}[/cyan]"
        return state

    def _render_status_table(self):
        table = Table(
            title=f"SZ Indoor Transport FSM  [{self.last_command}]",
            expand=True,
        )
        table.add_column("Node", no_wrap=True)
        table.add_column("Role", no_wrap=True)
        table.add_column("State", no_wrap=True)
        table.add_column("Age", justify="right")
        table.add_column("Health", justify="right")
        table.add_column("Z", justify="right")
        table.add_column("Prepared")
        table.add_column("Achieved")
        table.add_column("RC")
        table.add_column("Topic")

        now = rospy.Time.now().to_sec()
        for name in sorted(self.rows):
            payload = self.rows[name]
            health = payload.get("health", {}) or {}
            rc = payload.get("rc", {}) or {}
            state = str(payload.get("state", "unknown"))
            role = str(payload.get("role", ""))
            age = safe_float(payload.get("state_age", 0.0))
            recv_age = now - safe_float(payload.get("_recv_time", now))
            rate = health.get("rate_hz")
            health_ok = bool(health.get("ok", True))
            health_text = "ok" if health_ok else str(health.get("reason", "bad"))
            if rate is not None:
                health_text = f"{health_text} {safe_float(rate):.1f}Hz"
            if recv_age > 2.0:
                health_text = f"[red]stale {recv_age:.1f}s[/red]"
            elif not health_ok:
                health_text = f"[yellow]{health_text}[/yellow]"

            z = health.get("z")
            z_text = "-" if z is None else f"{safe_float(z):.2f}"
            prepared = ",".join(payload.get("prepared", []) or [])
            achieved = ",".join(payload.get("achieved", []) or [])
            rc_text = "fresh" if rc.get("fresh", False) else "stale"
            if recv_age > 2.0:
                rc_text = "-"

            table.add_row(
                name,
                role,
                self._style_state(state),
                f"{age:.1f}s",
                health_text,
                z_text,
                prepared or "-",
                achieved or "-",
                rc_text,
                str(payload.get("_topic", "")),
            )
        return table

    def _render_gcs_info(self):
        gcs = self.rows.get("gcs", {})
        takeoff = gcs.get("takeoff", {}) or {}
        rc = gcs.get("rc", {}) or {}
        info = Table.grid(expand=True)
        info.add_column(justify="left")
        info.add_column(justify="left")
        state = str(gcs.get("state", "-"))
        prepared = ",".join(gcs.get("prepared", []) or []) or "-"
        achieved = ",".join(gcs.get("achieved", []) or []) or "-"
        all_prepared = str(gcs.get("all_prepared", "-"))
        all_achieved = str(gcs.get("all_achieved", "-"))
        takeoff_text = "{:.1f}/{:.1f}s h={:.2f}".format(
            safe_float(takeoff.get("elapsed", 0.0)),
            safe_float(takeoff.get("duration", 0.0)),
            safe_float(takeoff.get("height", 0.0)),
        )
        rc_text = "takeoff={} land={} abort={} {}".format(
            rc.get("takeoff_value", "-"),
            rc.get("land_value", "-"),
            rc.get("abort_value", "-"),
            "fresh" if rc.get("fresh", False) else "stale",
        )
        info.add_row("[bold]GCS[/bold]", f"state={state}")
        info.add_row("prepared", f"{prepared}  all={all_prepared}")
        info.add_row("achieved", f"{achieved}  all={all_achieved}")
        info.add_row("takeoff", takeoff_text)
        info.add_row("rc", rc_text)
        info.add_row("last command", self.last_command)
        return Panel(info, title="GCS Info", expand=True)

    def _render_events(self):
        table = Table(title="Recent Events", expand=True)
        table.add_column("Age", justify="right", no_wrap=True)
        table.add_column("Event")
        now = rospy.Time.now().to_sec()
        for stamp, text in list(self.events)[-self.max_events:]:
            table.add_row(f"{now - stamp:.1f}s", text)
        return table

    def _render(self):
        return Group(
            self._render_status_table(),
            self._render_gcs_info(),
            self._render_events(),
        )

    def _send_operator_command(self, cmd: str):
        payload = {
            "cmd": cmd,
            "event": cmd,
            "run_id": self.run_id,
            "source": "fsm_monitor",
            "stamp": rospy.Time.now().to_sec(),
        }
        try:
            rospy.wait_for_service(self.operator_service, timeout=0.2)
            resp = rospy.ServiceProxy(self.operator_service, JsonCommand)(
                json_dumps(payload)
            )
            status = "ok" if resp.success else "rejected"
            self.last_command = f"{cmd}: {status} {resp.message}"
            self._add_event(f"operator {cmd} {status}: {resp.message}")
        except Exception as exc:
            self.last_command = f"{cmd}: failed {exc}"
            self._add_event(f"operator {cmd} failed: {exc}")

    def _handle_keys(self):
        if not self.keyboard:
            return
        for key in self.keyboard.pop_all():
            if key == "t":
                self._send_operator_command("takeoff")
            elif key == "l":
                self._send_operator_command("land")
            elif key == "a":
                self._send_operator_command("abort")
            elif key == "r":
                self._send_operator_command("reset")

    def spin(self):
        rate = rospy.Rate(max(self.refresh_hz, 0.5))
        try:
            with Live(
                self._render(),
                console=self.console,
                refresh_per_second=max(self.refresh_hz, 0.5),
                screen=self.screen,
            ) as live:
                while not rospy.is_shutdown():
                    self._handle_keys()
                    live.update(self._render())
                    rate.sleep()
        finally:
            if self.keyboard:
                self.keyboard.stop()


def main():
    rospy.init_node("fsm_monitor")
    FSMMonitor().spin()


if __name__ == "__main__":
    main()

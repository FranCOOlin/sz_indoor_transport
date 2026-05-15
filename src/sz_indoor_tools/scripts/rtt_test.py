#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from collections import deque
import threading
from typing import Deque, Dict, List

import rospy
from std_msgs.msg import Float64, Header


ROLE_GCS = "gcs"
ROLE_UAV = "uav"


def percentile(values: List[float], pct: float) -> float:
    if not values:
        return 0.0

    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * pct))
    return ordered[max(0, min(index, len(ordered) - 1))]


class RttTestNode:
    def __init__(self):
        rospy.init_node("rtt_test")

        self.role = str(rospy.get_param("~role", ROLE_GCS)).lower()
        self.uav_id = str(rospy.get_param("~uav_id", "uav0"))
        self.ping_topic = str(rospy.get_param("~ping_topic", "/rtt_test/ping"))
        self.pong_topic = str(rospy.get_param("~pong_topic", "/rtt_test/pong"))
        self.queue_size = int(rospy.get_param("~queue_size", 100))
        self.report_interval = max(0.1, float(rospy.get_param("~report_interval", 1.0)))

        if self.role == ROLE_GCS:
            self._init_gcs()
        elif self.role == ROLE_UAV:
            self._init_uav()
        else:
            rospy.logerr("Unsupported role '%s'. Use 'gcs' or 'uav'.", self.role)
            rospy.signal_shutdown("invalid role")

    def _init_gcs(self):
        self.rate_hz = max(0.1, float(rospy.get_param("~rate_hz", 10.0)))
        self.timeout_sec = max(0.01, float(rospy.get_param("~timeout_sec", 1.0)))
        self.window_size = max(1, int(rospy.get_param("~window_size", 100)))
        self.warn_rtt_ms = float(rospy.get_param("~warn_rtt_ms", 0.0))
        self.latency_topic = str(rospy.get_param("~latency_topic", "/rtt_test/rtt_ms"))

        self.seq = 0
        self.sent_count = 0
        self.recv_count = 0
        self.timeout_count = 0
        self.late_count = 0
        self.last_rtt_ms = 0.0
        self.pending: Dict[int, float] = {}
        self.samples_ms: Deque[float] = deque(maxlen=self.window_size)
        self.lock = threading.RLock()

        self.ping_pub = rospy.Publisher(
            self.ping_topic, Header, queue_size=self.queue_size
        )
        self.latency_pub = rospy.Publisher(
            self.latency_topic, Float64, queue_size=self.queue_size
        )
        self.pong_sub = rospy.Subscriber(
            self.pong_topic, Header, self._handle_pong, queue_size=self.queue_size
        )

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._send_ping)
        rospy.Timer(rospy.Duration(self.report_interval), self._report_gcs)

        rospy.loginfo(
            "RTT GCS started for %s: ping=%s pong=%s rate=%.2fHz timeout=%.3fs",
            self.uav_id,
            self.ping_topic,
            self.pong_topic,
            self.rate_hz,
            self.timeout_sec,
        )

    def _init_uav(self):
        self.echo_count = 0
        self.last_seq = None
        self.lock = threading.RLock()

        self.pong_pub = rospy.Publisher(
            self.pong_topic, Header, queue_size=self.queue_size
        )
        self.ping_sub = rospy.Subscriber(
            self.ping_topic, Header, self._handle_ping, queue_size=self.queue_size
        )
        rospy.Timer(rospy.Duration(self.report_interval), self._report_uav)

        rospy.loginfo(
            "RTT UAV echo started for %s: ping=%s pong=%s",
            self.uav_id,
            self.ping_topic,
            self.pong_topic,
        )

    def _send_ping(self, _event):
        now = rospy.Time.now()
        now_sec = now.to_sec()
        self._prune_timeouts(now_sec)

        msg = Header()
        msg.stamp = now
        msg.frame_id = self.uav_id

        with self.lock:
            msg.seq = self.seq
            self.pending[self.seq] = now_sec
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            self.sent_count += 1

        self.ping_pub.publish(msg)

    def _handle_pong(self, msg: Header):
        now = rospy.Time.now()
        now_sec = now.to_sec()

        rtt_ms = max(0.0, (now - msg.stamp).to_sec() * 1000.0)

        with self.lock:
            send_time = self.pending.pop(msg.seq, None)
            if send_time is None:
                self.late_count += 1

            self.last_rtt_ms = rtt_ms
            self.samples_ms.append(rtt_ms)
            self.recv_count += 1

        self.latency_pub.publish(Float64(data=rtt_ms))

        if self.warn_rtt_ms > 0.0 and rtt_ms > self.warn_rtt_ms:
            rospy.logwarn_throttle(
                1.0,
                "RTT %s high: seq=%u rtt=%.2fms threshold=%.2fms",
                self.uav_id,
                msg.seq,
                rtt_ms,
                self.warn_rtt_ms,
            )

        self._prune_timeouts(now_sec)

    def _handle_ping(self, msg: Header):
        with self.lock:
            self.last_seq = msg.seq
            self.echo_count += 1

        self.pong_pub.publish(msg)

    def _prune_timeouts(self, now_sec: float):
        with self.lock:
            timed_out = [
                seq
                for seq, sent_sec in self.pending.items()
                if now_sec - sent_sec > self.timeout_sec
            ]
            for seq in timed_out:
                self.pending.pop(seq, None)
            self.timeout_count += len(timed_out)

    def _report_gcs(self, _event):
        self._prune_timeouts(rospy.Time.now().to_sec())
        with self.lock:
            samples = list(self.samples_ms)
            sent_count = self.sent_count
            recv_count = self.recv_count
            timeout_count = self.timeout_count
            late_count = self.late_count
            pending_count = len(self.pending)
            last_rtt_ms = self.last_rtt_ms

        loss_count = max(0, sent_count - recv_count - pending_count)
        loss_ratio = 100.0 * loss_count / max(1, sent_count)

        if samples:
            avg_ms = sum(samples) / len(samples)
            min_ms = min(samples)
            max_ms = max(samples)
            p50_ms = percentile(samples, 0.50)
            p95_ms = percentile(samples, 0.95)
        else:
            avg_ms = min_ms = max_ms = p50_ms = p95_ms = 0.0

        rospy.loginfo(
            (
                "RTT %s sent=%d recv=%d pending=%d timeout=%d late=%d "
                "loss=%.1f%% last=%.2fms avg=%.2fms min=%.2fms "
                "p50=%.2fms p95=%.2fms max=%.2fms"
            ),
            self.uav_id,
            sent_count,
            recv_count,
            pending_count,
            timeout_count,
            late_count,
            loss_ratio,
            last_rtt_ms,
            avg_ms,
            min_ms,
            p50_ms,
            p95_ms,
            max_ms,
        )

    def _report_uav(self, _event):
        with self.lock:
            echo_count = self.echo_count
            last_seq = self.last_seq

        rospy.loginfo(
            "RTT %s echo_count=%d last_seq=%s ping=%s pong=%s",
            self.uav_id,
            echo_count,
            str(last_seq),
            self.ping_topic,
            self.pong_topic,
        )


if __name__ == "__main__":
    try:
        RttTestNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

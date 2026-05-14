#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rospy
from std_msgs.msg import String, Float64MultiArray, Bool
from sz_indoor_fsm.srv import JsonCommand, JsonCommandResponse
from sz_indoor_controller.msg import TrajPoint


class TrajRouterNode:
    """
    轨迹路由器节点 - 对接 GCS FSM
    """
    def __init__(self):
        rospy.init_node('traj_router', anonymous=True)
        
        # 参数配置
        self.takeoff_duration = rospy.get_param("~takeoff_duration", 6.0)
        self.landing_duration = rospy.get_param("~landing_duration", 5.0)
        self.target_height = rospy.get_param("~target_height", 1.0)
        self.uav_id = rospy.get_param("~uav_id", "/uav1")
        
        # 状态定义
        self.STATE_IDLE = "idle"
        self.STATE_TAKEOFF = "takeoff"
        self.STATE_HOVER = "hover"
        self.STATE_TRAJ_FOLLOWING = "traj_following"
        self.STATE_LANDING = "landing"
        
        self.current_state = self.STATE_IDLE
        self.state_start_time = rospy.Time.now()
        
        # 实时位置
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # 锁定位置
        self.lock_x = 0.0
        self.lock_y = 0.0
        self.lock_z = 0.0
        
        # 允许轨迹接收标志
        self.trajectory_active = False
        self.last_point_time = rospy.Time.now()
        self.trajectory_started = False
        
        # ========== 向下端口 ==========
        self.trajectory_request_pub = rospy.Publisher('/trajectory_request', String, queue_size=10)
        self.flag_pub = rospy.Publisher(self.uav_id + '/traj_generation_flag', Bool, queue_size=10)
        
        # ========== 从下端口接收轨迹 ==========
        traj_topic = self.uav_id + "/planning/traj_point"
        rospy.Subscriber(traj_topic, TrajPoint, self.trajectory_data_callback)
        
        # ========== 获取当前位置 ==========
        quadrotor_state_topic = self.uav_id + "/quadrotor_state"
        rospy.Subscriber(quadrotor_state_topic, Float64MultiArray, self.quadrotor_state_callback)
        
        # ========== 发布轨迹给执行机构 ==========
        self.trajectory_pub = rospy.Publisher(self.uav_id + "/trajectory", TrajPoint, queue_size=10)
        
        # ========== 向上端口 service ==========
        self.server = rospy.Service('/traj_router/command', JsonCommand, self.handle_command)
        
        # ========== 合并的定时器（200Hz） ==========
        rospy.Timer(rospy.Duration(0.005), self.main_timer_callback)
        
        rospy.loginfo("Traj Router 已启动")
    
    def quadrotor_state_callback(self, msg):
        if len(msg.data) >= 3:
            self.current_x = msg.data[0]
            self.current_y = msg.data[1]
            self.current_z = msg.data[2]
    
    def handle_command(self, req):
        """处理 GCS 发来的命令"""
        try:
            payload = json.loads(req.json)
        except Exception as e:
            return JsonCommandResponse(False, f"invalid json: {e}")
        
        cmd = payload.get("cmd", "")
        
        rospy.loginfo("[命令] cmd=%s", cmd)
        
        if cmd == "takeoff":
            return self.handle_takeoff(payload)
        elif cmd == "traj_following":
            return self.handle_traj_following(payload)
        elif cmd == "land":
            return self.handle_land(payload)
        else:
            return JsonCommandResponse(False, f"unknown cmd: {cmd}")
    
    def handle_takeoff(self, payload):
        height = float(payload.get("height", self.target_height))
        duration = float(payload.get("duration", self.takeoff_duration))
        
        if height <= 0 or duration <= 0:
            return JsonCommandResponse(False, "height and duration must be positive")
        
        self.target_height = height
        self.takeoff_duration = duration
        
        if self.current_state == self.STATE_IDLE:
            self.transition_to_state(self.STATE_TAKEOFF)
            return JsonCommandResponse(True, "takeoff accepted")
        else:
            return JsonCommandResponse(False, f"cannot takeoff from {self.current_state}")
    
    def handle_traj_following(self, payload):
        if self.current_state == self.STATE_HOVER:
            self.request_trajectory_from_downstream("main_trajectory")
            return JsonCommandResponse(True, "traj_following accepted")
        else:
            return JsonCommandResponse(False, f"cannot start from {self.current_state}")
    
    def handle_land(self, payload):
        if self.current_state in [self.STATE_HOVER, self.STATE_TRAJ_FOLLOWING]:
            self.transition_to_state(self.STATE_LANDING)
            return JsonCommandResponse(True, "land accepted")
        else:
            return JsonCommandResponse(False, f"cannot land from {self.current_state}")
    
    def request_trajectory_from_downstream(self, trajectory_type):
        flag = Bool()
        flag.data = True
        self.flag_pub.publish(flag)
        
        msg = String()
        msg.data = trajectory_type
        self.trajectory_request_pub.publish(msg)
        
        self.trajectory_active = True
        self.trajectory_started = False
    
    def trajectory_data_callback(self, msg):
        if not self.trajectory_active:
            return
        
        if not self.trajectory_started:
            rospy.loginfo("收到第一个轨迹点，切换到 traj_following")
            self.transition_to_state(self.STATE_TRAJ_FOLLOWING)
            self.trajectory_started = True
        
        if self.current_state == self.STATE_TRAJ_FOLLOWING:
            self.trajectory_pub.publish(msg)
            self.last_point_time = rospy.Time.now()
    
    def transition_to_state(self, new_state):
        if new_state == self.current_state:
            return
        
        rospy.loginfo("[状态] %s -> %s", self.current_state, new_state)
        
        if new_state in [self.STATE_HOVER, self.STATE_TAKEOFF, self.STATE_LANDING]:
            self.lock_x = self.current_x
            self.lock_y = self.current_y
            self.lock_z = -self.current_z
        
        if new_state != self.STATE_TRAJ_FOLLOWING:
            self.trajectory_active = False
            self.trajectory_started = False
        
        self.current_state = new_state
        self.state_start_time = rospy.Time.now()
    
    def set_height(self, t, duration, x, y, height):
        traj = TrajPoint()
        
        if t >= duration:
            traj.pd.x = x
            traj.pd.y = y
            traj.pd.z = -height
            traj.dpd.x = traj.dpd.y = traj.dpd.z = 0.0
            traj.d2pd.x = traj.d2pd.y = traj.d2pd.z = 0.0
            traj.d3pd.x = traj.d3pd.y = traj.d3pd.z = 0.0
            traj.d4pd.x = traj.d4pd.y = traj.d4pd.z = 0.0
            traj.d5pd.x = traj.d5pd.y = traj.d5pd.z = 0.0
            traj.yawd = 0.0
            traj.yawd_dot = 0.0
        elif t < 0.0:
            traj.pd.x = x
            traj.pd.y = y
            traj.pd.z = 1.0
            traj.dpd.x = traj.dpd.y = traj.dpd.z = 0.0
            traj.d2pd.x = traj.d2pd.y = traj.d2pd.z = 0.0
            traj.d3pd.x = traj.d3pd.y = traj.d3pd.z = 0.0
            traj.d4pd.x = traj.d4pd.y = traj.d4pd.z = 0.0
            traj.d5pd.x = traj.d5pd.y = traj.d5pd.z = 0.0
            traj.yawd = 0.0
            traj.yawd_dot = 0.0
        else:
            t_norm = t / duration
            traj.pd.x = x
            traj.pd.y = y
            traj.pd.z = -height * (3*t_norm*t_norm - 2*t_norm*t_norm*t_norm)
            traj.dpd.x = traj.dpd.y = 0.0
            traj.dpd.z = -height * (6*t_norm - 6*t_norm*t_norm) / duration
            traj.d2pd.x = traj.d2pd.y = 0.0
            traj.d2pd.z = -height * (6 - 12*t_norm) / (duration * duration)
            traj.d3pd.x = traj.d3pd.y = 0.0
            traj.d3pd.z = 12 * height / (duration * duration * duration)
            traj.d4pd.x = traj.d4pd.y = traj.d4pd.z = 0.0
            traj.d5pd.x = traj.d5pd.y = traj.d5pd.z = 0.0
            traj.yawd = 0.0
            traj.yawd_dot = 0.0
        
        return traj
    
    def main_timer_callback(self, event):
        """主定时器：200Hz，根据状态发布轨迹点 + 状态更新"""
        current_time = rospy.Time.now()
        elapsed = (current_time - self.state_start_time).to_sec()
        since_last_point = (current_time - self.last_point_time).to_sec()
        
        # ========== 根据状态发布轨迹点 ==========
        if self.current_state == self.STATE_TAKEOFF:
            point = self.set_height(elapsed, self.takeoff_duration, 
                                     self.lock_x, self.lock_y, self.target_height)
            self.trajectory_pub.publish(point)
        
        elif self.current_state == self.STATE_LANDING:
            t = self.landing_duration - elapsed
            point = self.set_height(t, self.landing_duration,
                                     self.lock_x, self.lock_y, self.lock_z)
            self.trajectory_pub.publish(point)
        
        elif self.current_state == self.STATE_HOVER:
            point = TrajPoint()
            point.pd.x = self.lock_x
            point.pd.y = self.lock_y
            point.pd.z = -self.lock_z
            point.dpd.x = point.dpd.y = point.dpd.z = 0.0
            point.d2pd.x = point.d2pd.y = point.d2pd.z = 0.0
            point.d3pd.x = point.d3pd.y = point.d3pd.z = 0.0
            point.d4pd.x = point.d4pd.y = point.d4pd.z = 0.0
            point.d5pd.x = point.d5pd.y = point.d5pd.z = 0.0
            point.yawd = 0.0
            point.yawd_dot = 0.0
            self.trajectory_pub.publish(point)
        
        # ========== 状态更新逻辑 ==========
        if self.current_state == self.STATE_TAKEOFF:
            if elapsed >= self.takeoff_duration:
                self.transition_to_state(self.STATE_HOVER)
        
        elif self.current_state == self.STATE_LANDING:
            if elapsed >= self.landing_duration + 1.0:  # 降落完成后多等1秒
                self.transition_to_state(self.STATE_IDLE)
        
        elif self.current_state == self.STATE_TRAJ_FOLLOWING:
            if self.trajectory_started and since_last_point > 0.1:
                rospy.logwarn("轨迹中断，开始降落")
                self.trajectory_active = False
                self.transition_to_state(self.STATE_LANDING)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = TrajRouterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
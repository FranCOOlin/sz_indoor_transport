#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rospy
from std_msgs.msg import String, Float64MultiArray, Bool
from sz_indoor_fsm.srv import JsonCommand, JsonCommandResponse
from sz_indoor_controller.msg import TrajPoint

# 单机到多机 ----
class UAVState:
    """单架无人机的状态"""
    def __init__(self, uav_id):
        self.uav_id = uav_id
        
        # 状态
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
        
        # 轨迹标志
        self.trajectory_active = False
        self.last_point_time = rospy.Time.now()
        self.trajectory_started = False
        
        # 参数
        self.takeoff_duration = 6.0
        self.landing_duration = 5.0
        self.target_height = 1.0
        
        # 发布器
        self.flag_pub = None
        self.trajectory_pub = None


class MultiTrajRouterNode:
    """
    多机轨迹路由器节点 - 对接 GCS FSM
    """
    
    def __init__(self):
        rospy.init_node('multi_traj_router', anonymous=True)
        
        # 从 launch 文件读取 participants
        participants_str = rospy.get_param("~participants", "")
        
        if not participants_str:
            rospy.logerr("未设置 participants 参数")
            rospy.signal_shutdown("缺少 participants 参数")
            return
        
        self.uavs = self._parse_participants(participants_str)
        
        if not self.uavs:
            rospy.logerr("participants 参数解析失败: %s", participants_str)
            rospy.signal_shutdown("participants 参数无效")
            return

        self.uav_states = {}
        self.default_traj_type = int(rospy.get_param("~traj_type", 1))
        self.landing_duration = float(rospy.get_param("~landing_duration", 5.0))
        self.trajectory_request_pub = rospy.Publisher(
            '/trajectory_request', String, queue_size=10
        )

        for uav_id in self.uavs:
            full_id = f"/{uav_id}" if not uav_id.startswith('/') else uav_id
            state = UAVState(full_id)
            state.landing_duration = self.landing_duration
            self.uav_states[full_id] = state
            self._init_uav_communication(state)
            rospy.loginfo(f"[初始化] {full_id}")
        
        # service
        self.server = rospy.Service('/traj_router/command', JsonCommand, self.handle_command)
        
        # 定时器
        rospy.Timer(rospy.Duration(0.005), self.main_timer_callback)
        
        rospy.loginfo(f"Multi Traj Router 已启动，管理 {len(self.uav_states)} 架无人机")
    
    def _parse_participants(self, participants_str):
        participants_str = participants_str.strip()
        if not participants_str:
            return []
        if ',' in participants_str and not participants_str.startswith('['):
            return [p.strip() for p in participants_str.split(',') if p.strip()]
        if participants_str.startswith('[') and participants_str.endswith(']'):
            inner = participants_str[1:-1]
            return [p.strip() for p in inner.split(',') if p.strip()]
        return [participants_str]
    
    def _init_uav_communication(self, state):
        uav_id = state.uav_id
        
        state.flag_pub = rospy.Publisher(f'{uav_id}/traj_generation_flag', Bool, queue_size=10)
        state.trajectory_pub = rospy.Publisher(f'{uav_id}/trajectory', TrajPoint, queue_size=10)
        
        traj_topic = f'{uav_id}/planning/traj_point'
        rospy.Subscriber(traj_topic, TrajPoint, 
                         lambda msg, sid=uav_id: self.trajectory_data_callback(msg, sid))
        
        quadrotor_state_topic = f'{uav_id}/quadrotor_state'
        rospy.Subscriber(quadrotor_state_topic, Float64MultiArray,
                         lambda msg, sid=uav_id: self.quadrotor_state_callback(msg, sid))
    
    def quadrotor_state_callback(self, msg, uav_id):
        if uav_id not in self.uav_states:
            return
        if len(msg.data) >= 3:
            state = self.uav_states[uav_id]
            state.current_x = msg.data[0]
            state.current_y = msg.data[1]
            state.current_z = msg.data[2]
    
    def trajectory_data_callback(self, msg, uav_id):
        if uav_id not in self.uav_states:
            return
        
        state = self.uav_states[uav_id]
        
        if not state.trajectory_active:
            return
        
        if not state.trajectory_started:
            rospy.loginfo(f"[{uav_id}] 收到轨迹点，开始跟随")
            self.transition_to_state(state, state.STATE_TRAJ_FOLLOWING)
            state.trajectory_started = True
        
        if state.current_state == state.STATE_TRAJ_FOLLOWING:
            state.trajectory_pub.publish(msg)
            state.last_point_time = rospy.Time.now()
    
    def handle_command(self, req):
        """处理 GCS 发来的命令 - 只处理 cmd、height、duration"""
        try:
            payload = json.loads(req.json)
        except Exception as e:
            return JsonCommandResponse(False, f"invalid json: {e}")
        
        cmd = payload.get("cmd", "")
        
        # 获取参与者，没有则使用所有无人机
        participants = payload.get("participants", self.uavs)
        
        participants_full = []
        for p in participants:
            full_id = f"/{p}" if not p.startswith('/') else p
            if full_id in self.uav_states:
                participants_full.append(full_id)
        
        if not participants_full:
            return JsonCommandResponse(False, "no valid participants")
        
        rospy.loginfo(f"[命令] cmd={cmd} uavs={participants_full}")
        
        if cmd == "takeoff":
            height = float(payload.get("height", 1.0))
            duration = float(payload.get("duration", 6.0))
            return self.handle_takeoff(participants_full, height, duration)
        
        elif cmd == "traj_following":
            return self.handle_traj_following(participants_full, payload)
        
        elif cmd == "land":
            return self.handle_land(participants_full)
        
        else:
            return JsonCommandResponse(False, f"unknown cmd: {cmd}")
    
    def handle_takeoff(self, participants, height, duration):
        """起飞命令"""
        if height <= 0 or duration <= 0:
            return JsonCommandResponse(False, "height and duration must be positive")
        
        success = []
        fail = []
        
        for uav_id in participants:
            state = self.uav_states[uav_id]
            state.target_height = height
            state.takeoff_duration = duration
            
            if state.current_state == state.STATE_IDLE:
                self.transition_to_state(state, state.STATE_TAKEOFF)
                success.append(uav_id)
            else:
                fail.append(f"{uav_id}({state.current_state})")
        
        if fail:
            return JsonCommandResponse(False, f"失败: {fail}")
        return JsonCommandResponse(True, f"起飞: {success}")
    
    def handle_traj_following(self, participants, payload):
        """开始主轨迹跟随。"""
        try:
            traj_type = int(payload.get("traj_type", self.default_traj_type))
        except (TypeError, ValueError):
            return JsonCommandResponse(False, "traj_type must be an integer")
        if traj_type <= 0:
            return JsonCommandResponse(False, "traj_type must be positive")

        trajectory_name = f"trajectory{traj_type}"
        success = []
        fail = []

        for uav_id in participants:
            state = self.uav_states[uav_id]
            if state.current_state in [
                state.STATE_TAKEOFF,
                state.STATE_HOVER,
                state.STATE_TRAJ_FOLLOWING,
            ]:
                self.request_trajectory_from_downstream(state, trajectory_name)
                success.append(uav_id)
            else:
                fail.append(f"{uav_id}({state.current_state})")

        if fail:
            return JsonCommandResponse(False, f"失败: {fail}")
        return JsonCommandResponse(True, f"轨迹跟随 {trajectory_name}: {success}")
    
    def handle_land(self, participants):
        """降落命令"""
        success = []
        fail = []
        
        for uav_id in participants:
            state = self.uav_states[uav_id]
            
            if state.current_state in [state.STATE_HOVER, state.STATE_TRAJ_FOLLOWING]:
                self.transition_to_state(state, state.STATE_LANDING)
                success.append(uav_id)
            else:
                fail.append(f"{uav_id}({state.current_state})")
        
        if fail:
            return JsonCommandResponse(False, f"失败: {fail}")
        return JsonCommandResponse(True, f"降落: {success}")
    
    def request_trajectory_from_downstream(self, state, trajectory_name):
        """请求轨迹"""
        flag = Bool()
        flag.data = True
        state.flag_pub.publish(flag)

        msg = String()
        msg.data = trajectory_name
        self.trajectory_request_pub.publish(msg)

        state.trajectory_active = True
        state.trajectory_started = False
    
    def transition_to_state(self, state, new_state):
        if new_state == state.current_state:
            return
        
        rospy.loginfo(f"[{state.uav_id}] {state.current_state} -> {new_state}")
        
        if new_state in [state.STATE_HOVER, state.STATE_TAKEOFF, state.STATE_LANDING]:
            state.lock_x = state.current_x
            state.lock_y = state.current_y
            state.lock_z = -state.current_z
        
        if new_state != state.STATE_TRAJ_FOLLOWING:
            state.trajectory_active = False
            state.trajectory_started = False
        
        state.current_state = new_state
        state.state_start_time = rospy.Time.now()
    
    def set_height(self, t, duration, x, y, height):
        """生成起飞/降落轨迹点"""
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
        """主定时器：200Hz"""
        current_time = rospy.Time.now()
        
        for uav_id, state in self.uav_states.items():
            elapsed = (current_time - state.state_start_time).to_sec()
            since_last_point = (current_time - state.last_point_time).to_sec()
            
            # 发布轨迹点
            if state.current_state == state.STATE_TAKEOFF:
                point = self.set_height(elapsed, state.takeoff_duration,
                                         state.lock_x, state.lock_y, state.target_height)
                state.trajectory_pub.publish(point)
            
            elif state.current_state == state.STATE_LANDING:
                t = state.landing_duration - elapsed
                point = self.set_height(t, state.landing_duration,
                                         state.lock_x, state.lock_y, state.lock_z)
                state.trajectory_pub.publish(point)
            
            elif state.current_state == state.STATE_HOVER:
                point = TrajPoint()
                point.pd.x = state.lock_x
                point.pd.y = state.lock_y
                point.pd.z = -state.lock_z
                point.dpd.x = point.dpd.y = point.dpd.z = 0.0
                point.d2pd.x = point.d2pd.y = point.d2pd.z = 0.0
                point.d3pd.x = point.d3pd.y = point.d3pd.z = 0.0
                point.d4pd.x = point.d4pd.y = point.d4pd.z = 0.0
                point.d5pd.x = point.d5pd.y = point.d5pd.z = 0.0
                point.yawd = 0.0
                point.yawd_dot = 0.0
                state.trajectory_pub.publish(point)
            
            # 状态更新
            if state.current_state == state.STATE_TAKEOFF:
                if elapsed >= state.takeoff_duration:
                    self.transition_to_state(state, state.STATE_HOVER)
            
            elif state.current_state == state.STATE_LANDING:
                if elapsed >= state.landing_duration + 1.0:
                    self.transition_to_state(state, state.STATE_IDLE)
            
            elif state.current_state == state.STATE_TRAJ_FOLLOWING:
                if state.trajectory_started and since_last_point > 0.1:
                    rospy.logwarn(f"[{uav_id}] 轨迹中断，降落")
                    state.trajectory_active = False
                    self.transition_to_state(state, state.STATE_LANDING)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = MultiTrajRouterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

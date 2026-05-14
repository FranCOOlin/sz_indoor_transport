#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Float64MultiArray, Bool
from std_srvs.srv import Trigger, TriggerResponse
from test_controller.msg import TrajPoint

class TrajectoryStateMachine:
    """
    轨迹调度状态机 - 实时转发版
    
    架构：
    上端口(服务) <-> 本节点 <-> 下端口(轨迹节点)
                     ↓
                  执行机构
    
    流程：
    1. 接收上端口服务请求
    2. 向下端口发送轨迹命令（请求轨迹）
    3. 从下端口接收轨迹数据（实时转发，不缓存）
    4. 立即将轨迹数据发布给执行机构
    5. 悬停/起飞/降落状态由本节点自己生成轨迹点（200Hz）
    """
    
    def __init__(self):
        rospy.init_node('trajectory_scheduler', anonymous=True)
        
        # 参数配置
        self.takeoff_duration = rospy.get_param("~takeoff_duration", 5.0)   # 起飞轨迹持续时间
        self.landing_duration = rospy.get_param("~landing_duration", 5.0)   # 降落轨迹持续时间
        self.target_height = rospy.get_param("~target_height", 1.0)          # 目标高度（正值）
        self.uav_id = rospy.get_param("~uav_id", "/uav1")                    # UAV ID
        
        # 状态定义
        self.STATE_IDLE = "idle"              # 初始状态（已降落状态）
        self.STATE_TAKEOFF = "takeoff"        # 起飞状态
        self.STATE_HOVER = "hover"            # 悬停状态
        self.STATE_TRAJ1 = "trajectory1"      # 轨迹1
        self.STATE_TRAJ2 = "trajectory2"      # 轨迹2
        self.STATE_TRAJ3 = "trajectory3"      # 轨迹3
        self.STATE_LANDING = "landing"        # 降落状态
        
        # 当前状态
        self.current_state = self.STATE_IDLE
        self.state_start_time = rospy.Time.now()
        
        # 实时位置（从/quadrotor_state持续更新）
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # 锁定位置（进入悬停/起飞/降落时锁定的位置）
        self.hover_x = 0.0
        self.hover_y = 0.0
        self.hover_z = 0.0
        
        # 实时转发标志
        self.trajectory_active = False        # 是否允许接收外部轨迹
        self.last_point_time = rospy.Time.now()   # 最后收到轨迹点的时间
        self.trajectory_started = False
        
        # 待处理的轨迹请求（用于延迟切换，防止offboard断流）
        self.pending_trajectory = None
        
        # ========== 向下端口（发送轨迹请求） ==========
        self.trajectory_request_pub = rospy.Publisher('/trajectory_request', String, queue_size=10)
        self.flag_pub = rospy.Publisher(self.uav_id + '/traj_generation_flag', Bool, queue_size=10)
        
        # ========== 从下端口接收轨迹数据（TrajPoint格式） ==========
        traj_topic = self.uav_id + "/planning/traj_point"
        rospy.Subscriber(traj_topic, TrajPoint, self.trajectory_data_callback)
        
        # ========== 从/quadrotor_state获取当前位姿（获取实际位置和高度） ==========
        quadrotor_state_topic = self.uav_id + "/quadrotor_state"
        rospy.Subscriber(quadrotor_state_topic, Float64MultiArray, self.quadrotor_state_callback)
        
        # ========== 发布轨迹给执行机构（TrajPoint格式） ==========
        self.trajectory_pub = rospy.Publisher(self.uav_id + "/trajectory", TrajPoint, queue_size=10)
        
        # ========== 向上端口（接收上位机服务请求） ==========
        rospy.Service('/request_takeoff', Trigger, self.handle_takeoff_request)
        rospy.Service('/request_trajectory1', Trigger, self.handle_traj1_request)
        rospy.Service('/request_trajectory2', Trigger, self.handle_traj2_request)
        rospy.Service('/request_trajectory3', Trigger, self.handle_traj3_request)
        rospy.Service('/request_landing', Trigger, self.handle_landing_request)
        
        # 定时器：状态更新（200Hz = 0.005秒）
        self.state_timer = rospy.Timer(rospy.Duration(0.005), self.update_state)
        
        # 定时器：发布悬停轨迹点（200Hz = 0.005秒）
        self.hover_timer = rospy.Timer(rospy.Duration(0.005), self.publish_hover_point)

        # 定时器：发布起飞轨迹点（200Hz = 0.005秒）
        self.takeoff_timer = rospy.Timer(rospy.Duration(0.005), self.publish_takeoff_point)
        
        # 定时器：发布降落轨迹点（200Hz = 0.005秒）
        self.landing_timer = rospy.Timer(rospy.Duration(0.005), self.publish_landing_point)
        
        rospy.loginfo("="*60)
        rospy.loginfo("轨迹调度状态机已启动（实时转发模式）")
        rospy.loginfo("UAV ID: %s", self.uav_id)
        rospy.loginfo("起飞持续时间: %.2f s, 降落持续时间: %.2f s", self.takeoff_duration, self.landing_duration)
        rospy.loginfo("目标高度: %.2f m", self.target_height)
        rospy.loginfo("当前状态: %s (初始状态)", self.current_state)
        rospy.loginfo("="*60)
    
    # ========== quadrotor_state回调（获取当前位置，用于实时更新） ==========
    
    def quadrotor_state_callback(self, msg):
        """
        从/quadrotor_state获取当前无人机位置
        msg.data[0] = x
        msg.data[1] = y
        msg.data[2] = z
        """
        if len(msg.data) < 3:
            rospy.logwarn("Invalid quadrotor_state message: expected at least 3 elements, received %lu", len(msg.data))
            return
        
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]
        self.current_z = msg.data[2]  # z轴向下为正
    
    # ========== 服务请求处理 ==========
    
    def handle_takeoff_request(self, req):
        if self.current_state == self.STATE_IDLE:
            self.transition_to_state(self.STATE_TAKEOFF)
            return TriggerResponse(success=True, message="起飞命令已接收")
        else:
            return TriggerResponse(success=False, message=f"当前状态 {self.current_state} 无法起飞")
    
    def handle_traj1_request(self, req):
        if self.current_state == self.STATE_HOVER:
            # 不立即切换状态，只请求轨迹，等待第一个点到达
            self.request_trajectory_from_downstream("trajectory1")
            return TriggerResponse(success=True, message="请求轨迹1")
        else:
            return TriggerResponse(success=False, message=f"当前状态 {self.current_state} 无法执行轨迹1")
    
    def handle_traj2_request(self, req):
        if self.current_state == self.STATE_HOVER:
            self.request_trajectory_from_downstream("trajectory2")
            return TriggerResponse(success=True, message="请求轨迹2")
        else:
            return TriggerResponse(success=False, message=f"当前状态 {self.current_state} 无法执行轨迹2")
    
    def handle_traj3_request(self, req):
        if self.current_state == self.STATE_HOVER:
            self.request_trajectory_from_downstream("trajectory3")
            return TriggerResponse(success=True, message="请求轨迹3")
        else:
            return TriggerResponse(success=False, message=f"当前状态 {self.current_state} 无法执行轨迹3")
    
    def handle_landing_request(self, req):
        if self.current_state in [self.STATE_HOVER, self.STATE_TRAJ1, 
                                   self.STATE_TRAJ2, self.STATE_TRAJ3]:
            self.transition_to_state(self.STATE_LANDING)
            return TriggerResponse(success=True, message="开始降落")
        else:
            return TriggerResponse(success=False, message=f"当前状态 {self.current_state} 无法降落")
    
    # ========== 状态转换 ==========
    
    def transition_to_state(self, new_state):
        if new_state == self.current_state:
            return
        
        rospy.loginfo("-"*60)
        rospy.loginfo("[状态转换] %s -> %s", self.current_state, new_state)
        
        # 取消待处理的轨迹请求
        self.pending_trajectory = None
        self.trajectory_started = False
        
        # 进入需要锁定位置的状态时，锁定当前位置
        if new_state in [self.STATE_HOVER, self.STATE_TAKEOFF, self.STATE_LANDING]:
            self.hover_x = self.current_x
            self.hover_y = self.current_y
            self.hover_z = -self.current_z
            rospy.loginfo("[锁定位置] (%.2f, %.2f, %.2f)", self.hover_x, self.hover_y, self.hover_z)
        
        if new_state == self.STATE_HOVER:
            self.trajectory_active = False
            rospy.loginfo("[悬停] 进入悬停状态，位置: (%.2f, %.2f), 高度: %.2f", 
                         self.hover_x, self.hover_y, self.hover_z)
        
        elif new_state == self.STATE_TAKEOFF:
            self.trajectory_active = False
            rospy.loginfo("[起飞] 开始起飞，目标高度: %.2f m", self.target_height)
            
        elif new_state == self.STATE_TRAJ1:
            rospy.loginfo("[轨迹1] 进入轨迹1状态")
        elif new_state == self.STATE_TRAJ2:
            rospy.loginfo("[轨迹2] 进入轨迹2状态")
        elif new_state == self.STATE_TRAJ3:
            rospy.loginfo("[轨迹3] 进入轨迹3状态")
            
        elif new_state == self.STATE_LANDING:
            self.trajectory_active = False
            rospy.loginfo("[降落] 开始降落，当前高度: %.2f m", self.hover_z)
            
        elif new_state == self.STATE_IDLE:
            self.trajectory_active = False
            rospy.loginfo("[已降落] 进入初始状态")
        
        self.current_state = new_state
        self.state_start_time = rospy.Time.now()
        rospy.loginfo("-"*60)
    
    def request_trajectory_from_downstream(self, trajectory_type):
        """向下端口请求轨迹数据（不立即切换状态，等待第一个点）"""
        # 发送轨迹生成标志
        flag = Bool()
        flag.data = True
        self.flag_pub.publish(flag)
        rospy.loginfo("[标志] 发送 traj_generation_flag = True")
        
        # 发送轨迹请求
        msg = String()
        msg.data = trajectory_type
        self.trajectory_request_pub.publish(msg)
        rospy.loginfo("[下端口->] 请求轨迹: %s", trajectory_type)
        
        # 记录待处理的轨迹类型，等待第一个点到达后再切换状态
        self.pending_trajectory = trajectory_type
        self.trajectory_active = True
        self.trajectory_started = False
    
    def set_height(self, t, takeoff_and_land_time, x, y, height):
        """
        生成起飞/降落轨迹点
        t: 当前时间（秒）
        takeoff_and_land_time: 起飞/降落总持续时间（秒）
        x, y: 目标位置的x,y坐标
        height: 目标高度（正值）
        """
        trajectory = TrajPoint()
        
        if t > takeoff_and_land_time:
            # 悬停状态（已完成起飞或降落）
            trajectory.pd.x = x
            trajectory.pd.y = y
            trajectory.pd.z = -height
            
            trajectory.dpd.x = 0.0
            trajectory.dpd.y = 0.0
            trajectory.dpd.z = 0.0
            
            trajectory.d2pd.x = 0.0
            trajectory.d2pd.y = 0.0
            trajectory.d2pd.z = 0.0
            
            trajectory.d3pd.x = 0.0
            trajectory.d3pd.y = 0.0
            trajectory.d3pd.z = 0.0
            
            trajectory.d4pd.x = 0.0
            trajectory.d4pd.y = 0.0
            trajectory.d4pd.z = 0.0
            
            trajectory.d5pd.x = 0.0
            trajectory.d5pd.y = 0.0
            trajectory.d5pd.z = 0.0
            
            trajectory.yawd = 0.0
            trajectory.yawd_dot = 0.0
            
        elif t < 0.0:
            # 初始状态（避免飞行）
            trajectory.pd.x = x
            trajectory.pd.y = y
            trajectory.pd.z = 1.0  # 安全高度
            
            trajectory.dpd.x = 0.0
            trajectory.dpd.y = 0.0
            trajectory.dpd.z = 0.0
            
            trajectory.d2pd.x = 0.0
            trajectory.d2pd.y = 0.0
            trajectory.d2pd.z = 0.0
            
            trajectory.d3pd.x = 0.0
            trajectory.d3pd.y = 0.0
            trajectory.d3pd.z = 0.0
            
            trajectory.d4pd.x = 0.0
            trajectory.d4pd.y = 0.0
            trajectory.d4pd.z = 0.0
            
            trajectory.d5pd.x = 0.0
            trajectory.d5pd.y = 0.0
            trajectory.d5pd.z = 0.0
            
            trajectory.yawd = 0.0
            trajectory.yawd_dot = 0.0
            
        else:
            # 上升/下降过程中（5次多项式轨迹）
            t_s = t / takeoff_and_land_time  # 归一化时间
            
            # 位置
            trajectory.pd.x = x
            trajectory.pd.y = y
            trajectory.pd.z = -height * (3*t_s*t_s - 2*t_s*t_s*t_s)
            
            # 速度（一阶导）
            trajectory.dpd.x = 0.0
            trajectory.dpd.y = 0.0
            trajectory.dpd.z = -height * (6*t_s - 6*t_s*t_s) / takeoff_and_land_time
            
            # 加速度（二阶导）
            trajectory.d2pd.x = 0.0
            trajectory.d2pd.y = 0.0
            trajectory.d2pd.z = -height * (6 - 12*t_s) / (takeoff_and_land_time * takeoff_and_land_time)
            
            # 加加速度（三阶导）
            trajectory.d3pd.x = 0.0
            trajectory.d3pd.y = 0.0
            trajectory.d3pd.z = 12 * height / (takeoff_and_land_time * takeoff_and_land_time * takeoff_and_land_time)
            
            # 更高阶导数为0
            trajectory.d4pd.x = 0.0
            trajectory.d4pd.y = 0.0
            trajectory.d4pd.z = 0.0
            
            trajectory.d5pd.x = 0.0
            trajectory.d5pd.y = 0.0
            trajectory.d5pd.z = 0.0
            
            # 偏航角
            trajectory.yawd = 0.0
            trajectory.yawd_dot = 0.0
        
        return trajectory

    # ========== 从下端口接收轨迹数据（实时转发） ==========
    
    def trajectory_data_callback(self, msg):
        """接收下端口发来的轨迹数据（TrajPoint格式）-> 立即转发，不缓存"""
        
        if not self.trajectory_active:
            return
        
        # 如果是第一个点，且有待切换的轨迹状态，则真正切换到轨迹状态
        if not self.trajectory_started and self.pending_trajectory is not None:
            rospy.loginfo("[轨迹开始] 收到第一个轨迹点，切换到 %s 状态", self.pending_trajectory)
            
            # 真正切换到轨迹状态
            if self.pending_trajectory == "trajectory1":
                self.transition_to_state(self.STATE_TRAJ1)
            elif self.pending_trajectory == "trajectory2":
                self.transition_to_state(self.STATE_TRAJ2)
            elif self.pending_trajectory == "trajectory3":
                self.transition_to_state(self.STATE_TRAJ3)
            
            self.pending_trajectory = None
            self.trajectory_started = True
        
        # 只有真正在轨迹状态才转发
        if self.current_state in [self.STATE_TRAJ1, self.STATE_TRAJ2, self.STATE_TRAJ3]:
            # 立即转发给执行机构
            self.trajectory_pub.publish(msg)
            
            # 更新时间
            self.last_point_time = rospy.Time.now()
    
    # ========== 发布起飞轨迹点（本节点自己生成，200Hz） ==========
    
    def publish_takeoff_point(self, event):
        """发布起飞轨迹点 - 只在起飞状态时发布，200Hz"""
        if self.current_state != self.STATE_TAKEOFF:
            return
        
        # 计算当前时间
        t = (rospy.Time.now() - self.state_start_time).to_sec()
        
        # 生成起飞轨迹点（使用锁定的hover_x, hover_y，从0到target_height）
        takeoff_point = self.set_height(t, self.takeoff_duration, self.hover_x, self.hover_y, self.target_height)
        
        # 发布轨迹点
        self.trajectory_pub.publish(takeoff_point)
    
    # ========== 发布降落轨迹点（本节点自己生成，200Hz） ==========
    
    def publish_landing_point(self, event):
        """发布降落轨迹点 - 只在降落状态时发布，200Hz"""
        if self.current_state != self.STATE_LANDING:
            return
        
        # 计算当前时间（降落时t从0到landing_duration）
        t = (rospy.Time.now() - self.state_start_time).to_sec()
        t = self.landing_duration - t
        
        # 生成降落轨迹点（使用锁定的hover_x, hover_y，从锁定的hover_z降到0）
        landing_point = self.set_height(t, self.landing_duration, self.hover_x, self.hover_y, self.hover_z)
        
        # 发布轨迹点
        self.trajectory_pub.publish(landing_point)
    
    # ========== 发布悬停轨迹点（本节点自己生成，200Hz） ==========
    
    def publish_hover_point(self, event):
        """发布悬停轨迹点 - 只在悬停状态时发布，200Hz"""
        if self.current_state != self.STATE_HOVER:
            return
        
        hover_point = TrajPoint()
        
        hover_point.pd.x = self.hover_x
        hover_point.pd.y = self.hover_y
        hover_point.pd.z = -self.hover_z
        
        hover_point.dpd.x = 0.0
        hover_point.dpd.y = 0.0
        hover_point.dpd.z = 0.0
        
        hover_point.d2pd.x = 0.0
        hover_point.d2pd.y = 0.0
        hover_point.d2pd.z = 0.0
        
        hover_point.d3pd.x = 0.0
        hover_point.d3pd.y = 0.0
        hover_point.d3pd.z = 0.0
        
        hover_point.d4pd.x = 0.0
        hover_point.d4pd.y = 0.0
        hover_point.d4pd.z = 0.0
        
        hover_point.d5pd.x = 0.0
        hover_point.d5pd.y = 0.0
        hover_point.d5pd.z = 0.0
        
        hover_point.yawd = 0.0
        hover_point.yawd_dot = 0.0
        
        self.trajectory_pub.publish(hover_point)
    
    # ========== 状态更新（200Hz，检查轨迹是否结束） ==========
    
    def update_state(self, event):
        """状态机更新逻辑 - 检查轨迹是否结束（200Hz高频检查）"""
        
        # 计算距离上次收到轨迹点的时间（只对需要接收下端口的轨迹有效）
        elapsed_since_last = (rospy.Time.now() - self.last_point_time).to_sec()
        state_elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
        
        # ========== 1. 起飞状态：时间到切换到悬停 ==========
        if self.current_state == self.STATE_TAKEOFF:
            if state_elapsed >= self.takeoff_duration + 3:
                rospy.loginfo("[起飞完成] 切换到悬停状态")
                self.transition_to_state(self.STATE_HOVER)
        
        # ========== 2. 降落状态：时间到切换到初始状态 ==========
        elif self.current_state == self.STATE_LANDING:
            if state_elapsed >= self.landing_duration + 3:
                rospy.loginfo("[降落完成] 切换到初始状态")
                self.transition_to_state(self.STATE_IDLE)
        
        # ========== 3. 轨迹状态：需要接收下端口数据 ==========
        elif self.current_state in [self.STATE_TRAJ1, self.STATE_TRAJ2, self.STATE_TRAJ3]:
            if not self.trajectory_active:
                return
            
            # 进入了轨迹后出现100ms超时没收到数据，认为轨迹中断，开始降落
            if self.trajectory_started and elapsed_since_last > 0.1:
                rospy.logwarn("[轨迹中断] %s 轨迹数据中断 (%.1fms)，立即开始降落", 
                             self.current_state, elapsed_since_last * 1000)
                self.trajectory_active = False
                self.transition_to_state(self.STATE_LANDING)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TrajectoryStateMachine()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("轨迹调度节点已关闭")
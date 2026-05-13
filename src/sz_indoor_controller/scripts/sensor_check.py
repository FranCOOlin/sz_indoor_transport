#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 测试通过力传感器估计 Load 位置
import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from std_msgs.msg import Float64

ROPE_LENGTH = 0.9  # m

# 传感器系 -> 机体系 旋转矩阵
R_SENSOR_TO_BODY = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0]
], dtype=float)

# 最新数据缓存
adc_vec_sensor = None                 # 传感器系下的ADC向量
x250_pos_world = None                 # X250 世界系位置
R_body_to_world = np.eye(3, dtype=float)  # 机体->世界
load_pos_world = None                 # Load 实际世界系位置

# 发布器
pub_est_pose = None
pub_err_vec = None
pub_err_norm = None

def quat_to_R_body_to_world(w, x, y, z):
    """由四元数(w,x,y,z)得到机体->世界的旋转矩阵（单位化后计算）"""
    q = np.array([w, x, y, z], dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.eye(3)
    w, x, y, z = q / n
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ], dtype=float)

def adc_cb(msg):
    global adc_vec_sensor
    adc_vec_sensor = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=float)

def x250_pose_cb(msg):
    global x250_pos_world, R_body_to_world
    p = msg.pose.position
    x250_pos_world = np.array([p.x, p.y, p.z], dtype=float)
    q = msg.pose.orientation
    R_body_to_world = quat_to_R_body_to_world(q.w, q.x, q.y, q.z)

def load_pose_cb(msg):
    global load_pos_world
    p = msg.pose.position
    load_pos_world = np.array([p.x, p.y, p.z], dtype=float)

def try_compute_and_publish():
    """当数据齐备时，计算估计位置并发布误差"""
    if adc_vec_sensor is None or x250_pos_world is None or load_pos_world is None:
        return

    # 1) 先单位化传感器系向量
    n = np.linalg.norm(adc_vec_sensor)
    if n <= 1e-9:
        return
    u_sensor = adc_vec_sensor / n

    # 2) 传感器系 -> 机体系（注意“左乘矩阵”）
    u_body = R_SENSOR_TO_BODY.dot(u_sensor)

    # 3) 机体系 -> 世界系
    u_world = R_body_to_world.dot(u_body)

    # 4) 估计 Load 世界系位置
    est_pos_world = x250_pos_world + ROPE_LENGTH * u_world

    # 5) 误差（估计 - 实际）
    err = est_pos_world - load_pos_world
    err_norm = float(np.linalg.norm(err))

    # 6) 发布估计位姿
    est_pose_msg = PoseStamped()
    est_pose_msg.header.stamp = rospy.Time.now()
    est_pose_msg.header.frame_id = "world"
    est_pose_msg.pose.position.x = est_pos_world[0]
    est_pose_msg.pose.position.y = est_pos_world[1]
    est_pose_msg.pose.position.z = est_pos_world[2]
    est_pose_msg.pose.orientation.w = 1.0  # 只估位置，姿态设单位四元数
    est_pose_msg.pose.orientation.x = 0.0
    est_pose_msg.pose.orientation.y = 0.0
    est_pose_msg.pose.orientation.z = 0.0
    pub_est_pose.publish(est_pose_msg)

    # 7) 发布误差向量与模长
    err_vec_msg = Vector3Stamped()
    err_vec_msg.header.stamp = est_pose_msg.header.stamp
    err_vec_msg.header.frame_id = "world"
    err_vec_msg.vector.x = err[0]
    err_vec_msg.vector.y = err[1]
    err_vec_msg.vector.z = err[2]
    pub_err_vec.publish(err_vec_msg)

    pub_err_norm.publish(Float64(data=err_norm))

def main():
    global pub_est_pose, pub_err_vec, pub_err_norm
    rospy.init_node("load_position_estimator", anonymous=True)

    # 订阅
    rospy.Subscriber("/triple_adc_value", Vector3Stamped, adc_cb, queue_size=50)
    rospy.Subscriber("/vrpn_client_node/X250/pose", PoseStamped, x250_pose_cb, queue_size=20)
    rospy.Subscriber("/vrpn_client_node/Load/pose", PoseStamped, load_pose_cb, queue_size=20)

    # 发布
    pub_est_pose = rospy.Publisher("/estimated_load/pose", PoseStamped, queue_size=20)
    pub_err_vec  = rospy.Publisher("/estimated_load/error", Vector3Stamped, queue_size=20)
    pub_err_norm = rospy.Publisher("/estimated_load/error_norm", Float64, queue_size=20)

    rate = rospy.Rate(100)
    rospy.loginfo("load_position_estimator started (with sensor->body rotation).")
    try:
        while not rospy.is_shutdown():
            try_compute_and_publish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()

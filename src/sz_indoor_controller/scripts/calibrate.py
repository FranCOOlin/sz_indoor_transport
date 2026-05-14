#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped, PoseStamped
import curses
import time
from scipy.io import savemat

S = np.empty((3, 0))  # 原始ADC数据
N = np.empty((3, 0))  # 旋转后的标签
latest_vector = None
R = np.eye(3)  # 当前旋转矩阵（从四元数得到）

def adc_callback(msg):
    global latest_vector
    latest_vector = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

def pose_callback(msg):
    global R
    q = msg.pose.orientation
    w, x, y, z = q.w, q.x, q.y, q.z
    R = np.array([
        [1 - 2*y**2 - 2*z**2,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x**2 - 2*z**2,     2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x**2 - 2*y**2]
    ])

def input_string(stdscr, prompt, default=""):
    curses.echo()
    stdscr.addstr(2, 0, " " * 50)
    stdscr.addstr(2, 0, prompt)
    stdscr.refresh()
    input_str = stdscr.getstr(2, len(prompt)).decode("utf-8").strip()
    curses.noecho()
    return input_str if input_str != "" else default

def main(stdscr):
    global S, N

    rospy.init_node('triple_adc_listener', anonymous=True)
    rospy.Subscriber('/triple_adc_value', Vector3Stamped, adc_callback)
    rospy.Subscriber('/vrpn_client_node/X250/pose', PoseStamped, pose_callback)
    rate = rospy.Rate(100)

    curses.curs_set(0)
    stdscr.clear()
    stdscr.addstr(0, 0, "Listening... Press [SPACE] to record label, [Ctrl+C] to exit.")
    stdscr.refresh()

    last_input_value = "0.0"

    try:
        while not rospy.is_shutdown():
            key = stdscr.getch()
            if key == 32:  # 空格键
                if latest_vector is not None:
                    S = np.hstack((S, latest_vector.reshape(3, 1)))

                prompt = f"Enter label value (default: {last_input_value}): "
                user_input = input_string(stdscr, prompt, last_input_value)

                try:
                    value = float(user_input)
                    last_input_value = user_input
                except ValueError:
                    stdscr.addstr(3, 0, "Invalid input, using 0.0")
                    value = 0.0

                new_vec = np.array([[0], [0], [value]])
                rotated_vec = R.T @ new_vec  # 转换到机体坐标系
                N = np.hstack((N, rotated_vec))

                stdscr.clear()
                stdscr.addstr(0, 0, f"Saved data. Matrix S shape: {S.shape}, N shape: {N.shape}")
                stdscr.refresh()

            rate.sleep()

    except KeyboardInterrupt:
        pass
    finally:
        rospy.signal_shutdown("User terminated")

    return S, N

if __name__ == '__main__':
    try:
        S_mat, N_mat = curses.wrapper(main)
        print("\\n[CTRL+C] Program exited.")
        print("Final ADC matrix S shape:", S_mat.shape)
        print("Final label matrix N shape:", N_mat.shape)

        # 保存为 MATLAB 可读的 .mat 文件
        savemat("./adc_data.mat", {
            "S": S_mat,
            "N": N_mat
        })
        print("Saved S and N matrices to: /mnt/data/adc_data.mat")

    except Exception as e:
        print("Unhandled exception:", e)
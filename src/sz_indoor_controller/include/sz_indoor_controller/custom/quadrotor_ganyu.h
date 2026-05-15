
// #ifndef CONTROLLER_MYCONTROLLER_H
// #define CONTROLLER_MYCONTROLLER_H

// #include "sz_indoor_controller/controller/controller.h"
// #include "sz_indoor_controller/custom/system_params.h"
// #include "sz_indoor_controller/custom/quadrotor_state.h"
// #include "sz_indoor_controller/custom/mytrajectory.h"
// #include "sz_indoor_controller/custom/quadrotor_control_input.h"
// #include <eigen3/Eigen/Dense>

// namespace controller {

// // 示例派生类 MyController，实现了 update() 函数
// class QuadrotorControllerGanYu : public Controller {
// public:
//   common::SystemParams &params;
//   common::QuadrotorState &state;
//   common::MyTrajectory &trajectory;
//   common::QuadrotorControlInput &control_input;
//   QuadrotorControllerGanYu(common::SystemParams &_params, common::QuadrotorState &_state, common::MyTrajectory &_trajectory, common::QuadrotorControlInput &_control_input)
//   : params(_params), state(_state), trajectory(_trajectory), control_input(_control_input) {}


//   double clamp(double x, double min, double max) {
//     return x < min ? min : (x > max ? max : x);
//   }

//   // 多项式计算
//   double polyval(double x, const Eigen::VectorXd& p) {

//       // 多项式计算 y = polyval(p1, x)
//       double y = 0.0;
//       int degree = p.size() - 1;
//       for (int i = 0; i < p.size(); ++i) {
//           y += p[i] * std::pow(x, degree - i);
//       }

//       return y;
//   }

//   // 符号函数
//   template <typename T>
//   int sign(T value) {
//       if (value > 0) return 1;
//       if (value < 0) return -1;
//       return 0;
//   }


//   // 交叉乘积矩阵
//   Eigen::Matrix3d S(const Eigen::Vector3d& vec) {
//     Eigen::Matrix3d mat;
//       mat <<  0,       -vec(2),  vec(1),
//               vec(2),  0,       -vec(0),
//             -vec(1),  vec(0),   0;
//       return mat;
//   }

//   // 投影矩阵
//   Eigen::Matrix3d PI(const Eigen::Vector3d& vec) {
//       return Eigen::Matrix3d::Identity() - vec * vec.transpose();
//   }

//   virtual ~QuadrotorControllerGanYu() { }

//   // 实现 update()，计算控制信号（此处仅为示例：返回目标航点与当前位置的差值组成的 1 维向量）
//   virtual void update() override {
//     Eigen::VectorXd output(4);

//     double kp = params.quadrotor_kp;
//     double kv = params.quadrotor_kv;
//     double kr = params.quadrotor_kr;
//     double hr = params.quadrotor_hr;
//     double mq = params.quadrotor_mq;
//     double g = params.g;
//     double use_polyval = params.use_polyval;
//     Eigen::Vector3d p1 = params.p1;
//     Eigen::Vector3d p2 = params.p2;
//     Eigen::Vector3d p3 = params.p3;
//     Eigen::Vector3d p4 = params.p4;
//     Eigen::Vector3d e3(0, 0, 1);
//     // 计算位置和速度误差
//     double b =0.5;
//     Eigen::Vector3d zp = state.p - trajectory.pd;
//     double nzp = zp.norm();
//     Eigen::Vector3d zv = state.vi - trajectory.dpd;
//     double nzv = zv.norm();
//     Eigen::Vector3d u = -kp * zp - kv * zv;
//     Eigen::Vector3d Fd = mq * (u - g * e3 + trajectory.d2pd);
//     // ROS_INFO("Fd: %f %f %f", Fd(0), Fd(1), Fd(2));
//     double Td = Fd.norm();
//     Eigen::Vector3d r3d = -Fd / Td;
//     Eigen::Vector3d r3 = state.R*e3;
//     double T = Td*r3d.dot(r3);
//     // ROS_INFO("r3d: %f %f %f", r3d(0), r3d(1), r3d(2));
//     Eigen::Vector3d F = -T*state.R*e3;
//     Eigen::Vector3d dzv = F/mq + g*e3 - trajectory.d2pd;
//     Eigen::Vector3d dFd = mq*(-kp*zv - kv*dzv + trajectory.d3pd);
//     Eigen::Vector3d dr3d = S(r3d)*S(r3d)*dFd/Fd.norm();
//     Eigen::Vector3d zr = r3 - r3d;
//     Eigen::Vector3d omega;
//     double yaw = state.euler(0);
//     omega = - S(e3)*S(e3)*(state.R.transpose()*S(r3d)*dr3d + kr/hr*S(e3)*state.R.transpose()*r3d + Td/(mq*hr)*S(e3)*state.R.transpose()*(b*zp+zv)) - 0.1*(yaw-1.57)*e3;

//     T = clamp(T, 0, params.max_thrust);
//     control_input.thrust = T;
//     control_input.omega = omega;
//     if(use_polyval) {
//       T = clamp(polyval(T*1000/9.8,p1),0,1.0);
//       omega(0) = clamp(sign(omega(0))*polyval(abs(omega(0)), p3),-3.14,3.14);
//       omega(1) = clamp(sign(omega(1))*polyval(abs(omega(1)), p2),-3.14,3.14);
//       omega(2) = clamp(0*sign(omega(2))*polyval(abs(omega(2)), p4),-3.14,3.14);
//     }
//     control_input.mavlink_thrust = T;
//     control_input.mavlink_omega = omega;
//     state.updated = false; // 等待下一次状态更新
//     // control_input.omega = dwd_n;
//   }
// };

// } // namespace controller

// #endif // CONTROLLER_MYCONTROLLER_H

#ifndef CONTROLLER_MYCONTROLLER_H
#define CONTROLLER_MYCONTROLLER_H

#include "sz_indoor_controller/controller/controller.h"
#include "sz_indoor_controller/custom/system_params.h"
#include "sz_indoor_controller/custom/quadrotor_state.h"
#include "sz_indoor_controller/custom/mytrajectory.h"
#include "sz_indoor_controller/custom/quadrotor_control_input.h"

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <ros/ros.h>

namespace controller {

class QuadrotorControllerGanYu : public Controller {
public:
  common::SystemParams &params;
  common::QuadrotorState &state;
  common::MyTrajectory &trajectory;
  common::QuadrotorControlInput &control_input;

  double zp_int_z;      // Z轴位置误差积分
  double &zp_int_input;
  double int_limit;     // 积分限幅值
  double ki_z;          // Z轴积分增益
  ros::Time last_time;  // 【新增】上次调用时间
  bool first_call;      // 【新增】是否是第一次调用

  QuadrotorControllerGanYu(common::SystemParams &_params,
                           common::QuadrotorState &_state,
                           common::MyTrajectory &_trajectory,
                           common::QuadrotorControlInput &_control_input,
                           double &_zp_int_z)
      : params(_params),
        state(_state),
        trajectory(_trajectory),
        control_input(_control_input),
        zp_int_z(0.0),
        zp_int_input(_zp_int_z),       
        int_limit(5.0),      // 积分限幅，防止积分饱和
        ki_z(0.1),           // Z轴积分增益，可根据需要调整
        first_call(true) {}   // 第一次调用标志

  virtual ~QuadrotorControllerGanYu() {}

  double clamp(double x, double min_val, double max_val) {
    return x < min_val ? min_val : (x > max_val ? max_val : x);
  }

  // 支持 Eigen::Vector3d / Eigen::VectorXd 等任意列向量
  template <typename Derived>
  double polyval(double x, const Eigen::MatrixBase<Derived> &p) {
    double y = 0.0;
    int degree = static_cast<int>(p.size()) - 1;
    for (int i = 0; i < p.size(); ++i) {
      y += p(i) * std::pow(x, degree - i);
    }
    return y;
  }

  template <typename T>
  int sign(T value) {
    if (value > static_cast<T>(0)) return 1;
    if (value < static_cast<T>(0)) return -1;
    return 0;
  }

  // 反对称矩阵（叉乘矩阵）
  Eigen::Matrix3d S(const Eigen::Vector3d &vec) {
    Eigen::Matrix3d mat;
    mat << 0.0,     -vec(2),  vec(1),
           vec(2),   0.0,    -vec(0),
          -vec(1),   vec(0),  0.0;
    return mat;
  }

  // 投影矩阵
  Eigen::Matrix3d PI(const Eigen::Vector3d &vec) {
    return Eigen::Matrix3d::Identity() - vec * vec.transpose();
  }

  virtual void update() override {
    // ========= 参数读取 =========
    Eigen::Vector3d kp = params.quadrotor_kp;   // [kpx, kpy, kpz]
    Eigen::Vector3d kv = params.quadrotor_kv;   // [kvx, kvy, kvz]

    // ========= 【新增】计算时间步长 =========
    ros::Time current_time = ros::Time::now();
    double dt = 0.005;  // 默认值
    if (!first_call) {
      dt = (current_time - last_time).toSec();
      // 限制最大时间步长，防止异常跳变
      if (dt > 0.05) dt = 0.05;
      if (dt < 0.0001) dt = 0.0001;
    } else {
      first_call = false;
    }
    last_time = current_time;

    double kr = params.quadrotor_kr;
    double hr = params.quadrotor_hr;
    double mq = params.quadrotor_mq;
    double g  = params.g;
    bool use_polyval = params.use_polyval;

    Eigen::Vector3d p1 = params.p1;
    Eigen::Vector3d p2 = params.p2;
    Eigen::Vector3d p3 = params.p3;
    Eigen::Vector3d p4 = params.p4;

    Eigen::Vector3d e3(0.0, 0.0, 1.0);

    // ========= 误差计算 =========
    double b = 0.5;

    Eigen::Vector3d zp = state.p  - trajectory.pd;    // 位置误差
    Eigen::Vector3d zv = state.vi - trajectory.dpd;   // 速度误差

    // ========= 【新增】Z轴积分项更新 =========
    // 只对Z轴误差进行积分
    double zp_z = zp(2);  // Z轴位置误差
    zp_int_z += ki_z * zp_z * dt;  // 积分累加

    // 积分限幅，防止积分饱和
    if (zp_int_z > int_limit) zp_int_z = int_limit;
    if (zp_int_z < -int_limit) zp_int_z = -int_limit;

    // 对角增益：u = -Kp*zp - Kv*zv
    // 用 cwiseProduct 等价于 diag(kx,ky,kz)*z
    Eigen::Vector3d u = -kp.cwiseProduct(zp) - kv.cwiseProduct(zv);
    u(2) -= zp_int_z;  // 【新增】Z轴添加积分项

    // ========= 期望力 =========
    Eigen::Vector3d Fd = mq * (u - g * e3 + trajectory.d2pd);
    double Td = Fd.norm();

    // 避免除零
    if (Td < 1e-6) {
      Td = 1e-6;
    }

    Eigen::Vector3d r3d = -Fd / Td;
    Eigen::Vector3d r3  = state.R * e3;

    double T = Td * r3d.dot(r3);

    Eigen::Vector3d F = -T * state.R * e3;
    Eigen::Vector3d dzv = F / mq + g * e3 - trajectory.d2pd;

    // dFd 同样要对应改成矩阵/分轴增益形式
    Eigen::Vector3d dFd = mq * (
        -kp.cwiseProduct(zv)
        -kv.cwiseProduct(dzv)
        + trajectory.d3pd
    );

    Eigen::Vector3d dr3d = S(r3d) * S(r3d) * dFd / Td;
    Eigen::Vector3d zr = r3 - r3d;
    (void)zr;  // 如果暂时不用，避免编译器警告

    double yaw = state.euler(0);

    Eigen::Vector3d omega =
        -S(e3) * S(e3) * (
            state.R.transpose() * S(r3d) * dr3d
            + kr / hr * S(e3) * state.R.transpose() * r3d
            + Td / (mq * hr) * S(e3) * state.R.transpose() * (b * zp + zv)
        )
        - 0.1 * (yaw - 1.57) * e3;

    // ========= 原始控制量 =========
    T = clamp(T, 0.0, params.max_thrust);
    control_input.thrust = T;
    control_input.omega  = omega;
    zp_int_input = zp_int_z;

    // ========= 推力/角速度映射 =========
    if (use_polyval) {
      T = clamp(polyval(T * 1000.0 / 9.8, p1), 0.0, 1.0);

      omega(0) = clamp(
          sign(omega(0)) * polyval(std::abs(omega(0)), p3),
          -3.14, 3.14
      );

      omega(1) = clamp(
          sign(omega(1)) * polyval(std::abs(omega(1)), p2),
          -3.14, 3.14
      );

      omega(2) = clamp(
          0.0 * sign(omega(2)) * polyval(std::abs(omega(2)), p4),
          -3.14, 3.14
      );
    }

    control_input.mavlink_thrust = T;
    control_input.mavlink_omega  = omega;

    state.updated = false;  // 等待下一次状态更新
  }
};

}  // namespace controller

#endif  // CONTROLLER_MYCONTROLLER_H

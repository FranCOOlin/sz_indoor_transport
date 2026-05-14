// ROS node: qsls_state_publisher
// 功能：从 vrpn_client_node 中读取无人机与负载的 pose 与 twist，发布 QSLSState, 用于暂时模拟力传感器

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include <sz_indoor_controller/QSLSState.h> // 使用你的真实消息类型路径

class QSLSStatePublisher {
public:
    QSLSStatePublisher(ros::NodeHandle& nh) {
        pub_ = nh.advertise<sz_indoor_controller::QSLSState>("/X250/qsls_state", 10);

        sub_pose_Q_ = nh.subscribe("/vrpn_client_node/X250/pose", 10, &QSLSStatePublisher::poseQCallback, this);
        sub_twist_Q_ = nh.subscribe("/vrpn_client_node/X250/twist", 10, &QSLSStatePublisher::twistQCallback, this);
        sub_pose_L_ = nh.subscribe("/vrpn_client_node/Load/pose", 10, &QSLSStatePublisher::poseLCallback, this);
        sub_twist_L_ = nh.subscribe("/vrpn_client_node/Load/twist", 10, &QSLSStatePublisher::twistLCallback, this);

        initialized_ = false;

        timer_ = nh.createTimer(ros::Duration(1.0 / 200.0), &QSLSStatePublisher::timerCallback, this); // 200Hz 定时器
    }

private:
    ros::Publisher pub_;
    ros::Subscriber sub_pose_Q_, sub_twist_Q_, sub_pose_L_, sub_twist_L_;
    ros::Timer timer_;

    geometry_msgs::Point pQ_, pL_;
    geometry_msgs::Vector3 vQ_, vL_;
    geometry_msgs::Quaternion quatQ_;
    bool has_pose_Q_ = false, has_twist_Q_ = false, has_pose_L_ = false, has_twist_L_ = false;
    bool initialized_;

    void poseQCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        pQ_ = msg->pose.position;
        quatQ_ = msg->pose.orientation;
        has_pose_Q_ = true;

    }

    void twistQCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        vQ_ = msg->twist.linear;
        has_twist_Q_ = true;
    }

    void poseLCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        pL_ = msg->pose.position;
        has_pose_L_ = true;
    }

    void twistLCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        vL_ = msg->twist.linear;
        has_twist_L_ = true;

    }

    void timerCallback(const ros::TimerEvent&) {

        if (!(has_pose_Q_ && has_twist_Q_ && has_pose_L_ && has_twist_L_)) return;

        Eigen::Vector3d pQ(pQ_.x, pQ_.y, pQ_.z);
        Eigen::Vector3d pL(pL_.x, pL_.y, pL_.z);
        Eigen::Vector3d vQ(vQ_.x, vQ_.y, vQ_.z);
        Eigen::Vector3d vL(vL_.x, vL_.y, vL_.z);

        Eigen::Vector3d delta = pL - pQ;
        double norm_delta = delta.norm();
        if (norm_delta < 1e-5) return; // 避免除零

        Eigen::Vector3d q = delta / norm_delta;

        Eigen::Vector3d vel_diff = vL - vQ;
        Eigen::Matrix3d S;
        S <<     0, -q.z(),  q.y(),
             q.z(),     0, -q.x(),
            -q.y(),  q.x(),     0;

        Eigen::Vector3d w = (1.0 / norm_delta) * S * vel_diff;

        // 发布消息
        sz_indoor_controller::QSLSState msg;
        msg.pL.x = pL_.x; msg.pL.y = pL_.y; msg.pL.z = pL_.z;
        msg.vL = vL_;
        msg.pQ.x = pQ_.x; msg.pQ.y = pQ_.y; msg.pQ.z = pQ_.z;
        msg.vQ = vQ_;
        msg.q.x = q.x(); msg.q.y = q.y(); msg.q.z = q.z();
        msg.w.x = w.x(); msg.w.y = w.y(); msg.w.z = w.z();
        msg.quat = quatQ_;
        msg.bL.x = msg.bL.y = msg.bL.z = 0;
        msg.bQ.x = msg.bQ.y = msg.bQ.z = 0;
        pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "qsls_state_publisher");
    ros::NodeHandle nh;
    QSLSStatePublisher node(nh);
    ros::spin();
    return 0;
}

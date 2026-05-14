#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <sz_indoor_controller/TrajPoint.h>

class FormationPlanner {
private:
    ros::NodeHandle nh;
    ros::Publisher traj_pub[4];
    ros::Subscriber flag_subs[4];
    ros::Subscriber state_subs[4];
    ros::Timer timer;

  
    Eigen::Vector3d current_pos[4];    
    Eigen::Vector3d d_offsets[4];      
    Eigen::Vector3d formation_offsets[4];
    int uav_ids[4] = {0, 1, 3, 4};     
    bool ready_flags[4] = {false, false, false, false};
    bool all_ready = false;

   
    double radius = 0.0;
    double omega = 0.3;
    double start_angle = 0.0;
    Eigen::Vector3d circle_center;
    double start_time;

public:
    FormationPlanner() {
        ros::NodeHandle pnh("~");
        pnh.param("omega", omega, omega);
        circle_center << 0.0, 0.0, 0.0;

        for (int i = 0; i < 4; ++i) {
            int id = uav_ids[i];
            std::string uav_ns = "/uav17" + std::to_string(id);
            traj_pub[i] = nh.advertise<sz_indoor_controller::TrajPoint>(uav_ns + "/planning/traj_point", 10);

            std::string flag_topic = "uav17" + std::to_string(id) + "/traj_generation_flag";
            flag_subs[i] = nh.subscribe<std_msgs::Bool>(
                flag_topic, 1, boost::bind(&FormationPlanner::flagCallback, this, _1, i));

            std::string state_topic = uav_ns + "/quadrotor_state";
            state_subs[i] = nh.subscribe<std_msgs::Float64MultiArray>(
                state_topic, 10, boost::bind(&FormationPlanner::stateCallback, this, _1, i));

            current_pos[i].setZero();
        }

        ros::Time::waitForValid();
        timer = nh.createTimer(ros::Duration(0.01), &FormationPlanner::timerCallback, this);
        ROS_INFO("Formation Planner Initialized. Waiting for UAVs 170, 171, 173, 174...");
        ROS_INFO("Master trajectory params: omega=%.3f, center_xy=[0.000, 0.000]", omega);
    }

    void stateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg, int index) {
        if (msg->data.size() >= 3) {
            current_pos[index] << msg->data[0], msg->data[1], msg->data[2];
        }
    }
    
    void flagCallback(const std_msgs::Bool::ConstPtr& msg, int index) {
            if (msg->data && !ready_flags[index]) {
                ready_flags[index] = true;
                ROS_INFO("UAV 17%d is READY at pos: [%.2f, %.2f, %.2f]", 
                         uav_ids[index], current_pos[index].x(), current_pos[index].y(), current_pos[index].z());
    
                if (!all_ready) {
                    bool check = true;
                    for (int j = 0; j < 4; ++j) {
                        if (!ready_flags[j]) { check = false; break; }
                    }
    
                    if (check) {
                        for (int j = 0; j < 4; ++j) {
                            d_offsets[j] = current_pos[j];
                        }
                        Eigen::Vector3d master_radius = d_offsets[0] - circle_center;
                        start_angle = atan2(master_radius.y(), master_radius.x());
                        radius = master_radius.head<2>().norm();
                        circle_center.z() = d_offsets[0].z();

                        Eigen::Vector3d r1_0(sin(start_angle), -cos(start_angle), 0.0);
                        Eigen::Vector3d r3_0(0.0, 0.0, 1.0);
                        Eigen::Vector3d r2_0 = r3_0.cross(r1_0);
                        Eigen::Matrix3d R0;
                        R0 << r1_0, r2_0, r3_0;
                        for (int j = 0; j < 4; ++j) {
                            formation_offsets[j] = R0.transpose() * (d_offsets[j] - d_offsets[0]);
                        }
                        all_ready = true;
                        start_time = ros::Time::now().toSec();
                        ROS_INFO("ALL UAVS READY! Offsets recorded. Starting trajectory...");
                        ROS_INFO("Master starts from current position [%.3f, %.3f, %.3f], circle center=[%.3f, %.3f, %.3f], actual radius=%.3f, start_angle=%.3f",
                                 d_offsets[0].x(), d_offsets[0].y(), d_offsets[0].z(),
                                 circle_center.x(), circle_center.y(), circle_center.z(), radius, start_angle);
                    }
                }
            }
        }
  

    void timerCallback(const ros::TimerEvent&) {
        if (!all_ready) return;

        double t = ros::Time::now().toSec() - start_time;
        double theta = start_angle - omega * t;
        double ct = cos(theta);
        double st = sin(theta);
    
        Eigen::Vector3d pL(circle_center.x() + radius * ct, circle_center.y() + radius * st , circle_center.z());
        Eigen::Vector3d vL(radius * omega * st, -radius * omega * ct, 0.0);
        Eigen::Vector3d aL(-radius * omega * omega * ct, -radius * omega * omega * st, 0.0);
        Eigen::Vector3d jL(-radius * pow(omega, 3) * st, radius * pow(omega, 3) * ct, 0.0);
        
        Eigen::Vector3d r1 = vL.normalized(); 
        Eigen::Vector3d r3(0.0, 0.0, 1.0);     
        Eigen::Vector3d r2 = r3.cross(r1);    
    
        
        Eigen::Vector3d dr1(-omega * ct, -omega * st, 0.0);
        Eigen::Vector3d ddr1(-omega * omega * st, omega * omega * ct, 0.0);
        Eigen::Vector3d dddr1(omega * omega * omega * ct, omega * omega * omega * st, 0.0);
    
        Eigen::Vector3d dr3(0, 0, 0);
        Eigen::Vector3d dr2 = r3.cross(dr1);
        Eigen::Vector3d ddr2 = r3.cross(ddr1);
        Eigen::Vector3d dddr2 = r3.cross(dddr1);
    
        Eigen::Matrix3d R_TI, dR_TI, ddR_TI, dddR_TI;
        R_TI << r1, r2, r3;
        dR_TI << dr1, dr2, dr3;
        ddR_TI << ddr1, ddr2, dr3;
        dddR_TI << dddr1, dddr2, dr3;
        
        for (int i = 0; i < 4; ++i) {
            sz_indoor_controller::TrajPoint msg_out;

            Eigen::Vector3d formation_offset = formation_offsets[i];
            Eigen::Vector3d pFi = pL + R_TI * formation_offset;
            Eigen::Vector3d dpFi = vL + dR_TI * formation_offset;
            Eigen::Vector3d ddpFi = aL + ddR_TI * formation_offset;
            Eigen::Vector3d dddpFi = jL + dddR_TI * formation_offset;

            msg_out.pd.x = pFi.x();
            msg_out.pd.y = pFi.y(); 
            msg_out.pd.z = pFi.z();
            
            msg_out.dpd.x = dpFi.x(); 
            msg_out.dpd.y = dpFi.y(); 
            msg_out.dpd.z = dpFi.z();
            
            msg_out.d2pd.x = ddpFi.x(); 
            msg_out.d2pd.y = ddpFi.y(); 
            msg_out.d2pd.z = ddpFi.z();
            
            msg_out.d3pd.x = dddpFi.x(); 
            msg_out.d3pd.y = dddpFi.y(); 
            msg_out.d3pd.z = dddpFi.z();

            msg_out.yawd = atan2(r1.y(), r1.x());
            msg_out.yawd_dot = omega;

            traj_pub[i].publish(msg_out);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "formation_traj_node");
    FormationPlanner fp;
    ros::spin();
    return 0;
}

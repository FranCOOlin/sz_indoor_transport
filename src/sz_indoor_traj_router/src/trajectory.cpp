#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <sz_indoor_controller/TrajPoint.h>

class FormationPlanner {
private:
    ros::NodeHandle nh;
    ros::Publisher traj_pub[4];
    ros::Subscriber flag_subs[4];
    ros::Subscriber state_subs[4];
    ros::Subscriber trajectory_request_sub;
    ros::Timer timer;

  
    Eigen::Vector3d current_pos[4];    
    Eigen::Vector3d d_offsets[4];      
    Eigen::Vector3d circle_formation_offsets[4];
    Eigen::Vector3d eight_formation_offsets[4];
    Eigen::Vector3d virtual_eight_formation_offsets[4];
    int uav_ids[4] = {0, 1, 3, 4};     
    bool ready_flags[4] = {false, false, false, false};
    bool all_ready = false;
    bool trajectory_request_received = false;
    bool trajectory_started = false;
    int pending_traj_choice = 0;
    int active_traj_choice = 0;

   
    double radius = 0.0;
    double omega = 0.3;
    double start_angle = 0.0;
    Eigen::Vector3d circle_center;
    Eigen::Vector3d virtual_leader_0;
    double start_time;

public:
    FormationPlanner() {
        ros::NodeHandle pnh("~");
        pnh.param("omega", omega, omega);
        circle_center << 0.0, 0.0, 0.0;
        trajectory_request_sub = nh.subscribe<std_msgs::String>(
            "/trajectory_request", 1, &FormationPlanner::trajectoryRequestCallback, this);

        for (int i = 0; i < 4; ++i) {
            int id = uav_ids[i];
            std::string uav_ns = "/uav17" + std::to_string(id);
            traj_pub[i] = nh.advertise<sz_indoor_controller::TrajPoint>(uav_ns + "/planning/traj_point", 10);

            std::string flag_topic = "/uav17" + std::to_string(id) + "/traj_generation_flag";
            flag_subs[i] = nh.subscribe<std_msgs::Bool>(
                flag_topic, 1, boost::bind(&FormationPlanner::flagCallback, this, _1, i));

            std::string state_topic = uav_ns + "/quadrotor_state";
            state_subs[i] = nh.subscribe<std_msgs::Float64MultiArray>(
                state_topic, 10, boost::bind(&FormationPlanner::stateCallback, this, _1, i));

            current_pos[i].setZero();
        }

        ros::Time::waitForValid();
        timer = nh.createTimer(ros::Duration(0.01), &FormationPlanner::timerCallback, this);
        ROS_INFO("Formation Planner Initialized. Waiting for UAVs 1, 2, 3, 4...");
        ROS_INFO("Master trajectory params: omega=%.3f, center_xy=[0.000, 0.000]", omega);
    }

    void stateCallback(const std_msgs::Float64MultiArray::ConstPtr& msg, int index) {
        if (msg->data.size() >= 3) {
            current_pos[index] << msg->data[0], msg->data[1], msg->data[2];
        }
    }

    void trajectoryRequestCallback(const std_msgs::String::ConstPtr& msg) {
        int requested_choice = 0;

        if (msg->data == "trajectory1") {
            requested_choice = 1;
        } else if (msg->data == "trajectory2") {
            requested_choice = 2;
        } else if (msg->data == "trajectory3") {
            requested_choice = 3;
        } else {
            ROS_WARN("Unsupported /trajectory_request: %s. Use trajectory1 for circle, trajectory2 for master figure-eight, trajectory3 for virtual-leader figure-eight.",
                     msg->data.c_str());
            return;
        }

        if (trajectory_started && requested_choice == active_traj_choice) {
            ROS_INFO("Trajectory request received again: %s. Current trajectory keeps running.",
                     msg->data.c_str());
            return;
        }

        pending_traj_choice = requested_choice;
        trajectory_request_received = true;
        trajectory_started = false;
        ROS_INFO("Trajectory request received: %s, pending choice=%d", msg->data.c_str(), pending_traj_choice);
        tryStartTrajectory();
    }

    Eigen::Matrix3d makeFrame(const Eigen::Vector3d& velocity) {
        Eigen::Vector3d r1 = velocity.normalized();
        Eigen::Vector3d r3(0.0, 0.0, 1.0);
        Eigen::Vector3d r2 = r3.cross(r1);
        Eigen::Matrix3d R;
        R << r1, r2, r3;
        return R;
    }

    void computeCircleMaster(double t, Eigen::Vector3d& pL, Eigen::Vector3d& vL,
                             Eigen::Vector3d& aL, Eigen::Vector3d& jL) {
        double theta = start_angle - omega * t;
        double ct = cos(theta);
        double st = sin(theta);

        pL << circle_center.x() + radius * ct, circle_center.y() + radius * st, circle_center.z();
        vL << radius * omega * st, -radius * omega * ct, 0.0;
        aL << -radius * omega * omega * ct, -radius * omega * omega * st, 0.0;
        jL << -radius * pow(omega, 3) * st, radius * pow(omega, 3) * ct, 0.0;
    }

    void computeEightTrajectory(const Eigen::Vector3d& start, double t, Eigen::Vector3d& pL,
                                Eigen::Vector3d& vL, Eigen::Vector3d& aL, Eigen::Vector3d& jL) {
        double theta = omega * t;
        double st = sin(theta);
        double ct = cos(theta);
        double c2t = cos(2.0 * theta);

        pL << start.x() + st,
              start.y() + ct * st,
              start.z();
        vL << omega * ct,
              omega * c2t,
              0.0;
        aL << -omega * omega * st,
              -2.0 * omega * omega * sin(2.0 * theta),
              0.0;
        jL << -pow(omega, 3) * ct,
              -4.0 * pow(omega, 3) * c2t,
              0.0;
    }

    void computeEightMaster(double t, Eigen::Vector3d& pL, Eigen::Vector3d& vL,
                            Eigen::Vector3d& aL, Eigen::Vector3d& jL) {
        computeEightTrajectory(d_offsets[0], t, pL, vL, aL, jL);
    }

    void computeVirtualEightMaster(double t, Eigen::Vector3d& pL, Eigen::Vector3d& vL,
                                   Eigen::Vector3d& aL, Eigen::Vector3d& jL) {
        computeEightTrajectory(virtual_leader_0, t, pL, vL, aL, jL);
    }

    void computeMaster(double t, Eigen::Vector3d& pL, Eigen::Vector3d& vL,
                       Eigen::Vector3d& aL, Eigen::Vector3d& jL) {
        if (active_traj_choice == 3) {
            computeVirtualEightMaster(t, pL, vL, aL, jL);
        } else if (active_traj_choice == 2) {
            computeEightMaster(t, pL, vL, aL, jL);
        } else {
            computeCircleMaster(t, pL, vL, aL, jL);
        }
    }

    Eigen::Matrix3d computeFrame(double t) {
        Eigen::Vector3d pL, vL, aL, jL;
        computeMaster(t, pL, vL, aL, jL);
        return makeFrame(vL);
    }

    void tryStartTrajectory() {
        if (!all_ready || !trajectory_request_received || trajectory_started) {
            return;
        }

        active_traj_choice = pending_traj_choice;
        start_time = ros::Time::now().toSec();
        trajectory_started = true;
        ROS_INFO("Trajectory started: choice=%d, t reset to 0.", active_traj_choice);
    }
    
    void flagCallback(const std_msgs::Bool::ConstPtr& msg, int index) {
            if (msg->data && !ready_flags[index]) {
                ready_flags[index] = true;
                ROS_INFO("UAV %d is READY at pos: [%.2f, %.2f, %.2f]", 
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
                        virtual_leader_0.setZero();
                        for (int j = 0; j < 4; ++j) {
                            virtual_leader_0 += d_offsets[j];
                        }
                        virtual_leader_0 /= 4.0;

                        Eigen::Matrix3d circle_R0 = makeFrame(Eigen::Vector3d(radius * omega * sin(start_angle),
                                                                               -radius * omega * cos(start_angle),
                                                                               0.0));
                        Eigen::Matrix3d eight_R0 = makeFrame(Eigen::Vector3d(omega, omega, 0.0));
                        for (int j = 0; j < 4; ++j) {
                            circle_formation_offsets[j] = circle_R0.transpose() * (d_offsets[j] - d_offsets[0]);
                            eight_formation_offsets[j] = eight_R0.transpose() * (d_offsets[j] - d_offsets[0]);
                            virtual_eight_formation_offsets[j] = eight_R0.transpose() * (d_offsets[j] - virtual_leader_0);
                        }
                        all_ready = true;
                        ROS_INFO("ALL UAVS READY! Offsets recorded. Starting trajectory...");
                        ROS_INFO("Master starts from current position [%.3f, %.3f, %.3f], circle center=[%.3f, %.3f, %.3f], actual radius=%.3f, start_angle=%.3f",
                                 d_offsets[0].x(), d_offsets[0].y(), d_offsets[0].z(),
                                 circle_center.x(), circle_center.y(), circle_center.z(), radius, start_angle);
                        ROS_INFO("Virtual leader initial position [%.3f, %.3f, %.3f]",
                                 virtual_leader_0.x(), virtual_leader_0.y(), virtual_leader_0.z());
                        tryStartTrajectory();
                    }
                }
            }
        }
  

    void timerCallback(const ros::TimerEvent&) {
        if (!trajectory_started) return;

        double t = ros::Time::now().toSec() - start_time;
        Eigen::Vector3d pL, vL, aL, jL;
        computeMaster(t, pL, vL, aL, jL);

        double h = 0.001;
        Eigen::Matrix3d R_TI = computeFrame(t);
        Eigen::Matrix3d Rp = computeFrame(t + h);
        Eigen::Matrix3d Rm = computeFrame(t - h);
        Eigen::Matrix3d Rpp = computeFrame(t + 2.0 * h);
        Eigen::Matrix3d Rmm = computeFrame(t - 2.0 * h);
        Eigen::Matrix3d dR_TI = (Rp - Rm) / (2.0 * h);
        Eigen::Matrix3d ddR_TI = (Rp - 2.0 * R_TI + Rm) / (h * h);
        Eigen::Matrix3d dddR_TI = (Rpp - 2.0 * Rp + 2.0 * Rm - Rmm) / (2.0 * h * h * h);
        Eigen::Vector3d r1 = R_TI.col(0);
        Eigen::Vector3d dr1 = dR_TI.col(0);
        
        for (int i = 0; i < 4; ++i) {
            sz_indoor_controller::TrajPoint msg_out;

            Eigen::Vector3d formation_offset = circle_formation_offsets[i];
            if (active_traj_choice == 2) {
                formation_offset = eight_formation_offsets[i];
            } else if (active_traj_choice == 3) {
                formation_offset = virtual_eight_formation_offsets[i];
            }
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
            msg_out.yawd_dot = r1.x() * dr1.y() - r1.y() * dr1.x();

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

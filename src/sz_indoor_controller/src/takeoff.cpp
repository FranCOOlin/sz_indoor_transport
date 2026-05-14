#include <ros/ros.h>
#include <cmath>
//mavros message
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
//ros message
#include <sz_indoor_controller/TrajPoint.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>

double height;
double rate_num;
double takeoff_start_time = 0.0;
double t = 0.0;
int takeoff_ing_flag = 0;
bool traj_flag = false;
bool land_flag = false;
bool takeoff_flag = false;
float takeoff_and_land_time = 5.0;
std::string uav_id;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped takeoff_pose;
mavros_msgs::SetMode offboard_set_mode;
mavros_msgs::CommandBool arm_cmd;
sz_indoor_controller::TrajPoint origin_point;
sz_indoor_controller::TrajPoint takeoff_trajectory;
sz_indoor_controller::TrajPoint track_trajectory;
sz_indoor_controller::TrajPoint land_trajectory;
ros::Subscriber state_sub;
ros::Subscriber pose_sub;
ros::Subscriber traj_sub;
ros::Subscriber land_sub;
ros::Subscriber takeoff_sub;
ros::Publisher traj_pub;
ros::Publisher traj_generation_flag_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void poseCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != 10)
    {
        ROS_WARN("Invalid state message: expected 3 elements, received %lu", msg->data.size());
        return;
    }
    current_pose.pose.position.x = msg->data[0];
    current_pose.pose.position.y = msg->data[1];
    current_pose.pose.position.z = msg->data[2];
}

void landCallback(const std_msgs::Bool::ConstPtr& msg)
{
    land_flag = msg->data;
}

void takeoffCallback(const std_msgs::Bool::ConstPtr& msg)
{
    takeoff_flag = msg->data;
}

sz_indoor_controller::TrajPoint Set_height(double t1,double takeoff_and_land_time,float x,float y)
{
    sz_indoor_controller::TrajPoint trajectory;
    if(t1 > takeoff_and_land_time) //hover
    {
        trajectory.pd.x = x; trajectory.pd.y = y; trajectory.pd.z = -height;
        trajectory.dpd.x = 0.0; trajectory.dpd.y = 0.0; trajectory.dpd.z = 0.0;
        trajectory.d2pd.x = 0.0; trajectory.d2pd.y = 0.0; trajectory.d2pd.z = 0.0;
        trajectory.d3pd.x = 0.0; trajectory.d3pd.y = 0.0; trajectory.d3pd.z = 0.0;
        trajectory.d4pd.x = 0.0; trajectory.d4pd.y = 0.0; trajectory.d4pd.z = 0.0;
        trajectory.d5pd.x = 0.0; trajectory.d5pd.y = 0.0; trajectory.d5pd.z = 0.0;
        trajectory.yawd = 0.0;
        trajectory.yawd_dot = 0.0;
    }
    else if(t1 < 0.0) //land
    {
        trajectory.pd.x = x; trajectory.pd.y = y; trajectory.pd.z = 1.0; //avoid flying
        trajectory.dpd.x = 0.0; trajectory.dpd.y = 0.0; trajectory.dpd.z = 0.0;
        trajectory.d2pd.x = 0.0; trajectory.d2pd.y = 0.0; trajectory.d2pd.z = 0.0;
        trajectory.d3pd.x = 0.0; trajectory.d3pd.y = 0.0; trajectory.d3pd.z = 0.0;
        trajectory.d4pd.x = 0.0; trajectory.d4pd.y = 0.0; trajectory.d4pd.z = 0.0;
        trajectory.d5pd.x = 0.0; trajectory.d5pd.y = 0.0; trajectory.d5pd.z = 0.0;
        trajectory.yawd = 0.0;
        trajectory.yawd_dot = 0.0;
    }
    else
    {
        double t_s = t1/takeoff_and_land_time; //dt_s = dt / takeoff_and_land_time
        trajectory.pd.x = x; trajectory.pd.y = y; trajectory.pd.z = -height * (3*t_s*t_s-2*t_s*t_s*t_s);
        trajectory.dpd.x = 0.0; trajectory.dpd.y = 0.0; trajectory.dpd.z = -height * (6*t_s-6*t_s*t_s) / takeoff_and_land_time;
        trajectory.d2pd.x = 0.0; trajectory.d2pd.y = 0.0; trajectory.d2pd.z = -height * (6-12*t_s)/(takeoff_and_land_time*takeoff_and_land_time);
        trajectory.d3pd.x = 0.0; trajectory.d3pd.y = 0.0; trajectory.d3pd.z = 12*height/(takeoff_and_land_time*takeoff_and_land_time*takeoff_and_land_time);
        trajectory.d4pd.x = 0.0; trajectory.d4pd.y = 0.0; trajectory.d4pd.z = 0.0;
        trajectory.d5pd.x = 0.0; trajectory.d5pd.y = 0.0; trajectory.d5pd.z = 0.0;
        trajectory.yawd = 0.0;
        trajectory.yawd_dot = 0.0;
    }
    return trajectory;
}

void trajCallback(const sz_indoor_controller::TrajPoint::ConstPtr& msg)
{
    track_trajectory.pd.x = msg->pd.x; track_trajectory.pd.y = msg->pd.y; track_trajectory.pd.z = msg->pd.z;
    track_trajectory.dpd.x = msg->dpd.x; track_trajectory.dpd.y = msg->dpd.y; track_trajectory.dpd.z = msg->dpd.z;
    track_trajectory.d2pd.x = msg->d2pd.x; track_trajectory.d2pd.y = msg->d2pd.y; track_trajectory.d2pd.z = msg->d2pd.z;
    track_trajectory.d3pd.x = msg->d3pd.x; track_trajectory.d3pd.y = msg->d3pd.y; track_trajectory.d3pd.z = msg->d3pd.z;
    track_trajectory.d4pd.x = msg->d4pd.x; track_trajectory.d4pd.y = msg->d4pd.y; track_trajectory.d4pd.z = msg->d4pd.z;
    track_trajectory.d5pd.x = msg->d5pd.x; track_trajectory.d5pd.y = msg->d5pd.y; track_trajectory.d5pd.z = msg->d5pd.z;
    track_trajectory.yawd = msg->yawd;
    track_trajectory.yawd_dot = msg->yawd_dot;
    traj_flag = true;
}

bool Takeoff(ros::Rate &rate)
{
    origin_point.pd.x = 0.0; origin_point.pd.y = 0.0; origin_point.pd.z = 1.0; //avoid flying
    origin_point.dpd.x = 0.0; origin_point.dpd.y = 0.0; origin_point.dpd.z = 0.0;
    origin_point.d2pd.x = 0.0; origin_point.d2pd.y = 0.0; origin_point.d2pd.z = 0.0;
    origin_point.d3pd.x = 0.0; origin_point.d3pd.y = 0.0; origin_point.d3pd.z = 0.0;
    origin_point.d4pd.x = 0.0; origin_point.d4pd.y = 0.0; origin_point.d4pd.z = 0.0;
    origin_point.d5pd.x = 0.0; origin_point.d5pd.y = 0.0; origin_point.d5pd.z = 0.0;
    origin_point.yawd = 0.0;
    origin_point.yawd_dot = 0.0;
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected to FCU");
    if(current_state.mode != "OFFBOARD")
    {
        for (int i = 0; i < int(rate_num * 1.2); i++)
                {
                    traj_pub.publish(origin_point);
                    rate.sleep();
                }
    }
    offboard_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    while (ros::ok())
        {
            if (current_state.mode != "OFFBOARD")
            {
                if (set_mode_client.call(offboard_set_mode) && offboard_set_mode.response.mode_sent)
                {
                    ROS_INFO("OFFBOARD mode enabling");
                }
                else
                {
                    traj_pub.publish(origin_point);
                }
            }
            else
            {
                if (!current_state.armed)
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        takeoff_start_time = ros::Time::now().toSec() + 0.2;
                        takeoff_pose.pose.position.x = current_pose.pose.position.x;
                        takeoff_pose.pose.position.y = current_pose.pose.position.y;
                    }
                }
                else if(takeoff_ing_flag == 0)
                {
                    takeoff_start_time = ros::Time::now().toSec() + 0.2;
                    traj_pub.publish(origin_point);
                    takeoff_pose.pose.position.x = current_pose.pose.position.x;
                    takeoff_pose.pose.position.y = current_pose.pose.position.y;
                }
            }
            if (takeoff_start_time > 0.1 || takeoff_ing_flag == 1)
            {
                takeoff_ing_flag = 1;
                t = std::max(ros::Time::now().toSec() - takeoff_start_time, 0.0);
                takeoff_trajectory = Set_height(t,takeoff_and_land_time,takeoff_pose.pose.position.x,takeoff_pose.pose.position.y);
                traj_pub.publish(takeoff_trajectory);
            }
            if (t > takeoff_and_land_time - 0.02 || land_flag)
            {
                ROS_INFO("Reach target height");
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    return true;
}

bool HoverAndWait(ros::Rate &rate)
{
    float hover_start_time = ros::Time::now().toSec();
    while (ros::ok())
        {
            if((traj_flag && (ros::Time::now().toSec() - hover_start_time > 2.0)) || land_flag)
            {
                ROS_INFO("get_traj");
                break;
            }
            else
            {
                sz_indoor_controller::TrajPoint target_pose = Set_height(2*takeoff_and_land_time,takeoff_and_land_time,takeoff_pose.pose.position.x,takeoff_pose.pose.position.y); //hover at height
                traj_pub.publish(target_pose);
                if(ros::Time::now().toSec() - hover_start_time > 2.0)
                {
                    std_msgs::Bool traj_gen_flag;
                    traj_gen_flag.data = true;
                    traj_generation_flag_pub.publish(traj_gen_flag);
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    return true;
}

void Land(ros::Rate &rate)
{
    double land_start_time = ros::Time::now().toSec();
    height = -current_pose.pose.position.z;
    while(ros::ok())
    {
        t = ros::Time::now().toSec() - land_start_time;
        t = takeoff_and_land_time - t;
        land_trajectory = Set_height(t,takeoff_and_land_time,current_pose.pose.position.x,current_pose.pose.position.y);
        traj_pub.publish(land_trajectory);
        ros::spinOnce();
        rate.sleep();
        if(t < -1.0)
        {
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void Receive_and_Send_traj(ros::Rate &rate)
{
    while (ros::ok() && !land_flag)
        {
            if(traj_flag)
            {
                traj_pub.publish(track_trajectory);
            }
            ros::spinOnce();
            rate.sleep();
        }
    Land(rate);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "takeoff_node",ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    nh_private.param<double>("height", height, 1.0);
    nh_private.param<double>("rate", rate_num, 200.0);
    nh_private.param<std::string>("uav_id", uav_id, std::string("uav173"));

    // 如果 launch 里传的是 "uav173" 而不是 "/uav173"，这里自动补上前导 /
    if (!uav_id.empty() && uav_id.front() != '/')
    {
      uav_id = "/" + uav_id;
    }

    ROS_INFO_STREAM("takeoff node using uav_id: " << uav_id);
    ros::Rate rate(rate_num);
    state_sub = nh.subscribe(uav_id + "/mavros/state", 1, stateCallback);
    pose_sub = nh.subscribe(uav_id + "/quadrotor_state", 1, poseCallback);
    traj_sub = nh.subscribe(uav_id + "/planning/traj_point", 1, trajCallback);
    //traj_sub = nh.subscribe("/uav2/planning/traj_point", 1, trajCallback);
    land_sub = nh.subscribe("/land_cmd", 1, landCallback);
    takeoff_sub = nh.subscribe("/takeoff_cmd", 1, takeoffCallback);
    traj_pub = nh.advertise<sz_indoor_controller::TrajPoint>(uav_id + "/trajectory", 1);
    traj_generation_flag_pub = nh.advertise<std_msgs::Bool>(uav_id + "/traj_generation_flag", 1);
    //traj_generation_flag_pub = nh.advertise<std_msgs::Bool>("/uav2/traj_generation_flag", 1);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(uav_id + "/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(uav_id + "/mavros/set_mode");
    ros::service::waitForService(uav_id + "/mavros/cmd/arming");
    ros::service::waitForService(uav_id + "/mavros/set_mode");
    while (!takeoff_flag)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Begin to takeoff");
    if (Takeoff(rate))
    {
        if(HoverAndWait(rate))
        {
            if(land_flag)
            {
                Land(rate);
            }
            else
            {
                Receive_and_Send_traj(rate);
            }
        }
    }
    return 0;
}

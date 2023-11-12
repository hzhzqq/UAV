/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
// #include <easyrobot_demo/include/easyrobot_demo/easyrobot.hpp>
// #include <easyrobot.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
using namespace Eigen;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double to_radians(double degrees){ return degrees / (180.0/M_PI);}
double to_degrees(double radians){ return radians * (180.0/M_PI);}

Vector3d cam_pos_trans_enu(Vector3d vehicle_local_pos, Vector3d cam_pos){
    double yaw = 90.0;
    double yaw_ = yaw - 90.0; // 北东地 转到 东北天

    Vector3d tar_pos_temp ;

    ROS_INFO("local: %.2f, %.2f, %.2f ", \
        vehicle_local_pos(0), vehicle_local_pos(1),vehicle_local_pos(2));
    ROS_INFO("camera zuobiaoxi location : %.2f, %.2f, %.2f ", \
        cam_pos(0), cam_pos(1),cam_pos(2));
    
    // 方法1
    // Eigen::Vector3d ea( -0,to_radians(-yaw_), 0);

    // //3.1 欧拉角转换为旋转矩阵 绕y轴逆时针旋转 _yaw_deg 
    // Eigen::Matrix3d rotation_matrix3;
    // rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
    //                    Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
    //                    Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    // tar_pos_temp = rotation_matrix3 * cam_pos;


    // 方法2 逆时针转yaw
    tar_pos_temp(0) =   cos(to_radians(yaw)) * cam_pos(0) + sin(to_radians(yaw)) * cam_pos(2);
    tar_pos_temp(2) = - sin(to_radians(yaw)) * cam_pos(0) + cos(to_radians(yaw)) * cam_pos(2);


    
    ROS_INFO("1 after rotate yaw  : %.2f, %.2f, %.2f ", \
        tar_pos_temp(0), tar_pos_temp(1),tar_pos_temp(2));

    // 方法1
    // 绕x轴顺时针旋转90度，到东北天坐标系
    // ea(0) = 0.0;
    // ea(1) = 0.0;
    // ea(2) = M_PI/2;
    // rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
    //                    Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) * 
    //                    Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());
    // tar_pos_temp = rotation_matrix3 * tar_pos_temp;

    // 方法2 逆时针转yaw
    Vector3d tar_pos_temp2=tar_pos_temp ;

    tar_pos_temp2(1) =   cos(to_radians(M_PI_2)) * tar_pos_temp(1) + sin(to_radians(M_PI_2)) * tar_pos_temp(2);
    tar_pos_temp2(2) = - sin(to_radians(M_PI_2)) * tar_pos_temp(1) + cos(to_radians(M_PI_2)) * tar_pos_temp(2);

    tar_pos_temp = tar_pos_temp2;
    ROS_INFO("2 after rotate x : %.2f, %.2f, %.2f ", \
        tar_pos_temp(0), tar_pos_temp(1),tar_pos_temp(2));
    tar_pos_temp += vehicle_local_pos;
    ROS_INFO("3 plus local : %.2f, %.2f, %.2f ", \
        tar_pos_temp(0), tar_pos_temp(1),tar_pos_temp(2));
        
    return tar_pos_temp;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Easyrobot uav(nh,"");
    // uav.getRobotState(true);
    // sleep(2);
    // uav.takeoff(1);
    // sleep(5);
    // sleep(10);
    // uav.changeMode("RTL");
    // sleep(10);
    

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.mode_sent){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }

        // local_pos_pub.publish(pose);
        // uav.control(0, 2.0, 0.0, 2.0, 1.57);  // (int control_type, float x, float y, float z, float yaw)

        // Vector3d local ={0.0,0.0,0.0}, cam_pos= {0.0,0.0,1.0},res;
        Vector3d local ={-1.0,0.0,0.0}, cam_pos= {0.0,0.0,2.0},res;
        res=cam_pos_trans_enu(local, cam_pos);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

#ifndef EASYROBOT_HPP
#define EASYROBOT_HPP

#include <cmath>
#include <Eigen/Eigen>
#include <locale>
#include "ros/ros.h"
#include "easyrobot_msgs/ControlCommand.h"
#include "easyrobot_msgs/RobotState.h"

// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f

class Easyrobot
{
public:
    Easyrobot(ros::NodeHandle &n, std::string ns);
    ros::Subscriber robot_state_sub;
    ros::Publisher robot_control_pub;
    easyrobot_msgs::ControlCommand command;
    easyrobot_msgs::RobotState state;
    void robotStateCB(const easyrobot_msgs::RobotStateConstPtr& msg);

    bool gotoPosEnu(bool block, float pos_x, float pos_y, float pos_z);
    bool gotoPosEnu(bool block, float pos_x, float pos_y, float pos_z, float yaw);
    bool gotoPosBody(bool block, float pos_x, float pos_y, float pos_z);
    bool gotoPosBody(bool block, float pos_x, float pos_y, float pos_z, float yaw);
    bool gotoPosGPS(bool block, double lat, double lon, float alt);
    bool gotoPosGPS(bool block, double lat, double lon, float alt, float yaw);

    void gotoVelEnu(float vel_x, float vel_y, float vel_z);
    void gotoVelEnu(float vel_x, float vel_y, float vel_z, float yaw);
    void gotoVelBody(float vel_x, float vel_y, float vel_z);
    void gotoVelBody(float vel_x, float vel_y, float vel_z, float yaw);

    void gotoVelxyPoszEnu(float vel_x, float vel_y, float pos_z);
    void gotoVelxyPoszEnu(float vel_x, float vel_y, float pos_z, float yaw);
    void gotoVelxyPoszBody(float vel_x, float vel_y, float pos_z);
    void gotoVelxyPoszBody(float vel_x, float vel_y, float pos_z, float yaw);

    bool changeMode(std::string mode);
    bool land();
    bool rtl();
    bool guided();

    bool unlock(bool danger);
    bool takeoff(float takeoff_height=1.5);
    easyrobot_msgs::RobotState getRobotState(bool print_enabled=false);

    float longitudeScale(double lat);
    float getDistance(const double lat_a, const double lon_a, const double lat_b, const double lon_b);
    bool detectionDistance(double x, double y, double z, double thresholds, double timeout);
    bool detectionGPSDistance(double lat, double lon, double alt, double thresholds, double timeout);
};

Easyrobot::Easyrobot(ros::NodeHandle &n, std::string ns)
{
    robot_state_sub = n.subscribe(ns + "/easyrobot/state", 10, &Easyrobot::robotStateCB, this);
    robot_control_pub = n.advertise<easyrobot_msgs::ControlCommand>(ns + "/easyrobot/command", 10);
    setlocale(LC_CTYPE, "zh_CN.utf8");
    sleep(1);
}

void Easyrobot::robotStateCB(const easyrobot_msgs::RobotStateConstPtr& msg)
{
    state = *msg;
}

bool Easyrobot::gotoPosEnu(bool block, float pos_x, float pos_y, float pos_z)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 1; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 4088; // 1111 1111 1000
    command.desired_position[0] = pos_x;
    command.desired_position[1] = pos_y;
    command.desired_position[2] = pos_z;

    robot_control_pub.publish(command);
    if(block)
    {
        return detectionDistance(pos_x, pos_y, pos_z, 0.5, 20);
    }
    
}

bool Easyrobot::gotoPosEnu(bool block, float pos_x, float pos_y, float pos_z, float yaw)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 1; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 3064; // 1011 1111 1000
    command.desired_position[0] = pos_x;
    command.desired_position[1] = pos_y;
    command.desired_position[2] = pos_z;
    command.desired_yaw = yaw;

    robot_control_pub.publish(command);
    if(block)
    {
        return detectionDistance(pos_x, pos_y, pos_z, 0.5, 20);
    }
}

bool Easyrobot::gotoPosBody(bool block, float pos_x, float pos_y, float pos_z)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 9; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 4088; // 1111 1111 1000
    command.desired_position[0] = pos_x;
    command.desired_position[1] = pos_y;
    command.desired_position[2] = pos_z;

    robot_control_pub.publish(command);
    if(block)
    {
        ros::spinOnce();
        double enu_pos_x, enu_pos_y, enu_pos_z;
        enu_pos_x = pos_x * cos(state.attitude.yaw) - pos_y * sin(state.attitude.yaw) + state.local_position.position.x;
        enu_pos_y = pos_x * sin(state.attitude.yaw) + pos_y * cos(state.attitude.yaw) + state.local_position.position.y;
        enu_pos_z = pos_z + state.local_position.position.z;
        return detectionDistance(enu_pos_x, enu_pos_y, enu_pos_z, 0.5, 20);
    }
}

bool Easyrobot::gotoPosBody(bool block, float pos_x, float pos_y, float pos_z, float yaw)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 9; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 3064; // 1011 1111 1000
    command.desired_position[0] = pos_x;
    command.desired_position[1] = pos_y;
    command.desired_position[2] = pos_z;
    command.desired_yaw = yaw;

    robot_control_pub.publish(command);
    if(block)
    {
        ros::spinOnce();
        double enu_pos_x, enu_pos_y, enu_pos_z;
        enu_pos_x = pos_x * cos(state.attitude.yaw) - pos_y * sin(state.attitude.yaw) + state.local_position.position.x;
        enu_pos_y = pos_x * sin(state.attitude.yaw) + pos_y * cos(state.attitude.yaw) + state.local_position.position.y;
        enu_pos_z = pos_z + state.local_position.position.z;
        return detectionDistance(enu_pos_x, enu_pos_y, enu_pos_z, 0.5, 20);
    }
}

bool Easyrobot::gotoPosGPS(bool block, double lat, double lon, float alt)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_GLOBAL_INT;
    command.coordinate_frame = 6; // FRAME_GLOBAL_INT = 5 FRAME_GLOBAL_REL_ALT = 6
                                  //  FRAME_GLOBAL_TERRAIN_ALT = 11
    command.type_mask = 4088; // 1111 1111 1000
    command.latitude = lat;
    command.longitude = lon;
    command.altitude = alt;

    robot_control_pub.publish(command);
    if(block)
    {
        return detectionGPSDistance(lat, lon, alt, 0.5, 20);
    }
}

bool Easyrobot::gotoPosGPS(bool block, double lat, double lon, float alt, float yaw)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_GLOBAL_INT;
    command.coordinate_frame = 6; // FRAME_GLOBAL_INT = 5 FRAME_GLOBAL_REL_ALT = 6
                                  //  FRAME_GLOBAL_TERRAIN_ALT = 11
    command.type_mask = 3064; // 1011 1111 1000
    command.latitude = lat;
    command.longitude = lon;
    command.altitude = alt;
    command.desired_yaw = yaw;

    robot_control_pub.publish(command);
    if(block)
    {
        return detectionGPSDistance(lat, lon, alt, 0.5, 20);
    }
}

void Easyrobot::gotoVelEnu(float vel_x, float vel_y, float vel_z)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 1; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 4039; // 1111 1100 0111
    command.desired_velocity[0] = vel_x;
    command.desired_velocity[1] = vel_y;
    command.desired_velocity[2] = vel_z;

    robot_control_pub.publish(command);
}

void Easyrobot::gotoVelEnu(float vel_x, float vel_y, float vel_z, float yaw)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 1; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 3015; // 1011 1100 0111
    command.desired_velocity[0] = vel_x;
    command.desired_velocity[1] = vel_y;
    command.desired_velocity[2] = vel_z;
    command.desired_yaw = yaw;

    robot_control_pub.publish(command);
}

void Easyrobot::gotoVelBody(float vel_x, float vel_y, float vel_z)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 9; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 4039; // 1111 1100 0111
    command.desired_velocity[0] = vel_x;
    command.desired_velocity[1] = vel_y;
    command.desired_velocity[2] = vel_z;

    robot_control_pub.publish(command);
}

void Easyrobot::gotoVelBody(float vel_x, float vel_y, float vel_z, float yaw)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 9; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 3015; // 1011 1100 0111
    command.desired_velocity[0] = vel_x;
    command.desired_velocity[1] = vel_y;
    command.desired_velocity[2] = vel_z;
    command.desired_yaw = yaw;

    robot_control_pub.publish(command);
}

void Easyrobot::gotoVelxyPoszEnu(float vel_x, float vel_y, float pos_z)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 1; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 4035; // 1111 1100 0011
    command.desired_velocity[0] = vel_x;
    command.desired_velocity[1] = vel_y;
    command.desired_position[2] = pos_z;

    robot_control_pub.publish(command);
}

void Easyrobot::gotoVelxyPoszEnu(float vel_x, float vel_y, float pos_z, float yaw)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 1; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 3011; // 1011 1100 0011
    command.desired_velocity[0] = vel_x;
    command.desired_velocity[1] = vel_y;
    command.desired_position[2] = pos_z;
    command.desired_yaw = yaw;

    robot_control_pub.publish(command);
}

void Easyrobot::gotoVelxyPoszBody(float vel_x, float vel_y, float pos_z)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 9; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 4035; // 1111 1100 0011
    command.desired_velocity[0] = vel_x;
    command.desired_velocity[1] = vel_y;
    command.desired_position[2] = pos_z;

    robot_control_pub.publish(command);
}

void Easyrobot::gotoVelxyPoszBody(float vel_x, float vel_y, float pos_z, float yaw)
{
    command.command_type = easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED;
    command.coordinate_frame = 9; // FRAME_LOCAL_NED = 1 FRAME_LOCAL_OFFSET_NED = 7
                                  // FRAME_BODY_NED = 8 FRAME_BODY_OFFSET_NED = 9
    command.type_mask = 3011; // 1011 1100 0011
    command.desired_velocity[0] = vel_x;
    command.desired_velocity[1] = vel_y;
    command.desired_position[2] = pos_z;
    command.desired_yaw = yaw;

    robot_control_pub.publish(command);
}

bool Easyrobot::changeMode(std::string mode)
{
    if(mode == "AUTO")
    {
        command.command_type = easyrobot_msgs::ControlCommand::AUTO;
    }
    else if(mode == "GUIDED")
    {
        command.command_type = easyrobot_msgs::ControlCommand::GUIDED;
    }
    else if(mode == "LOITER")
    {
        command.command_type = easyrobot_msgs::ControlCommand::LOITER;
    }
    else if(mode == "RTL")
    {
        command.command_type = easyrobot_msgs::ControlCommand::RTL;
    }
    else if(mode == "CIRCLE")
    {
        command.command_type = easyrobot_msgs::ControlCommand::CIRCLE;
    }
    else if(mode == "LAND")
    {
        command.command_type = easyrobot_msgs::ControlCommand::LAND;
    }
    else if(mode == "POSHOLD")
    {
        command.command_type = easyrobot_msgs::ControlCommand::POSHOLD;
    }
    else if(mode == "BRAKE")
    {
        command.command_type = easyrobot_msgs::ControlCommand::BRAKE;
    }
    else if(mode == "FOLLOW")
    {
        command.command_type = easyrobot_msgs::ControlCommand::FOLLOW;
    }
    else
    {
        ROS_WARN("[%s] mode is not supported", mode.c_str());
        return false;
    }
    robot_control_pub.publish(command);
    ros::Time t1 = ros::Time::now();
    while(1)
    {
        ros::spinOnce();
        if(state.mode == mode)
        {
            return true;
        }
        if((ros::Time::now() - t1).toSec() >= 3)
        {
            return false;
        }
        usleep(100000);
    }
}

bool Easyrobot::land()
{
    return changeMode("LAND");
}

bool Easyrobot::rtl()
{
    return changeMode("RTL");
}

bool Easyrobot::guided()
{
    return changeMode("GUIDED");
}

bool Easyrobot::unlock(bool danger)
{
    if(danger)
    {
        command.command_type = easyrobot_msgs::ControlCommand::ARMED;
    }
    else
    {
        command.command_type = easyrobot_msgs::ControlCommand::DISARMED;
    }
    robot_control_pub.publish(command);

    ros::Time t1 = ros::Time::now();
    ros::spinOnce();
    sleep(2);
    if(state.armed == danger)
    {
        return true;
    }
    else
    {
        return false;
    }
    

}

bool Easyrobot::takeoff(float takeoff_height)
{
    std::cout << "takeoff" << std::endl;
    command.command_type = easyrobot_msgs::ControlCommand::TAKEOFF;
    command.takeoff_height = takeoff_height;
    robot_control_pub.publish(command);
    ros::Time t1 = ros::Time::now();
    while(1)
    {
        if(state.armed && (state.mode == "TAKEOFF"))
        {
            sleep(3);
            ROS_INFO("takeoff ok");
            if(state.local_position.position.z - takeoff_height < 0.3)
            {
                return true;
            }
        }
        if((ros::Time::now() - t1).toSec() >= 10)
        {
            return false;
        }
    }
    
}

easyrobot_msgs::RobotState Easyrobot::getRobotState(bool print_enabled)
{
    //调用回调函数,更新机器人状态
    ros::spinOnce();
    if(print_enabled)
    {
        //打印机器人类型信息
        ROS_INFO("robot : [%s]", state.type.c_str());
        //打印机器人连接情况
        if(state.connected)
        {
            ROS_INFO("[%s] connected", state.type.c_str());
        }
        else
        {
            ROS_ERROR("[%s] unconnected", state.type.c_str());
        }
        //打印机器人解锁情况
        if(state.armed)
        {
            ROS_INFO("[%s] armed", state.type.c_str());
        }
        else
        {
            ROS_INFO("[%s] disarmed", state.type.c_str());
        }
        //打印机器人控制模式
        ROS_INFO("fcu mode : [%s]", state.mode.c_str());
        //GPS传感器状态
        ROS_INFO("GPS status : [%s]", state.gps_status.c_str());
        //全局坐标系(WGS84)位置数据
        ROS_INFO("Global Position : latitude:[%f] longitude:[%f] altitude:[%f]",state.latitude, state.longitude, state.altitude);
        //本地坐标系(ENU)位置数据
        ROS_INFO("Local Positoon : X:[%f] Y:[%f] Z:[%f]", state.local_position.position.x, state.local_position.position.y, state.local_position.position.z);
        //速度数据
        ROS_INFO("Linear : X:[%f] Y:[%f] z:[%f]", state.linear.x, state.linear.y, state.linear.z);
        //姿态数据
        ROS_INFO("Attitude : roll:[%f] pitch:[%f] yaw:[%f]", state.attitude.roll, state.attitude.pitch, state.attitude.yaw);
        //电量数据
        ROS_INFO("Battry Voltage: [%f] V", state.battery_voltage);
        int percentage = state.battery_percentage * 100;
        ROS_INFO("Battry Percentage: %d%s",percentage,"%");
    }
    return state;
}

float Easyrobot::longitudeScale(double lat)
{
    float scale = cosf(lat * (3.1415926 / 180.0f));
    if (scale < 0.01f) {
        scale = 0.01f;
    } else if (scale > 1.0f) {
        scale = 1.0f;
    }

    return scale;
}

// return distance in meters between two locations
float Easyrobot::getDistance(const double lat_a, const double lon_a, const double lat_b, const double lon_b)
{
    float dlat              = (float)(lat_b * 1e7- lat_a * 1e7);
    float dlong             = ((float)(lon_b * 1e7- lon_a * 1e7)) * longitudeScale(lat_b);
    return std::sqrt(dlat*dlat + dlong*dlong) * LOCATION_SCALING_FACTOR;
}

bool Easyrobot::detectionDistance(double x, double y, double z, double thresholds, double timeout)
{
    Eigen::Vector3d current_pos;
    Eigen::Vector3d goal_pos;
    double distance;
    goal_pos[0] = x;
    goal_pos[1] = y;
    goal_pos[2] = z;
    ros::Time t1 = ros::Time::now();
    while(1)
    {
        ros::spinOnce();
        current_pos[0] = state.local_position.position.x;
        current_pos[1] = state.local_position.position.y;
        current_pos[2] = state.local_position.position.z;
        distance = (goal_pos - current_pos).norm();

        if(distance <= thresholds)
        {
            ROS_INFO("到达目标点");
            return true;
        }
        if((ros::Time::now() - t1).toSec() >= timeout)
        {
            ROS_WARN("超时,未能到达目标点");
            return false;
        }
        usleep(100000);
    }
}

bool Easyrobot::detectionGPSDistance(double lat, double lon, double alt, double thresholds, double timeout)
{
    double horizon_distance, height_distance,distance;
    ros::Time t1 = ros::Time::now();
    while(1)
    {
        ros::spinOnce();
        horizon_distance = getDistance(lat, lon, state.latitude, state.longitude);
        height_distance = alt - state.local_position.position.z;
        distance = sqrt(pow(horizon_distance,2) + pow(height_distance,2));
        if(distance <= thresholds)
        {
            ROS_INFO("到达目标点");
            return true;
        }
        if((ros::Time::now() - t1).toSec() >= timeout)
        {
            ROS_WARN("超时,未能到达目标点");
            return false;
        }
        usleep(100000);
    }
}


#endif

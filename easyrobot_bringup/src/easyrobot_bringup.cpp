// ROS API接口头文件
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
//mavros_msgs 消息引用头文件
#include "mavros_msgs/State.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/BatteryStatus.h"
#include "mavros_msgs/GPSRAW.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include <mavros_msgs/CommandTOL.h>
#include "mavros_msgs/GPSRAW.h"
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/GlobalPositionTarget.h"
//geometry_msgs 消息引用头文件
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
//sensor_msgs 消息引用头文件
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/BatteryState.h"
//geographic_msgs 消息引用头文件
#include "geographic_msgs/GeoPoseStamped.h"
//std_msgs 消息引用头文件
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
//easyrobot_msgs 消息引用头文件
#include <easyrobot_msgs/ControlCommand.h>
#include <easyrobot_msgs/RobotState.h>
#include <mavros_msgs/HomePosition.h>

//订阅者

//全局坐标系(WGS84)位置数据订阅者
ros::Subscriber global_position_sub;
//本地坐标系(END)位置数据订阅者
ros::Subscriber local_position_sub;
//本地坐标系(END)速度数据订阅者
ros::Subscriber local_velocity_sub;
//机器人状态数据订阅者订阅者
ros::Subscriber fcu_state_sub;
//电池状态数据订阅者订阅者
ros::Subscriber battery_state_sub;
//机器人控制命令数据订阅者
ros::Subscriber command_sub;
//GPS状态数据订阅者
ros::Subscriber gps_status_sub;
ros::Subscriber home_position_sub;

//发布者
//全局坐标系(WGS84)位置控制数据发布者
ros::Publisher global_position_control_pub;
//本地坐标系(ENU)位置控制发布者
ros::Publisher local_position_control_pub;

//外部定位数据发布者
ros::Publisher external_location_pub;
//机器人状态数据发布者
ros::Publisher robot_state_pub;

ros::Publisher setpoint_raw_pub;

//服务
//解锁/上锁
ros::ServiceClient robot_arming_client;
//飞行模式
ros::ServiceClient robot_setmode_client;

ros::ServiceClient robot_takeoff_client;

//消息
//全局坐标系(WGS84)位置数据
sensor_msgs::NavSatFix global_position;
//本地坐标系(END)位置数据
geometry_msgs::PoseStamped local_position;
//速度数据
geometry_msgs::TwistStamped local_velocity;

mavros_msgs::HomePosition home_position;

//机器人状态数据
mavros_msgs::State fcu_state;
//机器人姿态数据
mavros_msgs::AttitudeTarget robot_attitude;

mavros_msgs::PositionTarget local_position_cmd;
mavros_msgs::GlobalPositionTarget global_position_cmd;


//机器人电池数据
sensor_msgs::BatteryState battery;
//GPS状态数据
mavros_msgs::GPSRAW gps_status;
//机器人解上锁指令数据
mavros_msgs::CommandBool armed_enable;
mavros_msgs::CommandTOL takeoff_cmd;
//机器人飞行模式切换指令数据
mavros_msgs::SetMode mode;
//机器人控制指令
easyrobot_msgs::ControlCommand command;
//机器人状态
easyrobot_msgs::RobotState robot_state;

//参数
//主循环频率
int mainloop_frequency;
//命名空间使能
bool namespace_enabled;
//命名空间
std::string ns;
//外部定位使能
bool external_location;
//定位来源
int location_source;
//电池节数
int battery_quantity;
//单节电芯低电压(对应电量为零)
float single_battery_low_voltage;
//单节电芯高电压(对应电量为满电)
float single_battery_high_voltage;
//机器人类型
std::string robot_type;


//回调函数
void HomePositionCB(const mavros_msgs::HomePosition msg)
{
    home_position = msg;
}

void GlobalPositionCB(const sensor_msgs::NavSatFixConstPtr& msg)
{
    global_position = *msg;
}

void LocalPositionCB(const geometry_msgs::PoseStampedConstPtr& msg)
{
    local_position = *msg;
}

void LocalVelocityCB(const geometry_msgs::TwistStampedConstPtr& msg)
{
    local_velocity = *msg;
}

void FcuStateCB(const mavros_msgs::StateConstPtr& msg)
{
    fcu_state = *msg;
}

void BatteryStateCB(const sensor_msgs::BatteryStateConstPtr& msg)
{
    battery = *msg;
}

void CommandCB(const easyrobot_msgs::ControlCommandConstPtr& msg)
{
    command = *msg;
}

void GPSStatusCB(const mavros_msgs::GPSRAWConstPtr& msg)
{
    gps_status = *msg;
}

//函数

float calculateBatteryPercentage(float current_voltage)
{
    float full_voltage, empty_voltage, battery_percentage;
    full_voltage = battery_quantity * single_battery_high_voltage;
    empty_voltage = battery_quantity * single_battery_low_voltage;
    battery_percentage = (current_voltage - empty_voltage)/(full_voltage - empty_voltage);
    return battery_percentage;
}

//机器人状态更新函数
void updateRobotState()
{
    //机器人类型:UAV(无人机);UGV(无人车);USV(无人船)
    robot_state.type = robot_type;
    //机器人连接情况,变量为true时为已连接,为false时为未连接
    robot_state.connected = fcu_state.connected;
    //机器人解锁情况,变量为true时为已解锁,为false时为未解锁
    //未解锁时,一般操作无法控制机器人
    robot_state.armed = fcu_state.armed;
    //机器人当前所处模式
    robot_state.mode = fcu_state.mode;
    //机器人经纬度以及海拔高度数据(坐标系:WGS84)
    robot_state.latitude = global_position.latitude;
    robot_state.longitude = global_position.longitude;
    robot_state.altitude = global_position.altitude;
    //机器人本地位置数据(坐标系:END)
    //END坐标系以东方向为X轴正半轴,北方向为Y轴正半轴,向上为Z轴正半轴,单位为米
    //坐标系原点为机器人开机启动初始所在位置点
    robot_state.local_position.position.x = local_position.pose.position.x;
    robot_state.local_position.position.y = local_position.pose.position.y;
    robot_state.local_position.position.z = local_position.pose.position.z;
    robot_state.local_position.orientation = local_position.pose.orientation;
    //机器人速度数据)(坐标系:END)
    robot_state.linear.x = local_velocity.twist.linear.x;
    robot_state.linear.y = local_velocity.twist.linear.y;
    robot_state.linear.z = local_velocity.twist.linear.z;

    //电池电量数据
    robot_state.battery_voltage = battery.voltage;
    robot_state.battery_percentage = calculateBatteryPercentage(battery.voltage);

    //将四元数转化为姿态角
    tf::Quaternion quat;
    tf::quaternionMsgToTF(local_position.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    robot_state.attitude.roll = roll;
    robot_state.attitude.pitch = pitch;
    robot_state.attitude.yaw = yaw;

    robot_state.home_lat = home_position.geo.latitude;
    robot_state.home_lng = home_position.geo.longitude;
    robot_state.home_alt = home_position.geo.altitude;

    //GPS状态
    /*
    NO_GPS:系统没有检测到GPS模块
    NO_FIX:系统检测到GPS模块,但没有定位数据
    2D_FIX:GPS模块有定位数据,但是无法正常使用,常常出现于机器人刚启动时,系统未完全正常工作的情况
    3D_FIX:GPS模块定位数据正常
    DGPS:GPS模块接收到纠偏数据,但数据质量较差,相较于GPS有更好的定位精度,但提升较小
    RTK_FLOATR:GPS模块接收到纠偏数据,数据质量一般,定位精度大概在分米级
    RTK_FIXED:GPS模块接收到纠偏数据,数据质量较好,定位精度大概在厘米级
    */
    switch (gps_status.fix_type)
    {
    case mavros_msgs::GPSRAW::GPS_FIX_TYPE_NO_GPS:
        robot_state.gps_status = "NO_GPS";
        break;

    case mavros_msgs::GPSRAW::GPS_FIX_TYPE_NO_FIX:
        robot_state.gps_status = "NO_FIX";
        break;

    case mavros_msgs::GPSRAW::GPS_FIX_TYPE_2D_FIX:
        robot_state.gps_status = "2D_FIX";
        break;

    case mavros_msgs::GPSRAW::GPS_FIX_TYPE_3D_FIX:
        robot_state.gps_status = "3D_FIX";
        break;

    case mavros_msgs::GPSRAW::GPS_FIX_TYPE_DGPS:
        robot_state.gps_status = "DGPS";
        break;

    case mavros_msgs::GPSRAW::GPS_FIX_TYPE_RTK_FLOATR:
        robot_state.gps_status = "RTK_FLOATR";
        break;

    case mavros_msgs::GPSRAW::GPS_FIX_TYPE_RTK_FIXEDR:
        robot_state.gps_status = "RTK_FIXEDR";
        break;
    
    default:
        robot_state.gps_status = "UNKNOWN_STATUS";
        break;
    }
    robot_state_pub.publish(robot_state);
}

//指令响应函数
void executeCommand()
{
    switch (command.command_type)
    {
        case easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_LOCAL_NED:
            local_position_cmd.header.stamp = ros::Time::now();
            local_position_cmd.header.frame_id = "LOCAL_NED";
            local_position_cmd.coordinate_frame = command.coordinate_frame;
            local_position_cmd.type_mask = command.type_mask;
            local_position_cmd.position.x = command.desired_position[0];
            local_position_cmd.position.y = command.desired_position[1];
            local_position_cmd.position.z = command.desired_position[2];
            local_position_cmd.velocity.x = command.desired_velocity[0];
            local_position_cmd.velocity.y = command.desired_velocity[1];
            local_position_cmd.velocity.z = command.desired_velocity[2];
            local_position_cmd.yaw = command.desired_yaw;
            local_position_control_pub.publish(local_position_cmd);
            break;

        case easyrobot_msgs::ControlCommand::SET_POSITION_TARGET_GLOBAL_INT:
            global_position_cmd.header.stamp = ros::Time::now();
            global_position_cmd.header.frame_id = "GLOBAL_INT";
            global_position_cmd.coordinate_frame = command.coordinate_frame;
            global_position_cmd.type_mask = command.type_mask;
            global_position_cmd.latitude = command.latitude;
            global_position_cmd.longitude = command.longitude;
            global_position_cmd.altitude = command.altitude;
            global_position_cmd.velocity.x = command.desired_velocity[0];
            global_position_cmd.velocity.y = command.desired_velocity[1];
            global_position_cmd.velocity.z = command.desired_velocity[2];
            global_position_cmd.yaw = command.desired_yaw;
            global_position_control_pub.publish(global_position_cmd);
            break;

    case easyrobot_msgs::ControlCommand::TAKEOFF:
        if(robot_state.mode != "GUIDED")
        {
            ROS_INFO("UAV mode not [GUIDED], set [GUIDED] mode ...");
            mode.request.custom_mode = "GUIDED";
            if(robot_setmode_client.call(mode))
            {
                ROS_INFO("set [GUIDED] mode success");
            }
            else
            {
                ROS_WARN("set [GUIDED] mode failed");
            }
        }
        if(!robot_state.armed)
        {
            ROS_INFO("UAV not armed, arming...");
            armed_enable.request.value = true;
            if(robot_arming_client.call(armed_enable))
            {
                ROS_INFO("UAV armed success");
            }
            else
            {
                ROS_WARN("UAV armed failed");
            }
            
        }

        takeoff_cmd.request.altitude = command.takeoff_height;
        takeoff_cmd.request.latitude = 0;
        takeoff_cmd.request.longitude = 0;
        takeoff_cmd.request.min_pitch = 0;
        takeoff_cmd.request.yaw = 0;
        robot_takeoff_client.call(takeoff_cmd);
        break;

    case easyrobot_msgs::ControlCommand::AUTO:
        if(robot_state.mode == "AUTO")
        {
            break;
        }
        ROS_INFO("set [AUTO] mode ...");
        mode.request.custom_mode = "AUTO";
        if(robot_setmode_client.call(mode))
        {
            ROS_INFO("set [AUTO] mode success");
        }
        else
        {
            ROS_ERROR("set [AUTO] mode failed");
        }
        break;

    case easyrobot_msgs::ControlCommand::GUIDED:
        if(robot_state.mode == "GUIDED")
        {
            break;
        }
        ROS_INFO("set [GUIDED] mode ...");
        mode.request.custom_mode = "GUIDED";
        if(robot_setmode_client.call(mode))
        {
            ROS_INFO("set [GUIDED] mode success");
        }
        else
        {
            ROS_ERROR("set [GUIDED] mode failed");
        }
        break;

    case easyrobot_msgs::ControlCommand::LOITER:
        if(robot_state.mode == "LOITER")
        {
            break;
        }
        ROS_INFO("set [LOITER] mode ...");
        mode.request.custom_mode = "LOITER";
        if(robot_setmode_client.call(mode))
        {
            ROS_INFO("set [LOITER] mode success");
        }
        else
        {
            ROS_ERROR("set [LOITER] mode failed");
        }
        break;

   case easyrobot_msgs::ControlCommand::RTL:
        if(robot_state.mode == "RTL")
        {
            break;
        }
        ROS_INFO("set [RTL] mode ...");
        mode.request.custom_mode = "RTL";
        if(robot_setmode_client.call(mode))
        {
            ROS_INFO("set [RTL] mode success");
        }
        else
        {
            ROS_ERROR("set [RTL] mode failed");
        }
        break;

   case easyrobot_msgs::ControlCommand::CIRCLE:
        if(robot_state.mode == "CIRCLE")
        {
            break;
        }
        ROS_INFO("set [CIRCLE] mode ...");
        mode.request.custom_mode = "CIRCLE";
        if(robot_setmode_client.call(mode))
        {
            ROS_INFO("set [CIRCLE] mode success");
        }
        else
        {
            ROS_ERROR("set [CIRCLE] mode failed");
        }
        break;

   case easyrobot_msgs::ControlCommand::LAND:
        if(robot_state.mode == "LAND")
        {
            break;
        }
        ROS_INFO("set [LAND] mode ...");
        mode.request.custom_mode = "LAND";
        if(robot_setmode_client.call(mode))
        {
            ROS_INFO("set [LAND] mode success");
        }
        else
        {
            ROS_ERROR("set [LAND] mode failed");
        }
        break;

    case easyrobot_msgs::ControlCommand::POSHOLD:
        if(robot_state.mode == "POSHOLD")
        {
            break;
        }
        ROS_INFO("set [POSHOLD] mode ...");
        mode.request.custom_mode = "POSHOLD";
        if(robot_setmode_client.call(mode))
        {
            ROS_INFO("set [POSHOLD] mode success");
        }
        else
        {
            ROS_ERROR("set [POSHOLD] mode failed");
        }
        break;

    case easyrobot_msgs::ControlCommand::BRAKE:
        if(robot_state.mode == "BRAKE")
        {
            break;
        }
        ROS_INFO("set [BRAKE] mode ...");
        mode.request.custom_mode = "BRAKE";
        if(robot_setmode_client.call(mode))
        {
            ROS_INFO("set [BRAKE] mode success");
        }
        else
        {
            ROS_ERROR("set [BRAKE] mode failed");
        }
        break;

   case easyrobot_msgs::ControlCommand::FOLLOW:
        if(robot_state.mode == "FOLLOW")
        {
            break;
        }
        ROS_INFO("set [FOLLOW] mode ...");
        mode.request.custom_mode = "FOLLOW";
        if(robot_setmode_client.call(mode))
        {
            ROS_INFO("set [FOLLOW] mode success");
        }
        else
        {
            ROS_ERROR("set [FOLLOW] mode failed");
        }
        break;

    case easyrobot_msgs::ControlCommand::ARMED:
        if(robot_state.armed)
        {
            break;
        }
        ROS_INFO("UAV arming ...");
        armed_enable.request.value = true;
        if(robot_arming_client.call(armed_enable))
        {
            ROS_INFO("UAV armed");
        }
        else
        {
            ROS_ERROR("UAV can not armed");
        }
        break;

    case easyrobot_msgs::ControlCommand::DISARMED:
        if(!robot_state.armed)
        {
            break;
        }
        ROS_INFO("UAV disarming ...");
        armed_enable.request.value = false;
        if(robot_arming_client.call(armed_enable))
        {
            ROS_INFO("UAV disarmed");
        }
        else
        {
            ROS_ERROR("UAV can not disarmed");
        }
        break;
    
    case easyrobot_msgs::ControlCommand::NONE:
    default:
        //ROS_ERROR("Unknown control command, unable to execute");
        break;
    }
    command.command_type = easyrobot_msgs::ControlCommand::NONE;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "easyrobot_bringup");
    ros::NodeHandle n("~");

    n.param<int>("mainloop_frequency", mainloop_frequency, 30);
    n.param<bool>("external_location", external_location, false);
    n.param<int>("location_source", location_source, 0);
    n.param<int>("battery_quantity", battery_quantity, 4);
    n.param<float>("single_battery_low_voltage", single_battery_low_voltage, 3.7);
    n.param<float>("single_battery_high_voltage", single_battery_high_voltage, 4.2);
    n.param<bool>("namespace_enabled", namespace_enabled, false);
    n.param<std::string>("ns", ns, "");
    std::cout << ns << std::endl;
    n.param<std::string>("robot_type", robot_type, "uav");

    ros::Rate r(mainloop_frequency);

    if(!namespace_enabled)
    {
        ns = "";
    }

    home_position_sub = n.subscribe(ns + "/mavros/home_position/home",1,HomePositionCB);
    global_position_sub = n.subscribe(ns + "/mavros/global_position/global",1,GlobalPositionCB);
    local_position_sub = n.subscribe(ns + "/mavros/local_position/pose", 1, LocalPositionCB);
    local_velocity_sub = n.subscribe(ns + "/mavros/local_position/velocity_local", 1, LocalVelocityCB);
    gps_status_sub = n.subscribe(ns + "/mavros/gpsstatus/gps1/raw", 1, GPSStatusCB);
    fcu_state_sub = n.subscribe(ns + "/mavros/state", 1, FcuStateCB);
    battery_state_sub = n.subscribe(ns + "/mavros/battery", 1, BatteryStateCB);
    command_sub = n.subscribe(ns + "/easyrobot/command", 1, CommandCB);
    
    local_position_control_pub = n.advertise<mavros_msgs::PositionTarget>(ns + "/mavros/setpoint_raw/local", 10);
    global_position_control_pub = n.advertise<mavros_msgs::GlobalPositionTarget>(ns + "/mavros/setpoint_raw/global", 10);
    external_location_pub = n.advertise<geometry_msgs::PoseStamped>(ns + "/mavros/vision_pose/pose", 10);
    robot_state_pub = n.advertise<easyrobot_msgs::RobotState>(ns + "/easyrobot/state", 10);
    
    robot_arming_client = n.serviceClient<mavros_msgs::CommandBool>(ns + "/mavros/cmd/arming");
    robot_setmode_client = n.serviceClient<mavros_msgs::SetMode>(ns + "/mavros/set_mode");
    robot_takeoff_client = n.serviceClient<mavros_msgs::CommandTOL>(ns + "/mavros/cmd/takeoff");

    while(ros::ok())
    {
        //调用回调函数
        ros::spinOnce();
        //更新机器人状态
        updateRobotState();
        //执行控制命令
        executeCommand();
        r.sleep();
    }
}

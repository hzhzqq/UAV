#include <ros/ros.h>
// 线性代数，矩阵和矢量运算的cpp库
#include <Eigen/Eigen>
#include <std_msgs/Bool.h>
#include <mavros/frame_tf.h>
// 地理日志库 用于GPS，地心和笛卡尔坐标系的转换
#include <GeographicLib/Geocentric.hpp>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>

// 目标位置的半径
double around_radius;
// 仿真标志
bool sim;

// 实例化对象
// 无人机目标位置发布者
ros::Publisher ugv1_goal_pub;
ros::Publisher ugv2_goal_pub;
ros::Publisher ugv3_goal_pub;

// 局部坐标系
geometry_msgs::PoseStamped uav1_pose;
geometry_msgs::PoseStamped ugv1_pose;
geometry_msgs::PoseStamped ugv2_pose;
geometry_msgs::PoseStamped ugv3_pose;

// 无人车目标位置
geometry_msgs::PoseStamped ugv1_goal;
geometry_msgs::PoseStamped ugv2_goal;
geometry_msgs::PoseStamped ugv3_goal;

// gps
sensor_msgs::NavSatFix uav1_gps;
sensor_msgs::NavSatFix ugv1_gps;
sensor_msgs::NavSatFix ugv2_gps;
sensor_msgs::NavSatFix ugv3_gps;

// 得到以无人车坐标系为基座标系，无人机的位置
geometry_msgs::Point getOffsetLocalPose(sensor_msgs::NavSatFix origin_gps_data, sensor_msgs::NavSatFix current_gps_data)
{
    geometry_msgs::Point offset_pose;
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    // 无人机GPS信息
    Eigen::Vector3d origin_gps; 
    // 无人机GPS信息对应的ECEF坐标系下的坐标
    Eigen::Vector3d origin_ecef; 
    // 无人车GPS信息
    Eigen::Vector3d current_uav_gps; 
    // 无人车GPS信息对应的ECEF坐标系下的坐标
    Eigen::Vector3d current_uav_ecef; 
    // ECEF坐标系下的坐标偏移量
    Eigen::Vector3d ecef_offset; 
    // ENU（东-北-天）坐标系下的坐标偏移量
    Eigen::Vector3d enu_offset; 

    // 无人机GPS经度
    origin_gps[0] = origin_gps_data.latitude;
    // 无人机GPS纬度
    origin_gps[1] = origin_gps_data.longitude;
    // 无人机GPS高度
    origin_gps[2] = origin_gps_data.altitude;

    // 无人车GPS经度
    current_uav_gps[0] = current_gps_data.latitude;
    // 无人车GPS纬度
    current_uav_gps[1] = current_gps_data.longitude;
    // 无人车GPS高度
    current_uav_gps[2] = current_gps_data.altitude;

    // 将原始GPS信息转换为ECEF坐标系下的坐标
    earth.Forward(origin_gps[0], origin_gps[1], origin_gps[2], origin_ecef[0], origin_ecef[1], origin_ecef[2]);
    earth.Forward(current_uav_gps[0], current_uav_gps[1], current_uav_gps[2], current_uav_ecef[0], current_uav_ecef[1], current_uav_ecef[2]);

    // 得到ECEF坐标系下机车的位置距离
    ecef_offset = current_uav_ecef - origin_ecef;
    // 将其转化成东北天坐标系
    enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, origin_gps);

    offset_pose.x = enu_offset[0];
    offset_pose.y = enu_offset[1];
    return offset_pose;
}

// 回调函数
void uav1GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    uav1_gps = *msg;
}

void uav1PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    uav1_pose = *msg;
}

void ugv1GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    ugv1_gps = *msg;
}

void ugv2GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    ugv2_gps = *msg;
}

void ugv3GPSCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    ugv3_gps = *msg;
}

void ugv1PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ugv1_pose = *msg;
}

void ugv2PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ugv2_pose = *msg;
}

void ugv3PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ugv3_pose = *msg;
}

void searchCallback(const std_msgs::BoolConstPtr& msg)
{
    if(sim)
    {
        // 仿真
        // 无人车和无人机的坐标系是一致的
        ugv1_goal = uav1_pose;
        ugv1_goal.header.stamp = ros::Time::now();
        ugv1_goal.header.frame_id = "map";
        ugv1_goal.pose.position.x += around_radius;
        ugv1_goal.pose.position.z = 0;
        // 改变朝向
        // yaw转四元数
        ugv1_goal.pose.orientation = tf::createQuaternionMsgFromYaw(3.14);

        ugv2_goal = uav1_pose;
        ugv2_goal.header.stamp = ros::Time::now();
        ugv2_goal.header.frame_id = "map";
        ugv2_goal.pose.position.x -= around_radius;
        ugv2_goal.pose.position.y += around_radius;
        ugv2_goal.pose.position.z = 0;
        ugv2_goal.pose.orientation = tf::createQuaternionMsgFromYaw(-0.52);

        ugv3_goal = uav1_pose;
        ugv3_goal.header.stamp = ros::Time::now();
        ugv3_goal.header.frame_id = "map";
        ugv3_goal.pose.position.x -= around_radius;
        ugv3_goal.pose.position.y -= around_radius;
        ugv3_goal.pose.position.z = 0;
        ugv3_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.52);

        ugv1_goal_pub.publish(ugv1_goal);
        ugv2_goal_pub.publish(ugv2_goal);
        ugv3_goal_pub.publish(ugv3_goal);
    }
    else
    {
        //真机
        geometry_msgs::Point ugv1_offset_pose = getOffsetLocalPose(ugv1_gps, uav1_gps);
        ugv1_goal.header.stamp = ros::Time::now();
        ugv1_goal.header.frame_id = "map";
        // 无人车的目标位置在它局部坐标系里是 当前位置+车和机的距离（机的位置就是目标的位置）+目标物半径范围
        ugv1_goal.pose.position.x = ugv1_offset_pose.x + ugv1_pose.pose.position.x + around_radius;
        ugv1_goal.pose.position.y = ugv1_offset_pose.y + ugv1_pose.pose.position.y;
        // 改变朝向
        // yaw（绕z轴旋转180°）转化为四元数
        ugv1_goal.pose.orientation = tf::createQuaternionMsgFromYaw(3.14);

        geometry_msgs::Point ugv2_offset_pose = getOffsetLocalPose(ugv2_gps, uav1_gps);
        ugv1_goal.header.stamp = ros::Time::now();
        ugv1_goal.header.frame_id = "map";
        ugv2_goal.pose.position.x = ugv2_offset_pose.x + ugv2_pose.pose.position.x - around_radius;
        ugv2_goal.pose.position.y = ugv2_offset_pose.y + ugv2_pose.pose.position.y + around_radius;
        ugv2_goal.pose.orientation = tf::createQuaternionMsgFromYaw(-0.52);

        geometry_msgs::Point ugv3_offset_pose = getOffsetLocalPose(ugv3_gps, uav1_gps);
        ugv1_goal.header.stamp = ros::Time::now();
        ugv1_goal.header.frame_id = "map";
        ugv3_goal.pose.position.x = ugv3_offset_pose.x + ugv3_pose.pose.position.x - around_radius;
        ugv3_goal.pose.position.y = ugv3_offset_pose.y + ugv3_pose.pose.position.y - around_radius;
        ugv3_goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.52);
        // 发布目标位置给无人车
        ugv1_goal_pub.publish(ugv1_goal);
        ugv2_goal_pub.publish(ugv2_goal);
        ugv3_goal_pub.publish(ugv3_goal);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "distribute_goal");
    ros::NodeHandle n;

    //从配置文件中获取参数
    ros::param::get("~around_radius",around_radius);
    ros::param::get("~sim", sim);

    // 订阅者：target_search程序里发布的话题/uav1/search/result
    ros::Subscriber search_sub = n.subscribe("/uav1/search/result", 1, searchCallback);
    // 订阅者：无人机的GPS信息
    ros::Subscriber uav1_gps_sub = n.subscribe("/uav1/mavros/global_position/global", 1, uav1GPSCallback);
    // 订阅者：无人机的本地位置信息
    ros::Subscriber uav1_pose_sub = n.subscribe("/uav1/mavros/local_position/pose", 1, uav1PoseCallback);
    // 订阅者：无人车的本地位置信息
    ros::Subscriber ugv1_pose_sub = n.subscribe("/ugv1/mavros/local_position/pose", 1, ugv1PoseCallback);
    ros::Subscriber ugv2_pose_sub = n.subscribe("/ugv2/mavros/local_position/pose", 1, ugv2PoseCallback);
    ros::Subscriber ugv3_pose_sub = n.subscribe("/ugv3/mavros/local_position/pose", 1, ugv3PoseCallback);
    // 订阅者：无人车的GPS信息
    ros::Subscriber ugv1_gps = n.subscribe("/ugv1/mavros/global_position/global", 1, ugv1GPSCallback);
    ros::Subscriber ugv2_gps = n.subscribe("/ugv2/mavros/global_position/global", 1, ugv2GPSCallback);
    ros::Subscriber ugv3_gps = n.subscribe("/ugv3/mavros/global_position/global", 1, ugv3GPSCallback);
    // 发布者：目标位置给无人车
    ugv1_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/ugv1/move_base_simple/goal", 10);
    ugv2_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/ugv2/move_base_simple/goal", 10);
    ugv3_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/ugv3/move_base_simple/goal", 10);
    ros::spin();
}

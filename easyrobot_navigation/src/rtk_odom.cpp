// ROS头文件
#include "ros/ros.h"  
// 变换广播器头文件
#include "tf/transform_broadcaster.h"  
// 里程计消息头文件
#include "nav_msgs/Odometry.h"  
// 变换消息头文件
#include "tf/tf.h"  
// 位姿消息头文件
#include "geometry_msgs/PoseStamped.h"  

// 里程计消息对象
nav_msgs::Odometry odom_msgs; 
// 里程计消息发布器对象
ros::Publisher odom_pub;  
// 变换消息对象
geometry_msgs::TransformStamped odom_trans;  

// 里程计位置协方差矩形
boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0, 
    0, 1e-3, 1e-9, 0, 0, 0,  
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0, 
    0, 0, 0, 0, 1e6, 0, 
    0, 0, 0, 0, 0, 1e-9}};  

void odom_publish()
{
    // 设置里程计消息时间戳为当前时间
    odom_msgs.header.stamp = ros::Time::now(); 
    // 设置里程计消息的参考坐标系
    odom_msgs.header.frame_id = "odom";  
    // 设置里程计消息的子坐标系
    odom_msgs.child_frame_id = "base_link";  
    // 设置里程计消息的位置协方差
    odom_msgs.pose.covariance = odom_pose_covariance;  

    // 设置变换消息时间戳为当前时间
    odom_trans.header.stamp = ros::Time::now();  
    // 设置变换消息的参考坐标系
    odom_trans.header.frame_id = "odom"; 
    // 设置变换消息的子坐标系
    odom_trans.child_frame_id = "base_link";  
    // 设置变换消息的平移部分的z坐标为0
    odom_trans.transform.translation.z = 0.0;  

}

void posCallback(const geometry_msgs::PoseStampedConstPtr & pos_msgs)
{
    // 将接收到的位姿消息赋值给里程计消息的位置部分
    odom_msgs.pose.pose = pos_msgs->pose;  
    // 将位姿消息的x坐标赋值给变换消息的平移部分的x坐标
    odom_trans.transform.translation.x = pos_msgs->pose.position.x;  
    // 将位姿消息的y坐标赋值给变换消息的平移部分的y坐标
    odom_trans.transform.translation.y = pos_msgs->pose.position.y;  
    // 将位姿消息的姿态信息赋值给变换消息的旋转部分
    odom_trans.transform.rotation = pos_msgs->pose.orientation;  
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "rtk_odom");  
    // 创建ROS节点句柄
    ros::NodeHandle n;  
    // 创建变换广播器对象
    tf::TransformBroadcaster odom_br= n; 
    // 订阅位姿消息话题，并指定回调函数
    ros::Subscriber pos_sub = n.subscribe("/mavros/local_position/pose", 1, posCallback); 
    // 创建里程计消息发布器对象
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom",1); 
    // 设置循环频率为100Hz
    ros::Rate r(100); 
    // 设置变换消息的旋转部分的z轴分量
    odom_trans.transform.rotation.z = -0.001; 
    // 设置变换消息的旋转部分的w轴分量
    odom_trans.transform.rotation.w = 0.999; 

    while(ros::ok())
    {
        // 处理回调函数
        ros::spinOnce();  
        // 发布里程计和变换消息
        odom_publish();  
        // 发布里程计消息
        odom_pub.publish(odom_msgs);  
        // 发布变换消息
        odom_br.sendTransform(odom_trans);  
        // 控制循环频率
        r.sleep();  
    }

    return 0;

}


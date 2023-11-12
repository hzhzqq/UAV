// ROS API接口头文件
#include <ros/ros.h>
//std_msgs 消息引用头文件
#include <std_msgs/Bool.h>
//mavros_msgs 消息引用头文件
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
//geographic_msgs 消息引用头文件
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
//apriltag_ros 消息引用头文件
#include <apriltag_ros/AprilTagDetectionArray.h>
//标准库math 引用头文件
#include <math.h>
//标准库ector 引用头文件
#include <vector>
//三角函数的库
#include <cmath>
#include <iomanip>
#include <tf/transform_datatypes.h>

//宏定义 误差
#define OFFSET 0.3f
//宏定义 派
#define M_pi 3.1415926535

//地图总长
int map_length;
//地图总宽
int map_width;
//相机水平视场角
float DFOV;
//飞行高度
int height;
//是否仿真
bool sim;

//飞机状态
mavros_msgs::State current_state;
//找到目标的标志
bool search_flag = false;
//无人机当前状态的回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
 
 //目标位置
geometry_msgs::Point goal_pose;
//apritag码的位置信息回调函数
void aruco_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    //
    if(msg->detections.size() >= 1)
    {
        //只获取position信息
        goal_pose=msg->detections[0].pose.pose.pose.position;
    }

}

//无人机当前位置的回调函数
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
current_pose=*msg;
}



// PID调节
double value_i = 0.0;
double dt = 0.1;
double PID(double value_target, double value_now, double value_last)
{
    double kp = 0.2;
    double ki = 0;
    double kd = 0.2;

    double value_p = value_target - value_now;
    value_i += (value_target - value_now)*dt;
    double value_d = (value_p-value_last)/dt;

    double control_value = kp*value_p + ki*value_i + kd*value_d;
    return control_value;
}


//给划定的map分块
geometry_msgs::Pose** AbstractMapCreat(int row, int col, float length,float width,int height,int maplength,int mapwidth)
{
    //定义一个二维数组 并为他分配内存
    geometry_msgs::Pose** map = new geometry_msgs::Pose* [row];
    for(int i = 0; i < row; i++){
        map[i] = new geometry_msgs::Pose [col];
    }
    ROS_INFO("ROW = %d col = %d", row, col);
    // 变量定义
    // 飞机在东北天坐标系下头朝北时，正前方一块区域的分块，每块中心的坐标
    double x,y;
    // (x,y)相对原点的距离
    double r;
    // (x,y)与原点的连线 线相对于x轴的角度
    double a;
    // 飞机偏航角
    // 根据四元数计算的到
    double yaw = tf::getYaw(current_pose.pose.orientation);
    ROS_INFO("yaw=%f=%f",yaw,yaw*180/M_pi);
    //给每一块map放入位置数据
    // 按照(1,1),(1,2)...(2,1)...的顺序划分
    for(int i = 0 ; i < row ; i++) 
    {
        for(int j = 0; j < col; j++)
        {
            if (i == row-1)
            { 
                // 为了不损失地图。顶行最后一个特殊处理
                if (j == col-1)
                {
                    x = mapwidth;
                    y = maplength/2;
                    r = sqrt(pow(x,2)+pow(y,2));
                    a = atan(y/x);
                    map[i][j].position.x = r*cos(yaw+a) + current_pose.pose.position.x;
                    map[i][j].position.y = r*sin(yaw+a) + current_pose.pose.position.y;
                    map[i][j].position.z = height;
                    ROS_INFO("abstract_map i=%d j=%d x = %f y = %f Z = %f",
                     i, j, map[i][j].position.x, map[i][j].position.y,map[i][j].position.z);

                }
                // 为了不损失地图。顶行特殊处理
                else
                {
                    x = mapwidth;
                    y = length/2 + j*length -(maplength/2);
                    r = sqrt(pow(x,2)+pow(y,2));
                    a = atan(y/x);
                    map[i][j].position.x = r*cos(yaw+a)+ current_pose.pose.position.x;
                    map[i][j].position.y = r*sin(yaw+a)+ current_pose.pose.position.y;
                    map[i][j].position.z = height;
                    ROS_INFO("abstract_map i=%d j=%d x = %f y = %f Z = %f",
                     i, j, map[i][j].position.x, map[i][j].position.y,map[i][j].position.z);

                } 
            }
            else
            {
                // 为了不损失地图，每行最后一个特殊处理
                if (j == col-1)
                {
                    x = width/2 + i*width;
                    y = maplength/2;
                    r = sqrt(pow(x,2)+pow(y,2));
                    a = atan(y/x);
                    map[i][j].position.x = r*cos(yaw+a)+ current_pose.pose.position.x;
                    map[i][j].position.y = r*sin(yaw+a)+ current_pose.pose.position.y;
                    map[i][j].position.z = height;
                    ROS_INFO("abstract_map i=%d j=%d x = %f y = %f Z = %f",
                     i, j, map[i][j].position.x, map[i][j].position.y,map[i][j].position.z);
                }
                else
                {
                    x = width/2 + i*width;
                    y = length/2 + j*length-(maplength/2);
                    r = sqrt(pow(x,2)+pow(y,2));
                    a = atan(y/x);
                    map[i][j].position.x = r*cos(yaw+a)+ current_pose.pose.position.x;
                    map[i][j].position.y = r*sin(yaw+a)+ current_pose.pose.position.y;
                    map[i][j].position.z = height;
                    ROS_INFO("abstract_map i=%d j=%d x = %f y = %f Z = %f",
                     i, j, map[i][j].position.x, map[i][j].position.y,map[i][j].position.z);
                }
            }
            

        }
    }
    return map;
}


//创建一个队列，将上面map的各个点按照“S”形压入队列
void PubPoseArrayCreat(geometry_msgs::Pose** map, geometry_msgs::PoseArray& posearr, int row, int col ,int height)
{
    int nums = 0;
    bool reverse_flag = 0;
    geometry_msgs::Pose home;
    home.position.x=current_pose.pose.position.x;
    home.position.y=current_pose.pose.position.y;
    home.position.z = height;
    posearr.poses.push_back(home);
    // 每行排完flag变化，做到首尾相接“S”形排序
    for(int i = 0;i < row;i++)
    {
        if(!reverse_flag){
            for(int j = 0;j < col;j++)
		    {
                nums++;
			    posearr.poses.push_back(map[i][j]);
                if (map[i][j].position.x == map[i][j-1].position.x && map[i][j].position.y == map[i][j-1].position.y)
                {
                    posearr.poses.pop_back();
                }
                
		    }
            reverse_flag = 1;
        }
        else{
            for(int j= col - 1;j >= 0;j--)
		    {
			    posearr.poses.push_back(map[i][j]);
                if (map[i][j].position.x == map[i][j+1].position.x && map[i][j].position.y == map[i][j+1].position.y)
                {
                    posearr.poses.pop_back();
                }
                nums++;
		    }
            reverse_flag = 0;
        }
	}
}


int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "offb_node");
    //创建句柄
    ros::NodeHandle nh;
 
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    //订阅话题：
    //    无人机状态
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    //    apriltag码位置
    ros::Subscriber apriltag_pose_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections",1,aruco_cb);
    //    无人机位置
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",1,pose_cb);

    //发布话题：
    //    位置发布
    ros::Publisher pose_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);     
    //    到达码正上方后 发布一个信号给无人车                         
    ros::Publisher search_pub = nh.advertise<std_msgs::Bool>("/search/result",10);

    //以20HZ的频率刷新
    ros::Rate rate(20.0);
    //确保与飞控连接
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    //参数
    //位置信息的二维指针
    geometry_msgs::Pose** abstract_map;
    //参数向量
    std::vector<int> args;
    //位置数组
    geometry_msgs::PoseArray PoseArray;
    
    //获取roslaunch五个参数：地图长，地图宽，相机DFOV，飞行高度和是否仿真。
    ros::param::get ("~map_length",map_length);
    ros::param::get ("~map_width",map_width);
    ros::param::get ("~DFOV",DFOV);
    ros::param::get ("~height",height);
    ros::param::get ("~sim",sim);
    ROS_INFO("length=%d, width=%d, DFOV=%f, height=%d, sim=%d",map_length,map_width,DFOV,height,sim);
    //计算每块的长宽    
    float block_length = 2*(height*tan((DFOV/2)/(180.0)*M_pi));
    round(block_length);
    float block_width = block_length*(3.0/4.0);
    round(block_width);
    ROS_INFO("block_length = %f ;block_width = %f",block_length,block_width);
    //分块，航点排序 hang lie
    int row = (map_width / block_width)+1;
    int col = (map_length / block_length)+1;
    ROS_INFO("row = %d col = %d", row, col);
    abstract_map = AbstractMapCreat(row, col, block_length ,block_width,height,map_length,map_width);  
    PubPoseArrayCreat(abstract_map, PoseArray, row, col ,height);
    int nums = 0 ;

    //起始点
    double dt=0.1;
    mavros_msgs::PositionTarget pose;
    pose.type_mask=0b111111111000;
    pose.position = PoseArray.poses.at(0).position;
    pose.coordinate_frame=1;

    // 仿真使用到的，定义消息类型
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
 
    //记录现在的时间点
    ros::Time last_request = ros::Time::now();
 
    //初始化pid相关的参数
    double target = 0;
    double valueX_now = 0;
    double valueY_now = 0;
    int count=0;

    int time = 0;

    //进入主循环
    while(ros::ok())
    {
        // 如果仿真，自动起飞
        // 先进入OFFBOARD模式，再进入怠速状态
        if(sim) 
        {
            if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        }

        //判断是否检测到码
        if(goal_pose.x != 0)
        {
            //在误差以内将视为到达码的正上方
            if(fabs(goal_pose.x)<0.5&&fabs(goal_pose.y)<0.5)
            {
                //mavros的控制话题
                pose.type_mask=0b111111111000;
                pose.position.x=current_pose.pose.position.x;
                pose.position.y=current_pose.pose.position.y;
                pose.position.z=current_pose.pose.position.z;
            
                //给车发信号说找到了
                if(!search_flag)
                {
                    std_msgs::Bool search;
                    search.data = true;
                    search_pub.publish(search);
                    search_flag = true;
                }
                //锁点
                while(ros::ok())
                {
                    ROS_INFO("ok");
                    pose_pub.publish(pose);
                } 
            //没有到达正上方就要调节
            }else
            {
                //用速度控制 pd调节
                if (time < 10)
                {
                    pose.type_mask=0b101111100011;
                    pose.yaw=0;
                    pose.velocity.x = 0;
                    pose.velocity.y = 0;
                    time ++;
                }else
                {
                    pose.type_mask=0b101111100011;
                    pose.yaw=0;
                    //给IPD调节的值X等于副的目标的Y
                    double valueX = -goal_pose.y;
                    //给IPD调节的值Y等于副的目标的X
                    double valueY = -goal_pose.x;
                    //偏差X
                    double dvalueX = PID(target, valueX, valueX_now);
                    //现在的值X
                    valueX_now = valueX;
                    //更新值X
                    valueX = valueX + dvalueX*dt;
                    pose.velocity.x = valueX*10;
                    //限制速度x的大小
                    if(pose.velocity.x>0.2) pose.velocity.x=0.2;
                    if(pose.velocity.x<-0.2) pose.velocity.x=-0.2;

                    double dvalueY = PID(target, valueY, valueY_now);
                    valueY_now = valueY;
                    valueY = valueY + dvalueY*dt;
                    pose.velocity.y = valueY*10;
                    if(pose.velocity.y>0.2) pose.velocity.y=0.2;
                    if(pose.velocity.y<-0.2) pose.velocity.y=-0.2;
                    
                    ROS_INFO("cam pose x=%f,y=%f",goal_pose.x,goal_pose.y);
                    ROS_INFO("Adjusting Vx=%f,Vy=%f",pose.velocity.x,pose.velocity.y);
                    ROS_INFO("uav pose x=%f y=%f z=%f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
                    /*pose.type_mask=0b111111111000;
                    pose.position.x=-goal_pose.y+current_pose.pose.position.x;
                    pose.position.y=-goal_pose.x+current_pose.pose.position.y;
                    pose.position.z=current_pose.pose.position.z;
                    ROS_INFO("cam pose x=%f,y=%f",goal_pose.x,goal_pose.y);
                    ROS_INFO("uav pose x=%f y=%f z=%f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);*/
                }


                
            }
        //如果没看到码就按队列跑    
        }else{
            
            if((abs(current_pose.pose.position.x - PoseArray.poses.at(nums).position.x) < OFFSET) &&
            (abs(current_pose.pose.position.y - PoseArray.poses.at(nums).position.y) < OFFSET) &&
            (abs(current_pose.pose.position.z - PoseArray.poses.at(nums).position.z) < OFFSET) && 
            nums < PoseArray.poses.size())
            {
                pose.type_mask=0b111111111000;
                nums++;
                //防止跑出界了
                if(nums >= PoseArray.poses.size()) 
                {
                    nums--;
                }
                pose.position.x = PoseArray.poses.at(nums).position.x;
                pose.position.y = PoseArray.poses.at(nums).position.y;
                pose.position.z = PoseArray.poses.at(nums).position.z;
                ROS_INFO("pose x = %f , pose y = %f ,pose z = %f ,", pose.position.x, pose.position.y,pose.position.z);
                //每到一个点停留5s以确保该区域不会因为飞机速度过快而没看到码
                ros::Time last_request = ros::Time::now();
                while(ros::Time::now() - last_request < ros::Duration(5.0))
                {
            
                    pose_pub.publish(pose);
                    ros::spinOnce();
                    rate.sleep();
                }
            }
        
        }
        //发布点位
        pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
        
    }
    return 0;
}

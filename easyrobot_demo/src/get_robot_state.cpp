#include <easyrobot.hpp>

int main(int argc, char** argv)
{
    //初始化
    ros::init(argc, argv, "get_robot_state");
    ros::NodeHandle n;
    Easyrobot uav(n,"");
    //获取无人机状态
    easyrobot_msgs::RobotState uav_state;
    uav_state = uav.getRobotState(true);
    
    return 0;
}
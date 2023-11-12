#include <easyrobot.hpp>

int main(int argc, char** argv)
{
    //初始化
    ros::init(argc, argv, "changemode");
    ros::NodeHandle n;
    Easyrobot uav(n,"");
    //切换无人机模式为'POSEHOLD'
    uav.changeMode("POSHOLD");
    return 0;
}
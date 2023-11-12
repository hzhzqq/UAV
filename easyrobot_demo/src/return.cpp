#include <easyrobot.hpp>

int main(int argc, char** argv)
{
    //初始化
    ros::init(argc, argv, "return");
    ros::NodeHandle n;
    Easyrobot uav(n,"");
    //切换为无人机模式为'RTL'
    uav.rtl();
    // 切换无人机模式为'LAND'
    // uav.land();
    // 切换无人机模式为'GUIDED'
    // uav.guided()''
    return 0;
}
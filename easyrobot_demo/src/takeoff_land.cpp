#include <easyrobot.hpp>

int main(int argc, char** argv)
{
    //初始化
    ros::init(argc, argv, "takeoff_land");
    ros::NodeHandle n;
    Easyrobot uav(n,"");
    //无人机自动起飞
    uav.takeoff(1);
    //等待10秒
    sleep(10);
    //无人机降落
    uav.land();
    return 0;
}

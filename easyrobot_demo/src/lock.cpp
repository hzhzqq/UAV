#include <easyrobot.hpp>

int main(int argc, char** argv)
{
    //初始化
    ros::init(argc, argv, "lock");
    ros::NodeHandle n;
    Easyrobot uav(n,"");
    //发送无人机解锁指令
    uav.unlock(true);
    //等待3秒
    sleep(3);
    //发送无人机上锁指令
    uav.unlock(false);
    return 0;
}
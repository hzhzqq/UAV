#r300_navigation
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
#灵遨小车底盘ros控制程序
--tab -e 'bash -c "sleep 5;roslaunch lingao_bringup bringup.launch; exec bash"' \
#mavros apm版
--tab -e 'bash -c "sleep 6;roslaunch mavros apm.launch; exec bash"' \
#mavros小车位置消息格式 转 里程计消息格式
--tab -e 'bash -c "sleep 7;roslaunch easyrobot_navigation rtk_odom.launch; exec bash"' \
#蓝海光电激光雷达
--tab -e 'bash -c "sleep 8;roslaunch bluesea2 amov-50C-3.launch; exec bash"' \
#navigation避障
--tab -e 'bash -c "sleep 9;roslaunch r200_navigation r200_navigation.launch; exec bash"' \



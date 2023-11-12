#apriltag码寻址脚本文件
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
#mavros的px4通讯
--tab -e 'bash -c "sleep 5; roslaunch mavros px4.launch; exec bash"' \
#启动摄像头
--tab -e 'bash -c "sleep 10; roslaunch usb_cam usb_cam-test.launch; exec bash"' \
#二维码搜寻程序
--tab -e 'bash -c "sleep 15; roslaunch easyrobot_coordination target_search.launch; exec bash"' \
#坐标系转换程序
--tab -e 'bash -c "sleep 20; roslaunch easyrobot_coordination distributegoal.launch; exec bash"' \


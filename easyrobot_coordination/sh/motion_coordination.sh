#r300_simulation_navigation
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3;roslaunch easyrobot_simulation coordination_1uav.launch; exec bash"' \
--tab -e 'bash -c "sleep 7;roslaunch easyrobot_simulation coordination_3ugv.launch; exec bash"' \
--tab -e 'bash -c "sleep 9;roslaunch easyrobot_navigation 3ugv_navigation.launch; exec bash"' \
--tab -e 'bash -c "sleep 10;roslaunch easyrobot_coordination distributegoal.launch; exec bash"' \
--tab -e 'bash -c "sleep 11;roslaunch easyrobot_coordination target_search.launch; exec bash"' \
--tab -e 'bash -c "sleep 12;rosrun rviz rviz; exec bash"' \
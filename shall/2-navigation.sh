###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/8C8PE4/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/8C8PE4/devel/setup.bash; roslaunch robot_slam navigation_shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/8C8PE4/devel/setup.bash; roslaunch robot_slam view_nav.launch; exec bash"' \

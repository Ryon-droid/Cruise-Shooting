###gmapping with abot###
chmod +x /home/abot/8C8PE4/src/robot_slam/scripts/shoot_2025_new.py /home/abot/8C8PE4/src/robot_slam/scripts/2026_shoot_demo.py /home/abot/8C8PE4/src/robot_slam/scripts/2026_shoot_target.py /home/abot/8C8PE4/src/TTS_audio/scripts/TTS.py
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/8C8PE4/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/8C8PE4/devel/setup.bash; roslaunch abot_bringup shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/8C8PE4/devel/setup.bash; roslaunch robot_slam navigation_shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/8C8PE4/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/8C8PE4/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/8C8PE4/devel/setup.bash; roslaunch find_object_2d find_object_2d_shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/8C8PE4/devel/setup.bash; rosrun TTS_audio TTS.py; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/8C8PE4/devel/setup.bash; rosrun robot_slam 2026_shoot_target.py; exec bash"' \
--tab -e 'bash -c "source ~/8C8PE4/devel/setup.bash; cd /home/abot/8C8PE4/src/robot_slam/scripts/; /home/abot/anaconda3/envs/py39/bin/python 2026_shoot_demo.py; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/8C8PE4/devel/setup.bash; roslaunch robot_slam multi_goal_shoot_2025.launch; exec bash"'

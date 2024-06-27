# CIRENS_robots

tf_relay_pkg: rebroadcast the 'base_link" tf from each '\robot{i}\tf to a common '\tf' topic as child frames of a 'world' root frame. 

followbotpkg: robot1 is designated as the leader.  Each i != 1, robot{i} moves in the direction of robot1 relative to its frame. 

analog_teleop.py:  GUI control of namespaced robot with analog dial

teleop.py: Gui control of a namespaced robot using arrow buttons

Steps to run followbot:

1. Build tf_relay_pkg and followbotpkg

2. 'ros2 run tf_relay relay'
 
3. 'ros2 run followbotpkg followbot'

Steps to run teleop.py or analog_teleop.py
1. pip3 install PyQt6
2. python3 teleop.py or python3 analog_teleop.py

Run Optitrack:
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.131 port:=3883

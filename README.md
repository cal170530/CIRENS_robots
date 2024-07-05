# CIRENS_robots
************Note: Cloning this repo into your workspace and running it doesn't seem to work(it should, but something is missing in this repo).   What does work is manually creating a package with: "ros2 pkg create --build-type ament_python --node-name agentcontrol agentcontrol_pkg".  Then copy/replace all the files in here into the package:setup.py, main.py,agentcontrol_sim.py,agentcontrol.py.  Then build the package with: "colcon build --symlink-install --packages-select agentcontrol_pkg"

tf_relay_pkg: rebroadcast the 'base_link" tf from each '\robot{i}\tf to a common '\tf' topic as child frames of a 'world' root frame. 

followbotpkg: robot1 is designated as the leader.  Each i != 1, robot{i} moves in the direction of robot1 relative to its frame. 

analog_teleop.py:  GUI control of namespaced robot with analog dial

teleop.py: Gui control of a namespaced robot using arrow buttons

Steps to run agentcontroller(consensus algorithm at the moment):

# running live robots without optitrack:

1. Build tf_relay_pkg and agentcontrol_pkg
2. Modify main.py to include the indices of the robots that will participate, e.g., 'agents = [1,2,5,10]'
3. 'ros2 run tf_relay relay'
4. 'ros2 run agentcontrol_pkg agentcontrol'

# running simulator or live robots with optitrack:   
1. Build agentcontrol_pkg
2. If running simulator: 'ros2 run agentcontrol_pkg agentcontrol -h" to see options
3. If running optitrack: 'ros2 run agentcontrol_pkg agentcontrol -h" to see options
   
Example usage:

ros2 run agentcontrol_pkg agentcontrol -cm -a 3 7 9          

-This runs consensus mode with agents set: {robot3, robot7, robot9}  

ros2 run agentcontrol_pkg agentcontrol -lfm -a 3 7 9 -l 3    

-This runs leader-follower mode with agent set: {robot3, robot7, robot9} and leader set: {robot3}

ros2 run agentcontrol_pkg agentcontrol -lfm -p /agentcontrol_pkg/agentcontrol_pkg/agent_setup.yaml  

-This runs leader-follower mode with the specified agent_setup.yaml path. 

-options not specified will be set to the default

Steps to run teleop.py or analog_teleop.py
1. pip3 install PyQt6
2. python3 teleop.py or python3 analog_teleop.py

-----Running Optitrack:
After installing packages detailed in the Box folder, this is the launch command to receive mocap topics:

ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.131 port:=3883

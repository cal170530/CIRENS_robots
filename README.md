# CIRENS_robots

tf_relay_pkg: rebroadcast the 'base_link" tf from each '\robot{i}\tf to a common '\tf' topic as child frames of a 'world' root frame. 

followbotpkg: robot1 is designated as the leader.  Each i != 1, robot{i} moves in the direction of robot1 relative to its frame. 

Steps:

1.Build tf_relay_pkg and followbotpkg

2. 'ros2 run tf_relay relay'
3. 
4. 'ros2 run followbotpkg followbot'


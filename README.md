# CIRENS_robots

tf_relay_pkg: rebroadcast the 'base_link" tf from each '\robot{i}\tf to a common '\tf' topic as child frames of a 'world' root frame. 

followbotpkg: robot1 is designated as the leader.  Each i != 1, robot{i} moves in the direction of robot1 relative to its frame. 

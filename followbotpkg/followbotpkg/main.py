import rclpy
from tf_relay.tf_relay import TFRelay
from tf_relay.tf_static_relay import TFStaticRelay
from rclpy.executors import SingleThreadedExecutor
import sys
from followbotpkg.followbot import FollowBot
from followbotpkg.leaderbot import LeaderBot

def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    agents = 2
    namespace = 'robot'

    if len(sys.argv) > 1:
        namespace = str(sys.argv[1])
        agents = int(sys.argv[2])

    followbot= []
    leaderbot = []

    try:
        
        followbot = FollowBot(agent='robot2',leader='robot1')
        #leaderbot = LeaderBot()
        #executor.add_node(leaderbot)
        executor.add_node(followbot)

        try:
            executor.spin()
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
        executor.shutdown()


if __name__ == '__main__':
    main()
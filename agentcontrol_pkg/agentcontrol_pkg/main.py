import rclpy
from rclpy.executors import SingleThreadedExecutor
import sys
from agentcontrol_pkg.agentcontrol import AgentController


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    agents = 2
    namespace = 'robot'

    if len(sys.argv) > 1:
        namespace = str(sys.argv[1])
        agents = int(sys.argv[2])
    N= []
    controllers = []
    for i in range(agents):
        Ni =[]
        for j in range(agents):
            Ni.append('robot'+str(j))
        N.append(Ni)
    
    

    
    
    try:
        for i in range(agents): 
              
            controller = AgentController(i,neighbors = N[i])
            controllers.append(controller)
      
        for controller in controllers:
            executor.add_node(controller)
        try:
            executor.spin()
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
            executor.shutdown()


if __name__ == '__main__':
    main()

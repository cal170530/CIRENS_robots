import rclpy
from rclpy.executors import SingleThreadedExecutor
import sys
from agentcontrol_pkg.agentcontrol import AgentController
from agentcontrol_pkg.agentcontrol_sim import AgentController_Sim
from agentcontrol_pkg.LF_formation_ctl import LF_formation_ctl
def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    agents = [1,2]
    leader = 3
    namespace = 'robot'
    mode = False

    #if len(sys.argv) == 1:
    match(len(sys.argv)):
        case 2:
            mode = int(sys.argv[1])
        case 3:
            namespace = str(sys.argv[2])
        case 4:
            agents = int(sys.argv[3])
        
    N= dict.fromkeys(agents)
    controllers = []
    for agent in agents:
        if agent != leader:
            Ni =[]
            for neighbor in agents:
                Ni.append(namespace+str(neighbor))
            N[agent] = Ni
        
    

    
    
    try:
        for agent in agents: 
            if mode ==0 or mode == 1:
                controller = AgentController_Sim(agent,neighbors = N[agent],mode = mode)
            elif mode ==2 and agent != leader:
                controller = LF_formation_ctl(agent,neighbors = N[agent],leader = leader, mode = mode) 
            else:
                controller = AgentController(agent,neighbors = N[agent])
            if agent != leader:
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

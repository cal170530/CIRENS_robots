import rclpy
from rclpy.executors import SingleThreadedExecutor
import sys
from agentcontrol_pkg.agentcontrol import AgentController
from agentcontrol_pkg.agentcontrol_sim import AgentController_Sim
from agentcontrol_pkg.LF_formation_ctl import LF_formation_ctl
from agentcontrol_pkg.Consensus_ctl import Consensus_ctl
import argparse
def main():
    rclpy.init()
    ##
   
    
    executor = SingleThreadedExecutor()

    opt, A,formation_distances = get_opt()
    agents = opt.agents
    leaders = opt.leaders
    namespace = opt.namespace
    mode = opt.mode
    #if len(sys.argv) == 1:
   
    N= dict.fromkeys(agents)
    Fd = dict.fromkeys(agents)
    controllers = []

    for i in range(len(agents)):
        if i not in leaders:
            Ni = []
            Fdi = dict.fromkeys(agents)
            for j in len(A[i]):
                if A[i][j]  != 0:
                    Ni.append(agents[j])
                    if opt.lformation:
                        Fdi[agents[j]] = formation_distances[i][j]
            N[agents[i]] = Ni
            if opt.lformation:
                Fd[agents[i]] = Fdi
    
    
    try:
        for agent in agents: 
            if mode ==0 or mode == 1:
                controller = Consensus_ctl(agent,neighbors = N[agent], namespace = opt.namespace, mode = mode)
            elif mode ==2:
                controller = LF_formation_ctl(agent,neighbors = N[agent],namespace = opt.namespace,leaders = leaders,Fd = Fd[agent], mode = mode) 
            else:
                controller = AgentController(agent,neighbors = N[agent])
            controllers.append(controller)

        for controller in controllers:
            executor.add_node(controller)
        try:
            executor.spin()
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
            executor.shutdown()

def get_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument("-m","--mode" ,type = int, help = "",default =0, help ="Enter mode: 0 = optitrack, 1= sim, 2 = tf/odom")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-cm","--consensus",action= "store_true",default = False, help = "Consensus Mode")
    group.add_argument("-lfm","--lformation",action = "store_true", default = False, help = " Leader/follower formation mode")
    parser.add_argument("-p", "--path",type = str, default = "agent_setup.yaml",help = "/path/to/agent_setup.yaml")
    parser.add_argument("-a","--agents", nargs = '+',type = int)
    parser.add_argument("-l","--leaders", nargs = '+',type = int )
    parser.add_argument("-ns","--namespace",type = str, default = 'robot', help = "namespace to be used")
    opt = parser.parse_args()
    formation_distances = []
    num_agents = len(opt.agents)
    with open(opt.path, 'r') as f:
         data = yaml.safe_load(f)
    if opt.consensus:
        A = np.ones((num_agents,num_agents))
    else:
        A = np.array(data['adjacency_matrix'])
    if opt.lformation: 
        formation_distances = np.array(data['formation_distances'])
    
    
    return opt,A,formation_distances

if __name__ == '__main__':
    main()

 

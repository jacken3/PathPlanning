import random
from env import Maze
import numpy as np
from Agent_brain import Agent
from config import Config
import pickle
import time

def taskExecute():

    Mission_Lost=[0]*len(Agent_list)
    Mission_stage=[0]*len(Agent_list)
    Mission_compelete=np.array([0]*len(Agent_list))
    #Mission_change=[0]*len(Agent_list)
    Agnet_direction={}
    DontGo=[[0]]*len(Agent_list)
    stage=[len(goal) for goal in dis_list]
    direction_all=[[25,0,25,0],[-25,0,-25,0],[0,25,0,25],[0,-25,0,-25]]
    index=[0]*len(Agent_list)
    Flag=0

    while not Mission_compelete.all():
        time.sleep(1)
        for Agent_tag in range(len(RouteFinal)):
            #任务状态未丢失则按原定路线行进
            if not Mission_Lost[Agent_tag] and not Mission_compelete[Agent_tag]:
                state_now=RouteFinal[Agent_tag][index[Agent_tag]]
                state_next=RouteFinal[Agent_tag][index[Agent_tag]+1]
                base_action=np.array([0,0])
                base_action[0] = state_next[0] - state_now[0]
                base_action[1] = state_next[1] - state_now[1]
                env_real.canvas.move(env_real.Agent_rect[Agent_tag],base_action[0],base_action[1])
                env_real.canvas.move(env_real.Agent[Agent_tag],base_action[0],base_action[1])
                env_real.update()
                index[Agent_tag]+=1
                #当前状态为之前仿真时定义的阶段终点
                if state_next==goal_coords[dis_list[Agent_tag][Mission_stage[Agent_tag]]]:
                    #但探测到不是实际地图中的目标点
                    if not state_next==env_real.canvas.coords(env_real.goal[dis_list[Agent_tag][Mission_stage[Agent_tag]]]):
                        #将机器人设为任务丢失状态
                        Mission_Lost[Agent_tag]=1
                    else:
                        Mission_stage[Agent_tag] += 1
                        if Mission_stage[Agent_tag]==stage[Agent_tag]:
                            Mission_compelete[Agent_tag]=1

            #任务丢失
            elif  Mission_Lost[Agent_tag] and not Mission_compelete[Agent_tag]:
                #第一次初始化矫正的目标
                if not Agnet_direction.setdefault(Agent_tag,0):
                    dx=goal_real[dis_list[Agent_tag][Mission_stage[Agent_tag]]][0]-goal[dis_list[Agent_tag][Mission_stage[Agent_tag]]][0]
                    dy=goal_real[dis_list[Agent_tag][Mission_stage[Agent_tag]]][1]-goal[dis_list[Agent_tag][Mission_stage[Agent_tag]]][1]
                    Agnet_direction[Agent_tag]=[dx,dy]
                #矫正路线
                next_direction=[]
                #右移
                if Agnet_direction[Agent_tag][0]>0:
                    next_direction.append(np.array([25,0,25,0]))
                #左移
                if Agnet_direction[Agent_tag][0]<0:
                    next_direction.append(np.array([-25,0,-25,0]))
                #下移
                if Agnet_direction[Agent_tag][1]>0:
                    next_direction.append(np.array([0,25,0,25]))
                #上移
                if Agnet_direction[Agent_tag][1]<0:
                    next_direction.append(np.array([0,-25,0,-25]))
                #已经到达校正后的目标
                if not next_direction:
                    Mission_Lost[Agent_tag] = 0
                    Mission_stage[Agent_tag] += 1
                    Agnet_direction[Agent_tag]=0
                    DontGo[Agent_tag]=[[0]]
                    if Mission_stage[Agent_tag]==stage[Agent_tag]:
                        Mission_compelete[Agent_tag]=1
                    #如果是中途目标发生变化则重新规划路线
                    if not Mission_compelete[Agent_tag]:
                        Path_new=RePlanning(goal_coords[dis_list[Agent_tag][Mission_stage[Agent_tag]]],env_real.canvas.coords(env_real.Agent_rect[Agent_tag]))
                        idx=index[Agent_tag]
                        while not RouteFinal[Agent_tag][idx]==goal_coords[dis_list[Agent_tag][Mission_stage[Agent_tag]]]:
                            del RouteFinal[Agent_tag][idx]
                        del RouteFinal[Agent_tag][idx]
                        for state in Path_new:
                            RouteFinal[Agent_tag].insert(idx,state)
                            idx+=1

                else:
                    for each in next_direction:
                        print(list(each))
                        print(list(each+np.array(env_real.canvas.coords(env_real.Agent_rect[Agent_tag]))))
                        if list(each) not in DontGo[Agent_tag] \
                            and list(each+np.array(env_real.canvas.coords(env_real.Agent_rect[Agent_tag]))) not in env_real.Obs:
                           env_real.canvas.move(env_real.Agent_rect[Agent_tag],each[0],each[1])
                           env_real.canvas.move(env_real.Agent[Agent_tag],each[0],each[1])
                           env_real.update()
                           Agnet_direction[Agent_tag][0]-=each[0]/25
                           Agnet_direction[Agent_tag][1]-=each[1]/25
                           DontGo[Agent_tag]=[list(-each)]
                           Flag=1
                           break
                    if not Flag:
                        for each in direction_all:
                            if list(np.array(each)+np.array(env_real.canvas.coords(env_real.Agent_rect[Agent_tag]))) not in env_real.Obs and\
                                each not in DontGo[Agent_tag]:  
                                env_real.canvas.move(env_real.Agent_rect[Agent_tag],each[0],each[1])
                                env_real.canvas.move(env_real.Agent[Agent_tag],each[0],each[1])
                                env_real.update()
                                Agnet_direction[Agent_tag][0]-=each[0]/25
                                Agnet_direction[Agent_tag][1]-=each[1]/25
                                DontGo[Agent_tag]=[list(-np.array(each))]
                                break
                    Flag=0
            

    print("Mission Completed")

def RePlanning(goal,state):
    state_now=state
    pher=env_real.pheromone[str(goal)]
    final_route=[state_now]
    while (True):
        
        #获取当前状态的邻居状态
        neighbors=get_neighbors(state_now,env_real,goal)
        #待选的下个状态的列表
        state_next=[]
        concen_max=0

        for neighbor in neighbors :
            if pher.setdefault(str(neighbor),0) >= concen_max:
                concen_max=pher[str(neighbor)]
        
        for neighbor in neighbors:
            if pher[str(neighbor)] == concen_max :
                state_next.append(neighbor)
        
        next_state=random.choice(state_next)
        state_now=next_state
        final_route.append(next_state)

        if next_state==goal:
            break
    return final_route  


def get_neighbors(state,env,goal):
        neighbors=[]

        surround=[[25,0,25,0],[-25,0,-25,0],[0,25,0,25],[0,-25,0,-25]]
        for each in surround:
            around=list(np.array(state)+np.array(each))
            if around not in env.Obs and (np.array(around)>0).all() and (np.array(around) < env.UNIT*env.MAZE_H).all() \
                and (around not in env.getGoal() or around == goal):
                 neighbors.append(around)

        return neighbors 

if __name__ == "__main__":

    #仿真环境和参数
    con=Config("config_4.ini")
    goal=eval(con.Maze_config["maze_goal"])
    goal_coords=[[25*(goal[i][0]-1)+2.5,25*(goal[i][1]-1)+2.5,25*(goal[i][0]-1)+22.5,25*(goal[i][1]-1)+22.5] for i in range(len(goal))]

    #实际环境配置文件
    con_real=Config("config_4_modify.ini")
    Agent_num=eval(con_real.Agent_config["num"])
    Agent_start=eval(con_real.Agent_config["start"])
    Agent_list=[]
    for i in range(Agent_num):
        Agent_list.append(Agent(Agent_start[i],e_greedy=0.8))
    
    #真实环境
    goal_real=eval(con_real.Maze_config["maze_goal"])    
    env_real = Maze(Agent_list,con_real)

    #读取仿真获得的数据
    with open("Route.data","rb") as inputfile:
        RouteFinal=pickle.load(inputfile)
    with open("Phermenon.data","rb") as inputfile:
        env_real.pheromone=pickle.load(inputfile)
    with open("dislist.data","rb") as inputfile:
        dis_list=pickle.load(inputfile)

  
    env_real.after(100, taskExecute)
    env_real.mainloop()
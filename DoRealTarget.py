from random import random
from env import Maze
import numpy as np
from Agent_brain import Agent
from config import Config
import pickle
import time

def update():
    for i in range(len(Agent_list)):
        #预计目标任务点
        pre_task=[goal_coords[j] for j in dis_list[i]]

    #按照原任务执行路径
    taskExecute()




def taskExecute():

    Mission_Lost=[0]*len(Agent_list)
    Mission_stage=[0]*len(Agent_list)
    Mission_compelete=np.array([0]*len(Agent_list))
    Agnet_direction={}
    DontGo=[]
    direction_all=[[25,0,25,0],[-25,0,-25,0],[0,25,0,25],[0,-25,0,-25]]
    index=[0]*len(Agent_list)
    flag=0

    while not Mission_compelete.all():

        for Agent_tag in range(len(RouteFinal)):
            #任务状态未丢失则按原定路线行进
            if not Mission_Lost[Agent_tag]:
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
            #任务丢失
            elif  Mission_Lost[Agent_tag]:
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
                    Mission_Lost[Agent_tag] == 0
                    Mission_stage[Agent_tag] += 1
                else:
                    for each in next_direction:
                        if list(each) not in DontGo \
                            and list(each+np.array(env_real.canvas.coords(env_real.Agent_rect[Agent_tag]))) not in env_real.Obs:
                           env_real.canvas.move(env_real.Agent_rect[Agent_tag],each[0],each[1])
                           env_real.canvas.move(env_real.Agent[Agent_tag],each[0],each[1])
                           dx-=each[0]/25
                           dy-=each[1]/25
                           Agnet_direction[Agent_tag]=[dx,dy]
                           DontGo=[list(-each)]
                           flag=1
                           break
                    if not flag:
                        for each in direction_all:
                            if list(np.array(each)+np.array(env_real.canvas.coords(env_real.Agent_rect[Agent_tag]))) not in env_real.Obs and each not in DontGo:  
                                env_real.canvas.move(env_real.Agent_rect[Agent_tag],each[0],each[1])
                                env_real.canvas.move(env_real.Agent[Agent_tag],each[0],each[1])
                                dx-=each[0]/25
                                dy-=each[1]/25
                                Agnet_direction[Agent_tag]=[dx,dy]
                                DontGo=[list(-np.array(each))]
                                flag=0
                                break

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

  
    env_real.after(100, update)
    env_real.mainloop()
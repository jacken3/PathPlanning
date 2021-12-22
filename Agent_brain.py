from typing_extensions import final
import numpy as np
from numpy import random
import random

UNIT = 25

class Agent(object):

    def __init__(self,start,e_greedy,AgentTag):
        self.actions = [0,1,2,3]  # a list
        self.start=start
        self.epsilon = e_greedy
        self.experience={}
        self.state= [12.5+(self.start[0]-1)*UNIT-10, 12.5+(self.start[1]-1)*UNIT-10,
           12.5+(self.start[0]-1)*UNIT+10, 12.5+(self.start[1]-1)*UNIT+10]
        self.tag=AgentTag

    def get_neighbors(self,env,goal):
        neighbors=[]

        surround=[[UNIT,0,UNIT,0],[-UNIT,0,-UNIT,0],[0,UNIT,0,UNIT],[0,-UNIT,0,-UNIT]]
        for each in surround:
            around=list(np.array(self.state)+np.array(each))
            if around not in env.Obs and (np.array(around)>0).all() and (np.array(around) < env.UNIT*env.MAZE_H).all(): \
                #and (around not in env.getGoal() or around == goal):
                 neighbors.append(around)

        return neighbors 

    def action(self,goal,env,HasExplored):

        #依据信息素浓度选择下一个状态
        state_next=[]
        neighbors=self.get_neighbors(env,goal)

        if np.random.rand() < self.epsilon:
    
            #获取特定目标点的信息素状态表
            pher=env.pheromone.setdefault(str(goal),dict())
            concen_max=0
            for neighbor in neighbors :
                if pher.setdefault(str(neighbor),0) >= concen_max:
                    concen_max=pher[str(neighbor)]
            
            for neighbor in neighbors:
                if pher[str(neighbor)] == concen_max and neighbor not in HasExplored:
                    state_next.append(neighbor)
            if state_next:
                next_state=random.choice(state_next)
            else:
                next_state = random.choice(neighbors)

        #随机选择动作
        else:
            next_state = random.choice(neighbors)

        return next_state

    def find_way(self,goal,env):
        #记录路径
        Route=[self.state]
        while(True):
            next_state=self.action(goal,env,Route)
            self.state=next_state
            Route.append(next_state)
            if next_state==goal:
                env.pheromoneBP(goal,Route)
                break
        return Route

    def final_route(self,cluster,env):

        self.state= [12.5+(self.start[0]-1)*UNIT-10, 12.5+(self.start[1]-1)*UNIT-10,
           12.5+(self.start[0]-1)*UNIT+10, 12.5+(self.start[1]-1)*UNIT+10]
        final_route=[self.state]
        #获得对应目标的信息素
        
        for goal in cluster:
            pher=env.pheromone.setdefault(str(goal),dict())

            while (True):
                #获取当前状态的邻居状态
                neighbors=self.get_neighbors(env,goal)
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
                self.state=next_state
                final_route.append(next_state)

                if next_state==goal:
                    break
                

        return final_route       


        

import imp
from typing_extensions import final
import numpy as np
from numpy import random
import random
import heapq
import math

UNIT = 25

"""
Agent类 内部包含智能体路径的生成逻辑函数
"""
class Agent(object):

    #初始化函数
    def __init__(self,start,e_greedy,AgentTag):
        self.actions = [0,1,2,3]  # 动作序列 上下左右
        self.start=start          #初始化起始点坐标
        self.epsilon = e_greedy   #探索率，在主程序中给定探索率
        self.experience={}        #经验记录字典，记录整个地图的信息素
        self.state= [12.5+(self.start[0]-1)*UNIT-10, 12.5+(self.start[1]-1)*UNIT-10,
           12.5+(self.start[0]-1)*UNIT+10, 12.5+(self.start[1]-1)*UNIT+10] #将起始点坐标转为像素坐标
        self.tag=AgentTag         #智能体编号

    #获取智能体四周合法的邻居栅格，所谓合法指的是不为障碍物且在地图之内
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

    def A_find_way(self,goal_coord,env):

        #初始化开集、闭集、父节点字典以及距离表
        start=tuple(self.start)
        OPEN = []  
        CLOSED = []  
        PARENT = dict()  
        g = dict()
        goal=(round((goal_coord[0]-2.5)/UNIT+1),round((goal_coord[1]-2.5)/UNIT+1))

        PARENT[start] = start
        g[start] = 0
        g[goal] = math.inf

        cost=lambda s,s_n:math.hypot(s_n[0] - s[0], s_n[1] - s[1]) #准确的cost
        heuristic=lambda s: math.hypot(goal[0] - s[0], goal[1] - s[1]) #用欧氏距离作为启发度函数
        f_value=lambda s: g[s] + heuristic(s) #计算节点的优先度
         

        heapq.heappush(OPEN,(f_value(start), start))
        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == goal:  # stop condition
                break

            for s_n in self.A_get_neighbor(s,env):
                new_cost = g[s] + cost(s, s_n)

                if s_n not in g:  #实际是对新加入的节点做了一个初始化
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s

                    heapq.heappush(OPEN, (f_value(s_n), s_n)) 

        Route = [goal]
        s = goal

        while True:
            s = PARENT.setdefault(s,None)
            Route.append(s)

            if s == start:
                break

        list.reverse(Route)
        Route_coord=[[12.5+(s[0]-1)*UNIT-10, 12.5+(s[1]-1)*UNIT-10,
           12.5+(s[0]-1)*UNIT+10, 12.5+(s[1]-1)*UNIT+10] for s in Route]
        
        env.pheromone.setdefault(str(goal_coord),dict())
        env.pheromoneBP(goal_coord,Route_coord)

        return Route_coord   

    def A_get_neighbor(self,s,env):
        neighbors=[]

        surround=[(1,0),(0,1),(-1,0),(0,-1)]
        for each in surround:
            around=(s[0]+each[0],s[1]+each[1])
            if around not in eval(env.con.Maze_config["maze_obs"]) and (np.array(around)>0).all() and (np.array(around) <= env.MAZE_H).all(): 
                 neighbors.append(around)

        return neighbors 


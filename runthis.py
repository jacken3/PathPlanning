from random import random
from env import Maze
import numpy as np
from Agent_brain import Agent
import random
from config import Config
import pandas as pd
import math


def update():

    goal_list=env.getGoal()

    for episode in range(500): 
        
        visual = 0 
        
        #重置每个机器人的状态
        for Robot in Agent_list:
            Robot.state= [12.5+(Robot.start[0]-1)*env.UNIT-10, 12.5+(Robot.start[1]-1)*env.UNIT-10,
           12.5+(Robot.start[0]-1)*env.UNIT+10, 12.5+(Robot.start[1]-1)*env.UNIT+10]
        #重置地图
        env.reset(visual)
        env.update()  

        #每个Agent随机寻路,需要的参数为Agent的目标
        for i,robot in zip(range(len(Agent_list)),Agent_list):
            
            Route=robot.find_way(goal_list[i],env)
            env.update()
            env.show_route(Route,i,0)

        #change goal for each Agent 本质是对goal列表进行了一个移位操作
        goal_list.insert(0,goal_list.pop())

    for Robot in Agent_list:
            Robot.state= [12.5+(Robot.start[0]-1)*env.UNIT-10, 12.5+(Robot.start[1]-1)*env.UNIT-10,\
                          12.5+(Robot.start[0]-1)*env.UNIT+10, 12.5+(Robot.start[1]-1)*env.UNIT+10]
    env.reset(visual)

    # 地图信息素信息构建完毕，开始实现目标的K-means聚类
    goal_cluster=env.goalCluster()
    print("分组情况：",goal_cluster)


    # 地图信息素信息构建完毕，进入任务调度处理 任务分配的标准为最大化信息素浓度和

    concen_sum_max=0
    
    for each_allocation_scheme in range(len(goal_cluster)):
        concen_sum=0
        for cluster,Agent in zip(goal_cluster,Agent_list):
            for goal in cluster:
                print(env.pheromone[str(goal)][str(Agent.state)])
                concen_sum += env.pheromone[str(goal)][str(Agent.state)]
        if concen_sum > concen_sum_max:
            concen_sum_max=concen_sum
            goal_last=[cluster for cluster in goal_cluster]

        goal_cluster.insert(0,goal_cluster.pop())

    #任务分配完毕，开始统筹路径中的避障问题
    Route_final=[]
    for cluster,robot in zip(goal_last,Agent_list):
            Route_final.append(robot.final_route(cluster,env))


    #Route_final=[[[2,1],[1,1],[1,2]],[[1,0],[1,1],[2,1]],[[2,0],[1,0],[1,1]]] 测试用
    #记录最长路径
    route_len=2
    #记录当前遍历的路径长度
    i=0
    len_Route=[0]*Agent_num
    while i < route_len-1:
        
        #右对齐所有路径
        for Agent_tag,route in zip(range(Agent_num),Route_final):
            if len(route)==i+1:
                if len_Route[Agent_tag]==0:
                    len_Route[Agent_tag]=len(route)
                route.append(route[i])
                

        #同一时刻各个Agent所在的坐标
        stateSameTime=[tuple(Route_final[Agent_index][i]) for Agent_index in range(len(Agent_list))]
        #下一个时刻各个Agent所在的坐标
        stateSameTimeNext=[tuple(Route_final[Agent_index][i+1]) for Agent_index in range(len(Agent_list))]

        collision=check_cons(stateSameTime,stateSameTimeNext)
        while collision[0] or collision[1]:
            """""
            点冲突的判断与处理

            判断是否有相同的坐标出现在同一时间，有则判定出现点冲突，对路径进行处理
            分当前时刻和未来时刻同时处理是为了避免出现避障造成新点冲突的情况
            例子
            (2,1)(1,0)(2,0)     (2,1)(1,0)(2,0)    (2,1)(1,0)(2,0) 
            (1,1)(1,1)(1,0)     (1,1)(1,0)(1,0)    (1,1)(1,0)(2,0)
            (1,2)(2,1)(1,1)-->  (1,2)(1,1)(1,1)--> (1,2)(1,1)(1,0)
                                (1,2)(2,1)(1,1)    (1,2)(2,1)(1,1)
            """""
            vertex_collision=collision[0]
            for con in vertex_collision:
                if not Route_final[con[0]][i]==Route_final[con[0]][i-1]:
                    Route_final[con[0]].insert(i,Route_final[con[0]][i-1])
                else:
                    Route_final[con[1]].insert(i,Route_final[con[1]][i-1])
            #更新
            stateSameTime=[tuple(Route_final[Agent_index][i]) for Agent_index in range(len(Agent_list))]
            stateSameTimeNext=[tuple(Route_final[Agent_index][i+1]) for Agent_index in range(len(Agent_list))]
            collision=check_cons(stateSameTime,stateSameTimeNext)
            
            """""
            边冲突的判断与处理

            判断是否存在坐标交叉前后时刻相等的情况
            """""
            edge_collision=collision[1]

            for con in edge_collision:
                #插入一个合法的坐标
                Route_final[con[0]].insert(i+1,legal_state(Route_final,con[0],i,env))
                #回到原路径上来
                Route_final[con[0]].insert(i+2,Route_final[con[0]][i])
            #更新
            stateSameTime=[tuple(Route_final[Agent_index][i]) for Agent_index in range(len(Agent_list))]
            stateSameTimeNext=[tuple(Route_final[Agent_index][i+1]) for Agent_index in range(len(Agent_list))]
            collision=check_cons(stateSameTime,stateSameTimeNext)

        route_len=max([len(route) for route in Route_final])
        i+=1
    for i in range(Agent_num):
        if len_Route[i]==0:
            len_Route[i]=route_len
    step_num_table=pd.DataFrame(columns=range(1,Agent_num+1),data=[len_Route])
    step_num_table.to_csv('step_count.csv',mode='a')
    # for i in range(Agent_num):
    #     count=0
    #     state_last=[0,0,0,0]
    #     while not Route_final[i][count]==state_last and count <len(Route_final[i])-1 :
    #         state_last=Route_final[i][count]
    #         count+=1
    #     len_Route.append(count)
    print(len_Route)
    env.final(Route_final)
    env.destroy()
    print('game over!')
    

def check_cons(state_union,state_union_next):

    vertex_collision=[]
    edge_collision=[]
    for i in range(len(state_union)):
        for j in range(i+1,len(state_union)):
            #边冲突
            if state_union[i]==state_union_next[j] and state_union[j]==state_union_next[i]:
                edge_collision.append((i,j))
            #顶点冲突
            if state_union[i]==state_union[j]:
                vertex_collision.append((i,j))

    return [vertex_collision,edge_collision]


def legal_state(route,index,time,env):
    state_now=np.array(route[index][time])
    #surround=np.array([[1,0],[-1,0,],[0,1],[0,-1]])
    surround=np.array([[env.UNIT,0,env.UNIT,0],[-env.UNIT,0,-env.UNIT,0],[0,env.UNIT,0,env.UNIT],[0,-env.UNIT,0,-env.UNIT]])
    states=[]
    for each in surround :
        #判断当前状态周围的除原路径上的其他可选状态env.UNIT*env.MAZE_H
        if list(state_now+each) not in env.Obs and (np.array(state_now+each)> 0).all() and (np.array(state_now+each) < env.UNIT*env.MAZE_H).all()\
                                            and not list(state_now+each)==route[index][time+1] and list(state_now+each) not in [s[time] for s in route]+[s[time+1] for s in route]:
                                            states.append(list(state_now+each))
    return random.choice(states)

if __name__ == "__main__":

    con=Config("config_4.ini")
    Agent_num=eval(con.Agent_config["num"])
    Agent_start=eval(con.Agent_config["start"])
    Agent_list=[]
    for i in range(Agent_num):
        Agent_list.append(Agent(Agent_start[i],e_greedy=0.8))

    env = Maze(Agent_list,con)

    env.after(100, update)
    env.mainloop()
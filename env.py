"""
Reinforcement learning maze example.
Red rectangle:          explorer.
Black rectangles:       hells       [reward = -1].
Yellow bin circle:      paradise    [reward = +1].
All other states:       ground      [reward = 0].
This script is the environment part of this example.
The RL is in RL_brain.py.
View more on my tutorial page: https://morvanzhou.github.io/tutorials/
"""


from tkinter.constants import TRUE
import numpy as np
import time
import math
#import sys
import tkinter as tk
from config import Config
from PIL import Image, ImageTk
from numpy.lib.twodim_base import triu_indices_from
import pandas as pd
from itertools import combinations


class Maze(tk.Tk, object):
    def __init__(self,Agent_list,con):
        super(Maze, self).__init__()
        self.con=con
        self.UNIT = eval(con.Maze_config["unit"])
        self.MAZE_H =eval(con.Maze_config["maze_h"])  
        self.MAZE_W =eval(con.Maze_config["maze_w"]) 
        self.Agent_list=Agent_list
        self.title('MAPP Simulation System v1.0')
        self.geometry('{0}x{1}'.format(self.MAZE_H * self.UNIT, self.MAZE_H * self.UNIT))
        self.iconbitmap('VisualSrc/HITlogoblue.ico')
        #以具体坐标的形式保存了所有障碍物
        self.Obs=[]
        #以在画布中的序号保存Agent
        self.Agent=[]
        #以在画布中的序号保存每个Agent对应的透明矩形方块
        self.Agent_rect=[]
        #以在画布中的序号保存每个目标
        self.goal=[]
        self.goal_img=[]
        #AgentIMG列表
        self.img_UAV=[]
        self._build_maze(con)
        #环境中的信息素(索引方式为str(goal)+str(state))
        self.pheromone={}

    def _build_maze(self,con):
        self.canvas = tk.Canvas(self, bg='gray',
                           height=self.MAZE_H * self.UNIT,
                           width=self.MAZE_W * self.UNIT)

        # create grids
        for c in range(0, self.MAZE_W * self.UNIT, self.UNIT):
            x0, y0, x1, y1 = c, 0, c, self.MAZE_H * self.UNIT
            self.canvas.create_line(x0, y0, x1, y1, fill = 'black', dash = (4,4))
        for r in range(0, self.MAZE_H * self.UNIT, self.UNIT):
            x0, y0, x1, y1 = 0, r, self.MAZE_W * self.UNIT, r
            self.canvas.create_line(x0, y0, x1, y1,fill = 'black', dash = (4,4))
        
        # create origin
        origin = np.array([12.5, 12.5])

        Obs=eval(con.Maze_config["maze_obs"]) 
        for each in range(len(Obs)):
            hell_center=origin + np.array([self.UNIT * (Obs[each][0]-1), self.UNIT * (Obs[each][1]-1)])
            hell=self.canvas.create_rectangle(
                hell_center[0] - 10, hell_center[1] - 10,
                hell_center[0] + 10, hell_center[1] + 10,
                fill='gray')
            self.Obs.append(self.canvas.coords(hell))

       
        global MBase_image 
        image = Image.open('VisualSrc/MBase.gif')
        image=image.resize((20,20))
        MBase_image = ImageTk.PhotoImage(image)

        goal=eval(con.Maze_config["maze_goal"]) 
        for each in range(len(goal)):
            oval_center=origin + np.array([self.UNIT * (goal[each][0]-1), self.UNIT * (goal[each][1]-1)])
            self.goal.append(self.canvas.create_rectangle(
                oval_center[0] - 10, oval_center[1] - 10,
                oval_center[0] + 10, oval_center[1] + 10,
                fill='',outline = ''))
            self.goal_img.append(self.canvas.create_image(oval_center[0], oval_center[1], anchor='center',image=MBase_image))
        

        # create red rect 1
        for each in range(len(self.Agent_list)):
            image_file=Image.open('VisualSrc/UAV_1.gif')#num'+str(each+1)+'.png
            image_file=image_file.resize((20,20))
            self.img_UAV.append(ImageTk.PhotoImage(image_file))
            x=(self.Agent_list[each].start[0]-1)*self.UNIT
            y=(self.Agent_list[each].start[1]-1)*self.UNIT
            rect_center = origin + np.array([x, y])
            self.Agent.append(self.canvas.create_image(rect_center[0], rect_center[1], anchor='center',image=self.img_UAV[each]))
            self.Agent_rect.append (self.canvas.create_oval(
                rect_center[0] - 10, rect_center[1] - 10,
                rect_center[0] + 10, rect_center[1] + 10,
                fill='',outline = ''))      
        self.canvas.pack()

    def reset(self,visual):
        
        if visual == 1:
            time.sleep(0.5)

        for i in range(len(self.Agent)):
            self.canvas.delete(self.Agent[i])
            self.canvas.delete(self.Agent_rect[i])
        self.Agent.clear()
        self.Agent_rect.clear()
        origin = np.array([12.5, 12.5])

        for each in range(len(self.Agent_list)):
            x=(self.Agent_list[each].start[0]-1)*self.UNIT
            y=(self.Agent_list[each].start[1]-1)*self.UNIT
            rect_center = origin + np.array([x, y])
            self.Agent.append(self.canvas.create_image(rect_center[0], rect_center[1], anchor='center',image=self.img_UAV[each]))
            rect_center = origin + np.array([x, y])
            self.Agent_rect.append (self.canvas.create_oval(
                rect_center[0] - 10, rect_center[1] - 10,
                rect_center[0] + 10, rect_center[1] + 10,
                fill='',outline = ''))
        self.update()
       
    def getGoal(self):
        goal=[]
        for each in self.goal:
            goal.append(self.canvas.coords(each))
        return goal
    #信息素回溯
    def pheromoneBP(self,goal,Route):
        Concentration=1
        gamma=0.9
        for state in Route[::-1]:
            if Concentration > self.pheromone[str(goal)].setdefault(str(state),0):
                self.pheromone[str(goal)][str(state)]=Concentration
            Concentration *= gamma

    def show_route(self,route,Agent_tag,visual):
        for i in range(len(route)-1):
            state_now=route[i]
            state_next=route[i+1]
            base_action=np.array([0,0])
            base_action[0] = state_next[0] - state_now[0]
            base_action[1] = state_next[1] - state_now[1]
            if visual==1:
                time.sleep(0.5)
            self.canvas.move(self.Agent_rect[Agent_tag],base_action[0],base_action[1])
            self.canvas.move(self.Agent[Agent_tag],base_action[0],base_action[1])
            self.update()

    def final(self,Route_list):
        length=max(len(route) for route in Route_list)

        #将所有Agent的路径对齐
        for route in Route_list:
            while len(route) < length:
                route.append(route[-1])

        for i in range(length-1):

            time.sleep(1)
            for Agent_tag,route in zip(range(len(Route_list)),Route_list):
                state_now=route[i]
                state_next=route[i+1]
                base_action=np.array([0,0])
                base_action[0] = state_next[0] - state_now[0]
                base_action[1] = state_next[1] - state_now[1]
                self.canvas.move(self.Agent_rect[Agent_tag],base_action[0],base_action[1])
                self.canvas.move(self.Agent[Agent_tag],base_action[0],base_action[1])
                self.update()

    def goalCluster(self):
        goal_cluster=[]
        if len(self.Agent_list)<=len(self.goal):
            goal_list=self.getGoal()
            goalEigenvector=[]
            distance_dic={}
            for goal in goal_list:
                Eigenvector=[]
                for Agent in self.Agent_list:
                    Eigenvector.append(self.pheromone[str(goal)][str(Agent.state)]) 
                goalEigenvector.append(Eigenvector)
            for i in range(len(goalEigenvector)):
                print("第%d个目标的特征向量："%(i+1),goalEigenvector[i])
                for j in range(i+1,len(goalEigenvector)):
                    dif=list(np.array(goalEigenvector[i])/np.array(goalEigenvector[j]))
                    dst=sum([abs(math.log(diff,0.9)) for diff in dif])
                    distance_dic[(i,j)]=dst
            distance_dic=sorted(distance_dic.items(),key=lambda x:x[1])
            cluster=[i for i in range(len(self.goal))]
            for k in range(len(self.goal)-len(self.Agent_list)):
                for i in range(len(self.goal)):
                    if cluster[i]==cluster[distance_dic[k][0][1]]:
                        cluster[i]=cluster[distance_dic[k][0][0]]
            cluster_dic={}
            for i in range(len(cluster)):
                if cluster_dic.setdefault(cluster[i],None)==None:  
                    cluster_dic[cluster[i]]=[goal_list[i]]
                else:
                    cluster_dic[cluster[i]].append(goal_list[i]) 
            for k in cluster_dic.keys():
                goal_cluster.append(cluster_dic[k])
            
        else:
            goal_list=self.getGoal()
            for goal in goal_list:
                goal_cluster.append([goal])
            print("Agent数量与Goal数量一致，无需聚类")
        
        return goal_cluster

    #利用相似度进行聚类
    def goalCluster_by_similarity(self):
        goalsCoord=self.getGoal()
        #相似度Table 行键为Agent的ID 列族为两个目标的两两组合
        TwogoalCombine=list(combinations(range(len(goalsCoord)),2))
        S_Table=pd.DataFrame(columns=TwogoalCombine,dtype=np.float64)

        for robot in self.Agent_list:
            S_Table=S_Table.append(pd.Series([0]*len(TwogoalCombine),index=S_Table.columns,name=robot.tag))
            for combine in TwogoalCombine:
                route=robot.final_route([goalsCoord[combine[0]]],self)
                route_=robot.final_route([goalsCoord[combine[1]]],self)
                #计算两条路径的相似度
                sim_score=self.Sim_Caculate(route,route_)
                S_Table.loc[robot.tag,[combine]]=sim_score       
        print("相似性矩阵:\n",S_Table)
        Sum_similarity=S_Table.apply(lambda x: x.sum())

        #将相似度最高的几个目标点进行聚类
        max_sort=Sum_similarity.nlargest(len(TwogoalCombine)).index
        # idx=0
        # max_n=list()
        # for _ in range(len(goalsCoord)-len(self.Agent_list)):
        #     if max_sort[idx][0][0] in 
        Close_list=[]
        Open_list=list(range(len(self.goal)))

        cluster_num=0
        for pair in max_sort:
            #集合次数足够
            if cluster_num==len(self.goal)-len(self.Agent_list):
                break
            #两个目标点都未曾参与过聚类
            elif (pair[0] in Open_list) and (pair[1] in Open_list):
                Open_list.remove(pair[0])
                Open_list.remove(pair[1])
                Close_list.append([pair[0],pair[1]])
                cluster_num+=1
            #两个目标点中至少有一个参与过聚类
            else:
                pass
                # #1号目标点没有参与过聚类
                # if pair[0] in Open_list :
                #     Open_list.remove(pair[0])
                #     for cluster in Close_list:
                #         if pair[1] in cluster:
                #             cluster.append(pair[0])
                # #2号目标点没有参与过聚类
                # elif pair[1] in Open_list:
                #     Open_list.remove(pair[1])
                #     for cluster in Close_list:
                #         if pair[0] in cluster:
                #             cluster.append(pair[1])
                # #二者都参与过聚类
                # else:
                #     for cluster1 in Close_list:
                #         if pair[0] in cluster1:
                #             for cluster2 in Close_list:
                #                 if pair[1] in cluster2:
                #                     Close_list.remove(cluster1)
                #                     Close_list.remove(cluster2)
                #                     Close_list.append(list(set(cluster1)|set(cluster2)))
                #                     break
                #             break
            
        for remain in Open_list:
            Close_list.append([remain])

        #返回目标点聚类
        goal_cluster=list()
        for cluster in Close_list:
            clusterCoord=list()
            for index in cluster:
                clusterCoord.append(goalsCoord[index])
            goal_cluster.append(clusterCoord)

        return goal_cluster

    def Sim_Caculate(self,route,route_):
        length=min(len(route),len(route_))
        len_=length
        if len(route)>=len(route_):
            for s in route_[::-1]:
                diff=np.max(abs(np.array(route)-np.array(s)),axis=1)
                if np.where(diff<=self.UNIT)[0].size!=0:
                    #sim_length=np.max(np.where(diff<=self.UNIT))+1
                    sim_score=len_/length
                    break
                sim_score=0
                len_-=1
        else:
            for s in route[::-1]:
                diff=np.max(abs(np.array(route_)-np.array(s)),axis=1)
                if np.where(diff<=self.UNIT)[0].size!=0:
                    #sim_length=np.max(np.where(diff<=self.UNIT))+1
                    sim_score=len_/length
                    break
                sim_score=0  
                len_-=1 

        return sim_score
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
#import sys
import tkinter as tk
from config import Config
from PIL import Image, ImageTk
from numpy.lib.twodim_base import triu_indices_from



class Maze(tk.Tk, object):
    def __init__(self,Agent_list,con):
        super(Maze, self).__init__()
        self.UNIT = eval(con.Maze_config["unit"])
        self.MAZE_H =eval(con.Maze_config["maze_h"])  
        self.MAZE_W =eval(con.Maze_config["maze_w"]) 
        self.Obs=[]
        self.Agent=[]
        self.Agent_rect=[]
        self.goal=[]
        self.img_UAV=[]
        self.title('MAPP Simulation System v1.0')
        self.geometry('{0}x{1}'.format(self.MAZE_H * self.UNIT, self.MAZE_H * self.UNIT))
        self.iconbitmap('HITlogoblue.ico')
        self._build_maze(Agent_list,con)
        #环境中的信息素
        self.pheromone={}

    def _build_maze(self,Agent_list,con):
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
        image = Image.open('MBase.gif')
        image=image.resize((20,20))
        MBase_image = ImageTk.PhotoImage(image)

        goal=eval(con.Maze_config["maze_goal"]) 
        for each in range(len(goal)):
            oval_center=origin + np.array([self.UNIT * (goal[each][0]-1), self.UNIT * (goal[each][1]-1)])
            self.goal.append(self.canvas.create_rectangle(
                oval_center[0] - 10, oval_center[1] - 10,
                oval_center[0] + 10, oval_center[1] + 10,
                fill='',outline = ''))
            self.canvas.create_image(oval_center[0], oval_center[1], anchor='center',image=MBase_image)
        

        # create red rect 1
        for each in range(len(Agent_list)):
            image_file=Image.open('num'+str(each+1)+'.png')
            image_file=image_file.resize((20,20))
            self.img_UAV.append(ImageTk.PhotoImage(image_file))
            x=(Agent_list[each].start[0]-1)*self.UNIT
            y=(Agent_list[each].start[1]-1)*self.UNIT
            rect_center = origin + np.array([x, y])
            self.Agent.append(self.canvas.create_image(rect_center[0], rect_center[1], anchor='center',image=self.img_UAV[each]))
            self.Agent_rect.append (self.canvas.create_oval(
                rect_center[0] - 10, rect_center[1] - 10,
                rect_center[0] + 10, rect_center[1] + 10,
                fill='',outline = ''))      
        self.canvas.pack()

    def reset(self,Agent_list,visual=1):
        
        if visual == 1:
            time.sleep(0.5)

        for i in range(len(self.Agent)):
            self.canvas.delete(self.Agent[i])
            self.canvas.delete(self.Agent_rect[i])
        self.Agent.clear()
        self.Agent_rect.clear()
        origin = np.array([12.5, 12.5])

        for each in range(len(Agent_list)):
            x=(Agent_list[each].start[0]-1)*self.UNIT
            y=(Agent_list[each].start[1]-1)*self.UNIT
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

            time.sleep(0)
            for Agent_tag,route in zip(range(len(Route_list)),Route_list):
                state_now=route[i]
                state_next=route[i+1]
                base_action=np.array([0,0])
                base_action[0] = state_next[0] - state_now[0]
                base_action[1] = state_next[1] - state_now[1]
                self.canvas.move(self.Agent_rect[Agent_tag],base_action[0],base_action[1])
                self.canvas.move(self.Agent[Agent_tag],base_action[0],base_action[1])
                self.update()

                

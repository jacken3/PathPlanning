import numpy as np
import cv2
import math
import configparser


Width=5
Height=5
UNIT=15

def callback(event,x,y,flag,param):
    x_num=math.ceil(x/UNIT)
    y_num=math.ceil(y/UNIT)
    #左键创建障碍物
    if event==cv2.EVENT_LBUTTONDOWN:
        if (x_num,y_num) not in Maze_Obs:
            Maze_Obs.append((x_num,y_num))
            cv2.rectangle(bg,(UNIT*(x_num-1)+1,UNIT*(y_num-1)+1),(UNIT*x_num-1,UNIT*y_num-1),(100,100,100),-1)
            cv2.imshow("MapConstruct",bg)
        else:
            Maze_Obs.remove((x_num,y_num))
            cv2.rectangle(bg,(UNIT*(x_num-1)+1,UNIT*(y_num-1)+1),(UNIT*x_num-1,UNIT*y_num-1),(255,255,255),-1)
            cv2.imshow("MapConstruct",bg)   
    #右键创建机器人
    if event==cv2.EVENT_RBUTTONDOWN:
        if [x_num,y_num] not in start:
            start.append([x_num,y_num])
            cv2.rectangle(bg,(UNIT*(x_num-1)+1,UNIT*(y_num-1)+1),(UNIT*x_num-1,UNIT*y_num-1),(0,0,255),-1)
            cv2.imshow("MapConstruct",bg)
        else:
            start.remove([x_num,y_num])
            cv2.rectangle(bg,(UNIT*(x_num-1)+1,UNIT*(y_num-1)+1),(UNIT*x_num-1,UNIT*y_num-1),(255,255,255),-1)
            cv2.imshow("MapConstruct",bg)
    #中键创建目标
    if event==cv2.EVENT_MBUTTONDOWN:
        if (x_num,y_num) not in Maze_Goal:
            Maze_Goal.append((x_num,y_num))
            cv2.rectangle(bg,(UNIT*(x_num-1)+1,UNIT*(y_num-1)+1),(UNIT*x_num-1,UNIT*y_num-1),(0,255,255),-1)
            cv2.imshow("MapConstruct",bg)
        else:
            Maze_Goal.remove((x_num,y_num))
            cv2.rectangle(bg,(UNIT*(x_num-1)+1,UNIT*(y_num-1)+1),(UNIT*x_num-1,UNIT*y_num-1),(255,255,255),-1)
            cv2.imshow("MapConstruct",bg)


start=list()
Maze_Goal=list()
Maze_Obs=list()
bg=np.zeros((Width*UNIT,Width*UNIT,3),np.uint8)+255
for c in range(0, Width*UNIT,UNIT):
            x0, y0, x1, y1 = c, 0, c, Width*UNIT
            cv2.line(bg,(x0,y0),(x1,y1),(0,0,0),1)
for r in range(0, Height*UNIT,UNIT):
            x0, y0, x1, y1 = 0, r, Height*UNIT, r
            cv2.line(bg,(x0,y0),(x1,y1),(0,0,0),1)
cv2.imshow("MapConstruct",bg)
cv2.namedWindow("MapConstruct")
cv2.setMouseCallback("MapConstruct", callback)
cv2.waitKey(0)
config=configparser.ConfigParser()
config.add_section("Agent")
config.set("Agent","num",str(len(start)))
config.set("Agent","start",str(start))
config.add_section("Maze")
config.set("Maze","Maze_H",str(Height))
config.set("Maze","Maze_W",str(Width))
config.set("Maze","UNIT",str(25))
config.set("Maze","Maze_Obs",str(Maze_Obs))
config.set("Maze","Maze_Goal",str(Maze_Goal))
config.write(open('MapConfig/config_test1.ini', "w"))
print("start:",start)
print("Obs:",Maze_Obs)
print("Goal:",Maze_Goal)


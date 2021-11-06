import random
from env import Maze
import numpy as np
from Agent_brain import Agent
from config import Config
import pickle
import time


def isLegal(goal,move):
    around=[[0,0,0,0],[25,0,25,0],[-25,0,-25,0],[0,25,0,25],[0,-25,0,-25],\
                    [25,25,25,25],[25,-25,25,-25],[-25,25,-25,25],[-25,-25,-25,-25]]
    #该目标已经被寻找到则不再移动
    if goalComplete.setdefault(str(env_real.canvas.coords(goal)),0):
        return False
    #目标移出边界也不移动
    coord=env_real.canvas.coords(goal)
    coord_after=np.array(coord)+np.array(move)
    if (coord_after<0).any() or coord_after[0]>env_real.MAZE_W*25 or coord_after[1]>env_real.MAZE_H*25:
        return False
    #目标移动到障碍物也不移动
    if list(coord_after) in env_real.Obs:
        return False

    #移动后目标重叠也不移动
    for each in env_real.goal:
        if env_real.canvas.coords(each) == list(coord_after):
            return False
    
    #八邻域内检测到Agent
    around=[list(i) for i in list(np.array(around)+np.array(coord))]
    for each_agent in env_real.Agent_rect:
        print(env_real.canvas.coords(each_agent))
        if env_real.canvas.coords(each_agent) in around:
            return False

    return True

def taskExecute():

    Mission_Lost=[0]*len(Agent_list)
    Mission_stage=[0]*len(Agent_list)
    Mission_compelete=np.array([0]*len(Agent_list))
    Agnet_direction={}
    DontGo=[[0]]*len(Agent_list)
    stage=[len(goal) for goal in dis_list]
    direction_all=[[25,0,25,0],[-25,0,-25,0],[0,25,0,25],[0,-25,0,-25]]
    index=[0]*len(Agent_list)
    Flag=0

    while not Mission_compelete.all():
        for each_goal,goal_img in zip(env_real.goal,env_real.goal_img):
            if np.random.random()>0.7:
                move=random.choice(direction_all)
                if isLegal(each_goal,move):
                    env_real.canvas.move(each_goal,move[0],move[1])
                    env_real.canvas.move(goal_img,move[0],move[1])
                    env_real.update()
        time.sleep(1)
        real_goal_coords=[env_real.canvas.coords(i) for i in env_real.goal]
        for Agent_tag in range(len(RouteFinal)):
            Flag2=0
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
                        goalComplete[str(state_next)]=1
                        Flag2=1
                        if Mission_stage[Agent_tag]==stage[Agent_tag]:
                            Mission_compelete[Agent_tag]=1
                #提前遭遇其他非预设目标
                if  (state_next in real_goal_coords) & (not Flag2):
                    goalComplete[str(state_next)]=1
                    #获取当前遇到的goal的标签
                    goal_tag=real_goal_coords.index(state_next)
                    #tag_agent是机器人的标签
                    Flag_find=False
                    for tag_agent in range(len(dis_list)):
                        #stage_tag是阶段的标签 
                        for stage_tag in range(len(dis_list[tag_agent])):
                            #找到该目标实际对应着哪个Agent的哪一个阶段的目标
                            if goal_tag==dis_list[tag_agent][stage_tag]:
                                stage_now=Mission_stage[tag_agent]
                                #该Agent还未执行到被影响的阶段：则直接对后半段执行重规划
                                if stage_tag > stage_now:
                                    # if tag_agent==Agent_tag:
                                    #     a=1
                                    #     pass
                                    if stage_tag+1>=len(dis_list[tag_agent]):
                                        dis_list[tag_agent].remove(stage_tag)
                                        stage[tag_agent]-=1
                                        Flag_find=True
                                        break
                                    #获取一条直达路径
                                    start=goal_coords[dis_list[tag_agent][stage_tag-1]]
                                    #start=env_real.canvas.coords(env_real.goal[dis_list[tag_agent][stage_tag-1]])
                                    end=goal_coords[dis_list[tag_agent][stage_tag+1]]
                                    #end=env_real.canvas.coords(env_real.goal[dis_list[tag_agent][stage_tag+1]])
                                    Path_new=RePlanning(end,start)
                                    #将获取的新路径插入到原来的路径之中
                                    #两个步骤。第一步首先删除原来在这两个状态之间的路径；第二部将新路径插入其中
                                    index_start=RouteFinal[tag_agent].index(start)
                                    index_end=RouteFinal[tag_agent].index(end)
                                    del RouteFinal[tag_agent][index_start:index_end+1]
                                    for s in Path_new:
                                        RouteFinal[tag_agent].insert(index_start,s)
                                        index_start+=1
                                    dis_list[tag_agent].remove(stage_tag)
                                    stage[tag_agent]-=1
                                    Flag_find=True
                                    break
                                #该Agent正在执行被影响的阶段:跳过下一个goal,直接规划到下下个goal的路径
                                if stage_tag == stage_now:
                                    if tag_agent==Agent_tag:
                                        Mission_stage[Agent_tag]+=1
                                        if Mission_stage[Agent_tag]==stage[Agent_tag]:
                                            Mission_compelete[Agent_tag]=1
                                            Flag_find=True
                                            break
                                        else:
                                            start=env_real.canvas.coords(env_real.Agent_rect[tag_agent])
                                            end=goal_coords[dis_list[tag_agent][stage_tag+1]]
                                            Path_new=RePlanning(end,start)
                                            index_start=RouteFinal[tag_agent].index(start)
                                            index_end=RouteFinal[tag_agent].index(end)
                                            del RouteFinal[tag_agent][index_start:index_end+1]
                                            for s in Path_new:
                                                RouteFinal[tag_agent].insert(index_start,s)
                                                index_start+=1
                                            Flag_find=True
                                            break
                                    #获取一条直达路径
                                    if stage_tag+1>=len(dis_list[tag_agent]):
                                        Mission_compelete[tag_agent]=1
                                        Flag_find=True
                                        break
                                    start=env_real.canvas.coords(env_real.Agent_rect[tag_agent])
                                    end=goal_coords[dis_list[tag_agent][stage_tag+1]]
                                    Path_new=RePlanning(end,start)
                                    #将获取的新路径插入到原来的路径之中
                                    #两个步骤。第一步首先删除原来在这两个状态之间的路径；第二部将新路径插入其中
                                    index_start=RouteFinal[tag_agent].index(start)
                                    index_end=RouteFinal[tag_agent].index(end)
                                    del RouteFinal[tag_agent][index_start:index_end+1]
                                    for s in Path_new:
                                        RouteFinal[tag_agent].insert(start,s)
                                        start+=1
                                    dis_list[tag_agent].remove(stage_tag)
                                    stage[tag_agent]-=1
                                    Flag_find=True
                                    break
                                #该Agent已经执行过被影响的阶段：不做任何处理
                                if stage_tag < stage_now:
                                    Flag_find=True
                                    break
                        if Flag_find:
                            break;            
            #任务丢失
            elif  Mission_Lost[Agent_tag] and not Mission_compelete[Agent_tag]:
                # #第一次初始化矫正的目标
                # if not Agnet_direction.setdefault(Agent_tag,0):
                #     dx=goal_real[dis_list[Agent_tag][Mission_stage[Agent_tag]]][0]-goal[dis_list[Agent_tag][Mission_stage[Agent_tag]]][0]
                #     dy=goal_real[dis_list[Agent_tag][Mission_stage[Agent_tag]]][1]-goal[dis_list[Agent_tag][Mission_stage[Agent_tag]]][1]
                #     Agnet_direction[Agent_tag]=[dx,dy]
                dx=(env_real.canvas.coords(env_real.goal[dis_list[Agent_tag][Mission_stage[Agent_tag]]])[0]-env_real.canvas.coords(env_real.Agent_rect[Agent_tag])[0])/25
                dy=(env_real.canvas.coords(env_real.goal[dis_list[Agent_tag][Mission_stage[Agent_tag]]])[1]-env_real.canvas.coords(env_real.Agent_rect[Agent_tag])[1])/25
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
                    goalComplete[str(env_real.canvas.coords(env_real.Agent_rect[Agent_tag]))]=1
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
                    #检查提前遭遇其他非预设目标
                    if env_real.canvas.coords(env_real.Agent_rect[Agent_tag]) in real_goal_coords:
                        state_next= env_real.canvas.coords(env_real.Agent_rect[Agent_tag])
                        goalComplete[str(state_next)]=1
                        #获取当前遇到的goal的标签
                        goal_tag=real_goal_coords.index(state_next)
                        #tag_agent是机器人的标签
                        Flag_find=False
                        for tag_agent in range(len(dis_list)):
                            #stage_tag是阶段的标签 
                            for stage_tag in range(len(dis_list[tag_agent])):
                                #找到该目标实际对应着哪个Agent的哪一个阶段的目标
                                if goal_tag==dis_list[tag_agent][stage_tag]:
                                    stage_now=Mission_stage[tag_agent]
                                    #该Agent还未执行到被影响的阶段：则直接对后半段执行重规划
                                    if stage_tag > stage_now:
                                        #获取一条直达路径
                                        start=goal_coords[dis_list[tag_agent][stage_tag-1]]
                                        #start=env_real.canvas.coords(env_real.goal[dis_list[tag_agent][stage_tag-1]])
                                        end=goal_coords[dis_list[tag_agent][stage_tag+1]]
                                        #end=env_real.canvas.coords(env_real.goal[dis_list[tag_agent][stage_tag+1]])
                                        Path_new=RePlanning(end,start)
                                        #将获取的新路径插入到原来的路径之中
                                        #两个步骤。第一步首先删除原来在这两个状态之间的路径；第二部将新路径插入其中
                                        index_start=RouteFinal[tag_agent].index(start)
                                        index_end=RouteFinal[tag_agent].index(end)
                                        del RouteFinal[tag_agent][index_start:index_end+1]
                                        for s in Path_new:
                                            RouteFinal[tag_agent].insert(index_start,s)
                                            index_start+=1
                                        dis_list[tag_agent].remove(stage_tag)
                                        stage[tag_agent]-=1
                                        Flag_find=True
                                        break
                                    #该Agent正在执行被影响的阶段:跳过下一个goal,直接规划到下下个goal的路径
                                    if stage_tag == stage_now:
                                        if tag_agent==Agent_tag:
                                            Mission_stage[Agent_tag]+=1
                                            if Mission_stage[Agent_tag]==stage[Agent_tag]:
                                                Mission_compelete[Agent_tag]=1
                                                Flag_find=True
                                                break
                                            else:
                                                start=env_real.canvas.coords(env_real.Agent_rect[tag_agent])
                                                end=goal_coords[dis_list[tag_agent][stage_tag+2]]
                                                Path_new=RePlanning(end,start)
                                                index_start=RouteFinal[tag_agent].index(start)
                                                index_end=RouteFinal[tag_agent].index(end)
                                                del RouteFinal[tag_agent][index_start:index_end+1]
                                                for s in Path_new:
                                                    RouteFinal[tag_agent].insert(index_start,s)
                                                    index_start+=1
                                                Flag_find=True
                                                break
                                        #获取一条直达路径
                                        if stage_tag+1>=len(dis_list[tag_agent]):
                                            Mission_compelete[tag_agent]=1
                                            Flag_find=True
                                            break
                                        start=env_real.canvas.coords(env_real.Agent_rect[tag_agent])
                                        end=goal_coords[dis_list[tag_agent][stage_tag+1]]
                                        Path_new=RePlanning(end,start)
                                        #将获取的新路径插入到原来的路径之中
                                        #两个步骤。第一步首先删除原来在这两个状态之间的路径；第二部将新路径插入其中
                                        index_start=RouteFinal[tag_agent].index(start)
                                        index_end=RouteFinal[tag_agent].index(end)
                                        del RouteFinal[tag_agent][index_start:index_end+1]
                                        for s in Path_new:
                                            RouteFinal[tag_agent].insert(start,s)
                                            start+=1
                                        dis_list[tag_agent].remove(stage_tag)
                                        stage[tag_agent]-=1
                                        Flag_find=True
                                        break
                                    #该Agent已经执行过被影响的阶段：不做任何处理
                                    if stage_tag < stage_now:
                                        Flag_find=True
                                        break
                            if Flag_find:
                                break;            
                    for each in next_direction:
                        print(list(each))
                        print(list(each+np.array(env_real.canvas.coords(env_real.Agent_rect[Agent_tag]))))
                        if list(each) not in DontGo[Agent_tag] \
                            and list(each+np.array(env_real.canvas.coords(env_real.Agent_rect[Agent_tag]))) not in env_real.Obs:
                           env_real.canvas.move(env_real.Agent_rect[Agent_tag],each[0],each[1])
                           env_real.canvas.move(env_real.Agent[Agent_tag],each[0],each[1])
                           env_real.update()
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
                :
                #and (around not in env.getGoal() or around == goal)
                 neighbors.append(around)

        return neighbors 

if __name__ == "__main__":

    #仿真环境和参数
    con=Config("config_5.ini")
    goal=eval(con.Maze_config["maze_goal"])
    goal_coords=[[25*(goal[i][0]-1)+2.5,25*(goal[i][1]-1)+2.5,25*(goal[i][0]-1)+22.5,25*(goal[i][1]-1)+22.5] for i in range(len(goal))]

    #实际环境配置文件
    con_real=Config("config_5.ini")
    Agent_num=eval(con_real.Agent_config["num"])
    Agent_start=eval(con_real.Agent_config["start"])
    Agent_list=[]
    for i in range(Agent_num):
        Agent_list.append(Agent(Agent_start[i],e_greedy=0.8))
    
    #真实环境
    goal_real=eval(con_real.Maze_config["maze_goal"])
    real_goal_coords=[[25*(goal_real[i][0]-1)+2.5,25*(goal_real[i][1]-1)+2.5,25*(goal_real[i][0]-1)+22.5,25*(goal_real[i][1]-1)+22.5] for i in range(len(goal_real))]    
    env_real = Maze(Agent_list,con_real)
    goalComplete=dict()
    for goal_set in goal:
        oval_center=[12.5,12.5] + np.array([25 * (goal_set[0]-1), 25 * (goal_set[1]-1)])
        env_real.canvas.create_oval(oval_center[0] - 5, oval_center[1] - 5,
                oval_center[0] + 5, oval_center[1] + 5,
                fill='red',outline = '')
    # for goalTag in range(len(env_real.goal)):
    #     goalComplete[goalTag]=0

    #读取仿真获得的数据
    with open("Route_1.data","rb") as inputfile:
        RouteFinal=pickle.load(inputfile)
    with open("Phermenon_1.data","rb") as inputfile:
        env_real.pheromone=pickle.load(inputfile)
    with open("dislist_1.data","rb") as inputfile:
        dis_list=pickle.load(inputfile)

    
    env_real.after(100, taskExecute)
    env_real.mainloop()
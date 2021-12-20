# import pickle
import numpy as np
from itertools import combinations

# a=[[1,2,3,4]]
# b=[1,2,3,4]
# if b in a:
#     print(1)

# # with open("test.data","rb") as input:
# #     a=pickle.load(input)
# # print(a)


# around=[[25,0,25,0],[-25,0,-25,0],[0,25,0,25],[0,-25,0,-25],\
#                     [25,25,25,25],[25,-25,25,-25],[-25,25,-25,25],[-25,-25,-25,-25]]
                    
# around2=[[0,0,0,0],[0,0,0,0],[0,25,0,25],[0,-25,0,-25],\
#                     [25,25,25,25],[25,-25,25,-25],[-25,25,-25,25],[-25,-25,-25,-25]]
# # a=[25,25,25,25]
# # c=np.array(around)+np.array(a)
# # print(c)
# # for i in itertools.combinations(around, 2):
# #     print(list(i))

# t=np.max(abs(np.array(around[0:5])-np.array(around2[0:5])),axis=1)

# print(np.count_nonzero(t>=25))

# import torch

# print(torch.__version__)
# print(torch.cuda.is_available())

import pandas as pd

# TwogoalCombine=list(combinations(range(5),2))
# S_Table=pd.DataFrame(columns=TwogoalCombine,dtype=np.float64)
# b=pd.Series([1]*len(TwogoalCombine),index=S_Table.columns,name="TEST")
# S_Table=S_Table.append(b)
# b=S_Table.apply(lambda x: x.sum()) 
# for a in b.nlargest(3).index:
#     print(a)
# print("nihao\n",S_Table)
# a=[1,2]
# b=[2,3]

max_n=[(0,1),(2,3),(1,3)]
Close_list=[]
Open_list=list(range(5))
for pair in max_n:
    #其中任意有一个经过处理
    if not pair[0] in Open_list or not pair[1] in Open_list:
        #1号没有经过处理
        if pair[0] in Open_list:
            Open_list.remove(pair[0])
            for cluster in Close_list:
                if pair[1] in cluster:
                    cluster.append(pair[0])
        #2号没有经过处理
        if pair[1] in Open_list:
            Open_list.remove(pair[1])
            for cluster in Close_list:
                if pair[0] in cluster:
                    cluster.append(pair[1])
        #二者都被处理过
        else:
            for cluster1 in Close_list:
                if pair[0] in cluster1:
                    for cluster2 in Close_list:
                        if pair[1] in cluster2:
                            Close_list.remove(cluster1)
                            Close_list.remove(cluster2)
                            Close_list.append(list(set(cluster1)|set(cluster2)))
                            break
                    break
    #二者都未经过处理
    else:
        Open_list.remove(pair[0])
        Open_list.remove(pair[1])
        Close_list.append([pair[0],pair[1]])

for remain in Open_list:
    Close_list.append([remain])
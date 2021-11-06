import pickle
import numpy as np

a=[[1,2,3,4]]
b=[1,2,3,4]
if b in a:
    print(1)

# with open("test.data","rb") as input:
#     a=pickle.load(input)
# print(a)
import numpy
import itertools

around=[[25,0,25,0],[-25,0,-25,0],[0,25,0,25],[0,-25,0,-25],\
                    [25,25,25,25],[25,-25,25,-25],[-25,25,-25,25],[-25,-25,-25,-25]]

a=[25,25,25,25]
c=np.array(around)+np.array(a)
print(c)
for i in itertools.permutations(around, len(around)):
    print(list(i))
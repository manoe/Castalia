#!/bin/python3

import random

masters=list(range(0,100))

for i in [10, 20, 30]:
    random.shuffle(masters)
    print('[Config p'+str(i)+']');
    for j in range(0,i):
        print('SN.node['+str(masters[j])+'].Application.isMaster = true');

print('[Config diag]')
for j in [0, 11, 22, 33, 44, 55, 66, 77, 88, 99]:
    print('SN.node['+str(j)+'].Application.isMaster = true');


#!/bin/python3

import st_data
import matplotlib.pyplot as plt
import numpy as np


avg=list()
std=list()
t_l=list()


for i in st_data.stat:
    if not int(i['t_l']) in t_l:
        t_l.append(int(i['t_l']))

for i in t_l:
    tmp_std=list()
    tmp_avg=list()
    for j in list(filter(lambda res: res['t_l']==str(i), st_data.stat)):
        tmp_std.append(float(j['std']))
        tmp_avg.append(float(j['avg']))
    avg.append(np.average(tmp_avg))
    std.append(np.average(tmp_std))

plt.errorbar(t_l,avg,std,capsize=3)
plt.show()


#!/bin/python3

import networkx as nx
import matplotlib.pyplot as plt
import graph
import pos
import neigh
import role
import numpy

import graph2
import pos2
import neigh2
import role2
import graph3
import pos3
import neigh3
import role3
import graph1
import pos1
import neigh1
import role1

fsize=7
G=nx.DiGraph()

G.add_edges_from(graph.edges)

print(list(G))

size=0;

for x in role.role:
    if role.role[x] == 'dead':
        G.remove_node(x)
        del pos.pos[x]
    else:
        size=size+1

edge_labels = dict([((n1, n2), d['path'])
                for n1, n2, d in G.edges(data=True)])

color_map = []

for i in list(G):
    if role.role[i] == 'Sink':
        color_map.append('tab:pink')
        print(i)
    elif role.role[i] == 'root':
        color_map.append('tab:brown')
    elif role.role[i] == 'sub-root':
        color_map.append('tab:cyan')
    elif role.role[i] == 'non-root':
        color_map.append('tab:blue')


plt.subplot(1,4,1)
plt.gca().set_title('(a)',loc='left')
nx.draw(G, pos.pos, node_color=color_map, with_labels=True)#, connectionstyle="arc3,rad=0.2")

nx.draw_networkx_edge_labels(G, pos.pos, edge_labels=edge_labels, label_pos=0.7,
                                             font_color='red', font_size=fsize)#, font_size=14)#, font_weight='bold')




########################################################


G=nx.DiGraph()

G.add_edges_from(graph1.edges)

print(list(G))

for x in role1.role:
    if role1.role[x] == 'dead':
        G.remove_node(x)
        del pos1.pos[x]


edge_labels = dict([((n1, n2), d['path'])
                            for n1, n2, d in G.edges(data=True)])

color_map = []

for i in list(G):
    if role1.role[i] == 'Sink':
        color_map.append('tab:pink')
        print(i)
    elif role1.role[i] == 'root':
        color_map.append('tab:brown')
    elif role1.role[i] == 'sub-root':
        color_map.append('tab:cyan')
    elif role1.role[i] == 'non-root':
        color_map.append('tab:blue')


plt.subplot(1,4,2)
plt.gca().set_title('(b)',loc='left')
nx.draw(G, pos1.pos, node_color=color_map,with_labels=True)#, connectionstyle="arc3,rad=0.2")

nx.draw_networkx_edge_labels(G, pos1.pos, edge_labels=edge_labels, label_pos=0.7,
                                             font_color='red', font_size=fsize)#, font_size=14)#, font_weight='bold')



#########################################################
G=nx.DiGraph()

G.add_edges_from(graph2.edges)


for x in role2.role:
    if role2.role[x] == 'dead':
        G.remove_node(x)
        del pos2.pos[x]


edge_labels = dict([((n1, n2), d['path'])
                            for n1, n2, d in G.edges(data=True)])

plt.subplot(1,4,3)
plt.gca().set_title('(c)',loc='left')

color_map=[]

for i in list(G):
    if role2.role[i] == 'Sink':
        color_map.append('tab:pink')
        print(i)
    elif role2.role[i] == 'root':
        color_map.append('tab:brown')
    elif role2.role[i] == 'sub-root':
        color_map.append('tab:cyan')
    elif role2.role[i] == 'non-root':
        color_map.append('tab:blue')



nx.draw(G, pos2.pos, node_color=color_map,with_labels=True)#, connectionstyle="arc3,rad=0.2")

nx.draw_networkx_edge_labels(G, pos2.pos, edge_labels=edge_labels, label_pos=0.7,
                                             font_color='red', font_size=fsize)#, font_size=14)#, font_weight='bold')



#nx.draw(G2,pos.pos,connectionstyle="arc3,rad=0.2")
#nx.draw(G, with_labels = True)
G=nx.DiGraph()

G.add_edges_from(graph3.edges)


for x in role3.role:
    if role3.role[x] == 'dead':
        G.remove_node(x)
        del pos3.pos[x]


edge_labels = dict([((n1, n2), d['path'])
                            for n1, n2, d in G.edges(data=True)])

plt.subplot(1,4,4)
plt.gca().set_title('(d)',loc='left')

color_map = []
for i in list(G):
    if role3.role[i] == 'Sink':
        color_map.append('tab:pink')
        print(i)
    elif role3.role[i] == 'root':
        color_map.append('tab:brown')
    elif role3.role[i] == 'sub-root':
        color_map.append('tab:cyan')
    elif role3.role[i] == 'non-root':
        color_map.append('tab:blue')


nx.draw(G, pos3.pos,node_color=color_map, with_labels=True)#, connectionstyle="arc3,rad=0.2")

nx.draw_networkx_edge_labels(G, pos3.pos, edge_labels=edge_labels, label_pos=0.7,
                                             font_color='red', font_size=fsize) #, font_weight='bold')



#nx.draw(G2,pos.pos,connectionstyle="arc3,rad=0.2")
#nx.draw(G, with_labels = True)
plt.tight_layout()
plt.show()


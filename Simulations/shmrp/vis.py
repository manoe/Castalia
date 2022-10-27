#!/bin/python3

import networkx as nx
import matplotlib.pyplot as plt
import graph
import pos
import role
import numpy

G=nx.DiGraph()

G.add_edges_from(graph.edges)

print(list(G))

for x in role.role:
    if role.role[x] == 'dead':
        G.remove_node(x)
        del pos.pos[x]


edge_labels = dict([((n1, n2), d['path'])
                            for n1, n2, d in G.edges(data=True)])


color_map = []

for i in list(G):
    if role.role[i] == 'central':
        color_map.append('tab:pink')
    elif role.role[i] == 'internal':
        color_map.append('tab:brown')
    elif role.role[i] == 'border':
        color_map.append('tab:cyan')
    elif role.role[i] == 'external':
        color_map.append('tab:blue')


#plt.subplot(1,2,1)
nx.draw(G, pos.pos, node_color=color_map, with_labels=True)#, connectionstyle="arc3,rad=0.2")

nx.draw_networkx_edge_labels(G, pos.pos, edge_labels=edge_labels, label_pos=0.7,
                                             font_color='red')#, font_size=14)#, font_weight='bold')

print('Number of edges: '+str(G.number_of_edges()))

for x in role.role:
    if role.role[x] == 'sink':
        G.remove_node(x)

print('Number of edges: '+str(G.number_of_edges()))

print('Avg number of edges: '+str(G.number_of_edges()/G.number_of_nodes()))

#nx.draw(G2,pos.pos,connectionstyle="arc3,rad=0.2")
#nx.draw(G, with_labels = True)

plt.show()


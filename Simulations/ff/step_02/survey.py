#!/bin/python3

import networkx as nx
import matplotlib.pyplot as plt
import graph
import pos
import neigh
import role
import numpy

G=nx.DiGraph()

G.add_edges_from(graph.edges)

#edge_labels = dict([((n1, n2), d['path'])
#                            for n1, n2, d in G.edges(data=True)])
#
#plt.subplot(1,2,1)
#nx.draw(G, pos.pos, with_labels=True)#, connectionstyle="arc3,rad=0.2")
#
#nx.draw_networkx_edge_labels(G, pos.pos, edge_labels=edge_labels, label_pos=0.7,
#                                             font_color='red')#, font_size=14)#, font_weight='bold')
#
#plt.subplot(1,2,2)
#G2=nx.DiGraph();
#G2.add_edges_from(neigh.neighs)
#
#print('Number of edges: '+str(G.number_of_edges()))

for x in role.role:
    if role.role[x] == 'root' or role.role[x] == 'sink':
        G.remove_node(x)

edge_list=list()

for x in list(G):
    e=0;
    for y in list(G):
        e+=G.number_of_edges(x,y)
    edge_list.append(e)

print('\'std\':\''+str(numpy.std(edge_list))+'\',')
print('\'avg\':\''+str(numpy.average(edge_list))+'\'')


#print('Number of edges: '+str(G.number_of_edges()))
#
#print('Avg number of edges: '+str(G.number_of_edges()/G.number_of_nodes()))
#
#nx.draw(G2,pos.pos,connectionstyle="arc3,rad=0.2")
#nx.draw(G, with_labels = True)

#plt.show()


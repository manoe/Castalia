#!/bin/python3

import networkx as nx
import matplotlib.pyplot as plt
import pos
import role
import numpy

G=nx.Graph()


for i in range(0,99):
    if(i%10 != 9):
        G.add_edge(str(i+1),str(i))
        if(i < 90):
            G.add_edge(str(i+11),str(i))

    if(i%10 != 0 and i < 89):
        G.add_edge(str(i+9),str(i))
    if(i < 90):
        G.add_edge(str(i+10),str(i))

G.remove_nodes_from([str(0), str(1), str(10),str(11)])

G.remove_edges_from([('20','21'), ('21','22'), ('12','22'),('21','12'),('12','2') ])

sub_root=['20', '2', '21', '22', '12']

trees = []

free_nodes = list(G)

for i in sub_root:
    dg=nx.DiGraph()
    dg.add_node(i)
    trees.append(dg)
    free_nodes.remove(i)

while free_nodes:
#    trees.sort(key=nx.dag_longest_path_length)
#    for i in trees:
    for i in trees:
        a=[];
        for j in list(i):
            for k in list(G.neighbors(j)):
                if k in free_nodes:
                    tmp_g=nx.DiGraph(i)
                    tmp_g.add_edge(k,j)
                    a.append((k,j,nx.dag_longest_path_length(tmp_g)))
        if a:
            min_edge=a[0];
            for l in a:
                if l[2] < min_edge[2]:
                    min_edge=l;
            i.add_edge(min_edge[0],min_edge[1])
            free_nodes.remove(min_edge[0])

T=nx.DiGraph()

for i in trees:
    T=nx.compose(T,i)

T.add_edges_from([('1','0'),('11','0'),('10','0'),('20','10'),('21','11'),('22','11'),('2','1'),('12','1')])

for i in trees:
    for j in sub_root:
        if str(j) in i:
            print("Sub-root: "+str(j))
    print("Longest path: "+str(nx.dag_longest_path_length(i)))
    print("Number of nodes: "+str(len(i)))
#G.add_edges_from(graph.edges)

f = open("static_route.ini","w")

for i in trees:
    path='NaN';
    for k in sub_root:
        if k in list(i):
            path=k;
    for j in list(i):
        if list(i[j]):
            f.write('SN.node['+j+'].Communication.Routing.static_path = '+path+'\n')
            f.write('SN.node['+j+'].Communication.Routing.static_next_hop = '+list(i[j])[0]+'\n')
f.write('SN.node[20].Communication.Routing.static_path = 20\n')
f.write('SN.node[20].Communication.Routing.static_next_hop = 10\n')
f.write('SN.node[21].Communication.Routing.static_path = 21\n')
f.write('SN.node[21].Communication.Routing.static_next_hop = 10\n')
f.write('SN.node[22].Communication.Routing.static_path = 22\n')
f.write('SN.node[22].Communication.Routing.static_next_hop = 11\n')
f.write('SN.node[12].Communication.Routing.static_path = 12\n')
f.write('SN.node[12].Communication.Routing.static_next_hop = 11\n')
f.write('SN.node[2].Communication.Routing.static_path = 2\n')
f.write('SN.node[2].Communication.Routing.static_next_hop = 1\n')

f.write('SN.node[10].Communication.Routing.static_path = 0\n')
f.write('SN.node[10].Communication.Routing.static_next_hop = 0\n')
f.write('SN.node[11].Communication.Routing.static_path = 0\n')
f.write('SN.node[11].Communication.Routing.static_next_hop = 0\n')
f.write('SN.node[1].Communication.Routing.static_path = 0\n')
f.write('SN.node[1].Communication.Routing.static_next_hop = 0\n')


f.close()

#for x in role.role:
#    if role.role[x] == 'dead':
#        G.remove_node(x)


#edge_labels = dict([((n1, n2), d['path'])
#                            for n1, n2, d in G.edges(data=True)])

#plt.subplot(1,2,1)
#nx.draw(G, pos.pos,with_labels=True )#, connectionstyle="arc3,rad=0.2")
list(G)
#nx.draw_networkx_edge_labels(G, pos.pos, edge_labels=edge_labels, label_pos=0.7,
#                                             font_color='red')#, font_size=14)#, font_weight='bold')

plt.subplot(1,2,2)
nx.draw(T, pos.pos)
#G2=nx.DiGraph();
#G2.add_edges_from(neigh.neighs)
#
#print('Number of edges: '+str(G.number_of_edges()))
#
#for x in role.role:
#    if role.role[x] == 'sink':
#        G.remove_node(x)
#
#print('Number of edges: '+str(G.number_of_edges()))
#
#print('Avg number of edges: '+str(G.number_of_edges()/G.number_of_nodes()))
#
#nx.draw(B,pos.pos)#,connectionstyle="arc3,rad=0.2")
##nx.draw(G, with_labels = True)
#
plt.show()



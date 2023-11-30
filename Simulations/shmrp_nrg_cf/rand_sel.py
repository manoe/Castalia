#!/bin/python3

import yaml
import random
import argparse
import operator

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='rand_sel', description='Select nodes randomly, yeah', epilog=':-(')
    parser.add_argument('-n', '--node_num', dest='node_num', action='store', type=int, default=4,
                        help='how many nodes to select')
    parser.add_argument('-s', '--seed', dest='seed', type=int, default=0)
    parser.add_argument('-p', '--prefer_hop', dest='phop', action='store_true')
    parser.add_argument('-f', '--file', dest='file', help='file with loc_pdr')
    args = parser.parse_args()

    loader = yaml.safe_load(open(args.file, 'r'))

    nodes = []
    random.seed(args.seed)

    if args.phop:
        n_prio_list = {i['node']: i['hop'] for i in loader if i['role'] == 'external'}
        hops = list(set(n_prio_list.values()))
        hops.sort()
        for i in hops:
            nh_list = [j for j in n_prio_list.keys() if n_prio_list[j] == i]
            random.shuffle(nh_list)
            nodes.extend(nh_list)
    else:
        nodes = [i['node'] for i in loader if i['role'] == 'external']
        random.shuffle(nodes)



    for i in nodes[0:args.node_num]:
        print(i)

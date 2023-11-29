#!/bin/python3

import random
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='rand.py', description='Merse twister', epilog=':-(')
    parser.add_argument('-n', '--node', dest='node', type=int)
    parser.add_argument('-r', '--ratio', dest='ratio', type=int)
    parser.add_argument(dest='seed', type=int)

    args = parser.parse_args()

    random.seed(args.seed)
    nodes = [ x for x in range(1, args.node) ]
    random.shuffle(nodes)

    if args.ratio == 100:
        for i in range(args.node):
            print(i)
    else:
        for i in range(int(args.node*args.ratio/100)):
            print(str(nodes[i]))


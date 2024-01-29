#!/bin/python3

import random
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog='rand.py', description='Merse twister', epilog=':-(')
    parser.add_argument('-i', '--iteration', dest='iter', type=int)
    parser.add_argument(dest='seed', type=int)

    args = parser.parse_args()

    random.seed(args.seed)
    for i in range(args.iter):
        print(random.getrandbits(32))

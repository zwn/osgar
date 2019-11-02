"""
  Extract position with topics statistic from AWS ROS_INFO output
"""
import sys
import os
from itertools import chain


def aws2info(filename, outname):
    KEY = 'Python3: stdout '
    with open(filename) as f:
        for line in f:
            if KEY in line and line.strip().endswith(']'):
                print(line[line.index(KEY)+len(KEY):].strip())


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename', help='AWS ROS recorded log file')
    args = parser.parse_args()

    outname = os.path.join(os.path.dirname(args.filename), 'info.txt')
    aws2info(args.filename, outname=outname)

# vim: expandtab sw=4 ts=4

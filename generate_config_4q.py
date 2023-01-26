#!/usr/bin/env python

import numpy as np
import pandas as pd
from functools import *
from itertools import *
from pathlib import Path
import santatools


origin = [(64, 0), (-32, 0), (-16, 0), (-8, 0), (-4, 0), (-2, 0), (-1, 0), (-1, 0)]


def init_config_map():
    base = [(64, 0), (0, 32), (0, 16), (0, 8), (0, 4), (0, 2), (0, 1), (0, 1)]
    vector_len = [max(abs(v[0]), abs(v[1])) for v in origin]
    x2config = dict()
    x2config[64] = base

    vectoridx = 2
    x = 64
    config = base.copy()
    for i in range(64):
        if i%2 == 0:
            config[1] = (config[1][0]+1, config[1][1])
        else:
            config[vectoridx] = (config[vectoridx][0]+1, config[vectoridx][1])
            if config[vectoridx][0] == vector_len[vectoridx]:
                vectoridx += 1
        x += 1
        x2config[x] = config.copy()

    vectoridx = 2
    x = 64
    config = base.copy()
    for i in range(64):
        if i%2 == 0:
            config[vectoridx] = (config[vectoridx][0]-1, config[vectoridx][1])
            if config[vectoridx][0] == -vector_len[vectoridx]:
                vectoridx += 1
        else:
            config[1] = (config[1][0]-1, config[1][1])
        x -= 1
        x2config[x] = config.copy()
    return x2config


def get_config_ur(x2config, point):
    config = x2config[point[0]].copy()
    # 128->64, 0->-64
    config[0] = (64, point[1] - 64)
    assert santatools.get_position(config) == point
    return config 


def get_initial_path():
    path = [origin]

    # cartesian: (0, 1) 
    config = santatools.rotate(path[-1], 7, -1)
    path.append(config)

    # cartesian: (1, 0) 
    config = santatools.rotate(path[-1], 7, -1)
    config = santatools.rotate(config, 0, -1)
    path.append(config)

    initial_rotate_steps = []
    for vector in origin:
        initial_rotate_steps.append(abs(vector[0]))

    # cartesian: (1, 63) 
    # rotate from link 32 till the second to last
    vector_idx = 1
    # sum([1,2,4,8,16,32])
    for i in range(63):
        config = santatools.rotate(path[-1], vector_idx, -1)
        path.append(config)
        initial_rotate_steps[vector_idx] -= 1
        if initial_rotate_steps[vector_idx] == 0:
            vector_idx += 1
    
    # cartesian: (1, 64) 
    config = santatools.rotate(config, 0, 1)
    path.append(config)

    return path


def load_tour(fname, idx2ary):
    points = []
    f = open(fname)
    start = False
    for line in f:
        if line == 'TOUR_SECTION\n':
            start = True
            continue
        if line == '-1\n':
            break
        if not start:
            continue
        n = int(line.rstrip('\n'))
        points.append(santatools.ary_to_cartesian(idx2ary[n]))
    f.close()
    return points


def map_point(point, quadrant):
    # ur: 2, 1
    # lr: 1, -2
    # ll: -2, -1
    # ul: -1, 2
    if quadrant == 'lr':
        point = (-point[1], point[0])
    elif quadrant == 'll':
        point = (-point[0], -point[1])
    elif quadrant == 'ul':
        point = (point[1], -point[0])
    return point


def rev_map_config(config, quadrant):
    # ur: 2, 1
    # lr: 1, -2
    # ll: -2, -1
    # ul: -1, 2
    for i, v in enumerate(config):
        if quadrant == 'lr':
            config[i] = (v[1], -v[0])
        elif quadrant == 'll':
            config[i] = (-v[0], -v[1])
        elif quadrant == 'ul':
            config[i] = (-v[1], v[0])
    return config


def generate_config_common(fname, x2config, idx2ary, quadrant):
    points = load_tour(fname, idx2ary)
    path = []
    for point in points:
        point = map_point(point, quadrant)
        config = get_config_ur(x2config, point)
        config = rev_map_config(config, quadrant)
        path.append(config)
    return path


def main():
    x2config = init_config_map()
    path = get_initial_path()

    tours = [
            ('LKH/tour_ur', 'ur'),
            ('LKH/tour_lr', 'lr'),
            ('LKH/tour_ll', 'll'),
            ('LKH/tour_ul', 'ul'),
    ]

    for fname, quadrant in tours:
        idx2ary, _ = santatools.get_idxary_map_4q(quadrant)
        path.extend(generate_config_common(fname, x2config, idx2ary, quadrant))

    path.extend(santatools.get_path_to_configuration(path[-1], origin)[1:])

    df_image = pd.read_csv('download/image.csv')
    image = santatools.df_to_image(df_image)

    cost = santatools.total_cost(path, image)
    print('cost: %f' % cost)
    cost_reconfig = santatools.total_cost_reconfig(path, image)
    print('cost_reconfig: %f, cost_image: %f' % (cost_reconfig, cost - cost_reconfig))

    santatools.check_path_valid(path)

    santatools.save_submission(path, 'sub_4q.csv')


if __name__ == '__main__':
    main()

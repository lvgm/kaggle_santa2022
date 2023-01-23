#!/usr/bin/env python

import numpy as np
import pandas as pd
from functools import *
from itertools import *
from pathlib import Path
import santatools


origin = [(64, 0), (-32, 0), (-16, 0), (-8, 0), (-4, 0), (-2, 0), (-1, 0), (-1, 0)]


# Functions to map between cartesian coordinates and array indexes
def cartesian_to_array(x, y, shape):
    m, n = shape[:2]
    i = (n - 1) // 2 - y
    j = (n - 1) // 2 + x
    if i < 0 or i >= m or j < 0 or j >= n:
        raise ValueError("Coordinates not within given dimensions.")
    return i, j


def array_to_cartesian(i, j, shape):
    m, n = shape[:2]
    if i < 0 or i >= m or j < 0 or j >= n:
        raise ValueError("Coordinates not within given dimensions.")
    y = (n - 1) // 2 - i
    x = j - (n - 1) // 2
    return x, y


# Functions to map an image between array and record formats
def image_to_dict(image):
    image = np.atleast_3d(image)
    kv_image = {}
    for i, j in product(range(len(image)), repeat=2):
        kv_image[array_to_cartesian(i, j, image.shape)] = tuple(image[i, j])
    return kv_image


def df_to_image(df):
    side = int(len(df) ** 0.5)  # assumes a square image
    return df.set_index(['x', 'y']).to_numpy().reshape(side, side, -1)


def get_position(config):
    return reduce(lambda p, q: (p[0] + q[0], p[1] + q[1]), config, (0, 0))


def rotate_link(vector, direction):
    x, y = vector
    if direction == 1:  # counter-clockwise
        if y >= x and y > -x:
            x -= 1
        elif y > x and y <= -x:
            y -= 1
        elif y <= x and y < -x:
            x += 1
        else:
            y += 1
    elif direction == -1:  # clockwise
        if y > x and y >= -x:
            x += 1
        elif y >= x and y < -x:
            y += 1
        elif y < x and y <= -x:
            x -= 1
        else:
            y -= 1
    return (x, y)


def rotate(config, i, direction):
    config = config.copy()
    config[i] = rotate_link(config[i], direction)
    return config


# Functions to compute the cost function

# Cost of reconfiguring the robotic arm: the square root of the number of links rotated
def reconfiguration_cost(from_config, to_config):
    nlinks = len(from_config)
    diffs = np.abs(np.asarray(from_config) - np.asarray(to_config)).sum(axis=1)
    return np.sqrt(diffs.sum())


# Cost of moving from one color to another: the sum of the absolute change in color components
def color_cost(from_position, to_position, image, color_scale=3.0):
    return np.abs(image[to_position] - image[from_position]).sum() * color_scale


# Total cost of one step: the reconfiguration cost plus the color cost
def step_cost(from_config, to_config, image):
    from_position = cartesian_to_array(*get_position(from_config), image.shape)
    to_position = cartesian_to_array(*get_position(to_config), image.shape)
    return (
        reconfiguration_cost(from_config, to_config) +
        color_cost(from_position, to_position, image)
    )


# Total cost of one step: the reconfiguration cost
def step_cost_reconfig(from_config, to_config, image):
    from_position = cartesian_to_array(*get_position(from_config), image.shape)
    to_position = cartesian_to_array(*get_position(to_config), image.shape)
    return (
        reconfiguration_cost(from_config, to_config)
    )


def get_direction(u, v):
    """Returns the sign of the angle from u to v."""
    direction = np.sign(np.cross(u, v))
    if direction == 0 and np.dot(u, v) < 0:
        direction = 1
    return direction


# compress a path between two points
def compress_path(path):
    r = [[] for _ in range(8)]
    for p in path:
        for i in range(8):
            if len(r[i]) == 0 or r[i][-1] != p[i]:
                r[i].append(p[i])
    mx = max([len(x) for x in r])
    
    for rr in r:
        while len(rr) < mx:
            rr.append(rr[-1])
    r = list(zip(*r))
    for i in range(len(r)):
        r[i] = list(r[i])
    return r


def get_path_to_configuration(from_config, to_config):
    path = [from_config]
    config = from_config.copy()
    while config != to_config:
        for i in range(len(config)):
            config = rotate(config, i, get_direction(config[i], to_config[i]))
        path.append(config)
    assert path[-1] == to_config
    return path


# Compute total cost of path over image
def total_cost(path, image):
    return reduce(
        lambda cost, pair: cost + step_cost(pair[0], pair[1], image),
        zip(path[:-1], path[1:]),
        0,
    )

# Compute reconfiguration cost of path
def total_cost_reconfig(path, image):
    return reduce(
        lambda cost, pair: cost + step_cost_reconfig(pair[0], pair[1], image),
        zip(path[:-1], path[1:]),
        0,
    )


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
    assert get_position(config) == point
    return config 


def ary_to_cartesian(point):
    return (point[1]-128, 128-point[0])


def get_initial_path():
    path = [origin]

    # cartesian: (0, 1) 
    config = rotate(path[-1], 7, -1)
    path.append(config)

    # cartesian: (1, 0) 
    config = rotate(path[-1], 7, -1)
    config = rotate(config, 0, -1)
    path.append(config)

    initial_rotate_steps = []
    for vector in origin:
        initial_rotate_steps.append(abs(vector[0]))

    # cartesian: (1, 63) 
    # rotate from link 32 till the second to last
    vector_idx = 1
    # sum([1,2,4,8,16,32])
    for i in range(63):
        config = rotate(path[-1], vector_idx, -1)
        path.append(config)
        initial_rotate_steps[vector_idx] -= 1
        if initial_rotate_steps[vector_idx] == 0:
            vector_idx += 1
    
    # cartesian: (1, 64) 
    config = rotate(config, 0, 1)
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
        points.append(ary_to_cartesian(idx2ary[n]))
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


def save_submission(path, fname):
    def config_to_string(config):
        return ';'.join([' '.join(map(str, vector)) for vector in config])

    submission = pd.Series(
        [config_to_string(config) for config in path],
        name="configuration",
    )
    submission.to_csv(fname, index=False)


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
        idx2ary, _ = santatools.get_idxary_map(quadrant)
        path.extend(generate_config_common(fname, x2config, idx2ary, quadrant))

    # Now make sure we end at the "origin" configuration
    path.extend(get_path_to_configuration(path[-1], origin)[1:])

    df_image = pd.read_csv('download/image.csv')
    image = df_to_image(df_image)

    cost = total_cost(path, image)
    print('cost: %f' % cost)
    cost_reconfig = total_cost_reconfig(path, image)
    print('cost_reconfig: %f, cost_image: %f' % (cost_reconfig, cost - cost_reconfig))

    santatools.check_path_valid(path)

    save_submission(path, 'sub_4q.csv')


if __name__ == '__main__':
    main()

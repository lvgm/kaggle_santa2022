#!/usr/bin/env python

import numpy as np
import pandas as pd
from functools import *
from itertools import *
from pathlib import Path
import random
import santatools


origin = [(64, 0), (-32, 0), (-16, 0), (-8, 0), (-4, 0), (-2, 0), (-1, 0), (-1, 0)]


def rotate(config, i, direction):
    config = config.copy()
    config[i] = santatools.rotate_link(config[i], direction)
    return config


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


def get_path_to_point(config, point):
    """Find a path of configurations to `point` starting at `config`."""
    path = [config]
    # Rotate each link, starting with the largest, until the point can
    # be reached by the remaining links. The last link must reach the
    # point itself.
    for i in range(len(config)):
        link = config[i]
        base = santatools.get_position(config[:i])
        relbase = (point[0] - base[0], point[1] - base[1])
        position = santatools.get_position(config[:i+1])
        relpos = (point[0] - position[0], point[1] - position[1])
        radius = reduce(lambda r, link: r + max(abs(link[0]), abs(link[1])), config[i+1:], 0)
        # Special case when next-to-last link lands on point.
        if radius == 1 and relpos == (0, 0):
            config = rotate(config, i, 1)
            if santatools.get_position(config) == point:  # Thanks @pgeiger
                path.append(config)
                break
            else:
                continue
        while np.max(np.abs(relpos)) > radius:
            direction = get_direction(link, relbase)
            config = rotate(config, i, direction)
            path.append(config)
            link = config[i]
            base = santatools.get_position(config[:i])
            relbase = (point[0] - base[0], point[1] - base[1])
            position = santatools.get_position(config[:i+1])
            relpos = (point[0] - position[0], point[1] - position[1])
            radius = reduce(lambda r, link: r + max(abs(link[0]), abs(link[1])), config[i+1:], 0)
    assert santatools.get_position(path[-1]) == point
    path = compress_path(path)
    return path


def find_duplicate_points(path):
    duplicate_points = {}
    for c in path:
        p = santatools.get_position(c)
        if p != (0,0):
            duplicate_points[p] = duplicate_points.get(p, 0) + 1
    return duplicate_points
    

def vector_diff_one(path):
    for i in range(len(path) - 1):
        for c0, c1 in zip(path[i], path[i+1]):
            if abs(c0[0] - c1[0]) + abs(c0[1] - c1[1]) > 1:
                return False
    return True


def run_remove(path):
    print("-- run remove --")
    print(f"Current length: {len(path)}")
    
    duplicate_points = find_duplicate_points(path)

    i = len(path) - 2
    while i >= 0 :
        local_p = path[i:i+3]
        p = santatools.get_position(local_p[1])
        new_local_p = compress_path(local_p)
        if vector_diff_one(new_local_p) and duplicate_points.get(p, 0) > 1 and len(new_local_p) < 3:
            path = path[:i+1] + path[i+2:]
            duplicate_points[p] -= 1
        i -= 1
    print(f"New length: {len(path)}")
    return path


def load_tour(fname):
    idx2ary, ary2idx = santatools.get_idxary_map_full()
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


def config_to_string(config):
    return ';'.join([' '.join(map(str, vector)) for vector in config])


'''
[[1,2], [3,4]] => ((0,2), (3,4))
'''
def ll2t(l):
    return tuple([tuple(x) for x in l])


'''
((1,2), (3,4)) => [[1,2], [3,4]]
'''
def t2ll(t):
    return [list(x) for x in t]


def get_move_link_num(p0, p1):
    return abs(p0[0] - p1[0]) + abs(p0[1] - p1[1])


# include as many states of the longest arm as possible
def sample_link64(configs, pool_size):
    if len(configs) <= pool_size:
        return configs

    pos2configs = dict()
    for config in configs:
        if config[0] not in pos2configs:
            pos2configs[config[0]] = [config]
        else:
            pos2configs[config[0]].append(config)
    n = pool_size//len(pos2configs)

    res = set()
    for k, v in pos2configs.items():
        for c in v[:n]:
            res.add(c)
    if len(res) < pool_size:
        for c in configs:
            if c not in res:
                res.add(c)
                if len(res) == pool_size:
                    break
    return res


def get_next_1_move_configs(configs, point, pool_size):
    next_configs = set()
    bt = dict()
    for config in configs:
        for i in range(len(origin)): #for each arm link
            for d in [-1,1]: #for each direction
                # Rotate link and get new position and vertical displacement:
                config2 = rotate(t2ll(config), i, d)
                if santatools.get_position(config2) == point:
                    config2 = ll2t(config2)
                    next_configs.add(config2)
                    bt[config2] = config


    if len(next_configs) > pool_size:
        next_configs = sample_link64(next_configs, pool_size)

    result_bt = dict()
    for config in next_configs:
        result_bt[config] = bt[config]
    return result_bt


def get_next_2_move_configs(configs, point, pool_size):
    next_configs = set()
    bt = dict()
    for config in configs:
        for i in range(len(origin)-1):
            for d1 in [-1,1]:
                for j in range(i+1,len(origin)):
                    for d2 in [-1,1]:
                        # Rotate two separate links, get position and vertical displacement:
                        config2 = rotate(t2ll(config), i, d1)
                        config2 = rotate(config2, j, d2)
                        if santatools.get_position(config2) == point:
                            config2 = ll2t(config2)
                            next_configs.add(config2)
                            bt[config2] = config

    if len(next_configs) > pool_size:
        #next_configs = random.sample(list(next_configs), pool_size)
        next_configs = sample_link64(next_configs, pool_size)

    result_bt = dict()
    for config in next_configs:
        result_bt[config] = bt[config]
    return result_bt


def get_next_3_move_configs(configs, point, pool_size):
    next_configs = set()
    bt = dict()
    for config in configs:
        for i in range(len(origin)-2):
            for d1 in [-1,1]:
                for j in range(i+1,len(origin)-1):
                    for d2 in [-1,1]:
                        for k in range(j+1,len(origin)):
                            for d3 in [-1,1]:
                                # Rotate two separate links, get position and vertical displacement:
                                config2 = rotate(t2ll(config), i, d1)
                                config2 = rotate(config2, j, d2)
                                config2 = rotate(config2, k, d3)
                                if santatools.get_position(config2) == point:
                                    config2 = ll2t(config2)
                                    next_configs.add(config2)
                                    bt[config2] = config

    if len(next_configs) > pool_size:
        next_configs = sample_link64(next_configs, pool_size)

    result_bt = dict()
    for config in next_configs:
        result_bt[config] = bt[config]
    return result_bt


def get_next_n_move_path(configs, p, pool_size, image):
    min_len = 1000
    best_paths = []
    for config in configs:
        path = get_path_to_point(t2ll(config), p)
        if len(path) < min_len:
            min_len = len(path)
            best_paths = [path]
        elif len(path) == min_len:
            best_paths.append(path)

    best_paths.sort(key = lambda path: santatools.total_cost(path, image))
    target_configs = set()
    target_size = 100
    bt_list = [(dict(),-1) for i in range(min_len-1)]
    for path in best_paths:
        config = ll2t(path[-1])
        if config not in target_configs:
            target_configs.add(config)
            for j in range(len(path)-1, 0, -1):
                k = ll2t(path[j])
                v = ll2t(path[j-1])
                bt_list[j-1][0][k] = v
            if len(target_configs) == target_size:
                break
    print('get_next_n_move_path(), input configs: %d output len %d' % (len(configs), min_len))
    return bt_list


def get_path_from_backlink(configs_path):
    print('get_path_from_backlink(): input len %d' % len(configs_path))
    path = []
    config = list(configs_path[-1][0].keys())[0]
    for i in range(len(configs_path)-1, -1, -1):
        path.append(config)
        config = configs_path[i][0][config]
    assert config == None
    path.reverse()
    path = [t2ll(x) for x in path]
    print('get_path_from_backlink(): output len %d' % len(path))
    return path


'''
beam search with go back retry with larger pool size
'''
def get_path(points, image):
    path = [origin]
    configs = {ll2t(path[-1]): None}
    pool_size_normal = 50
    pool_size_retry = 1000
    back_cnt = 800
    pool_size = pool_size_normal
    configs_path = [(configs, 0)]  # last config and next point to reach
    second_time = False
    retry_i = 100000
    print('pool_size_normal:', pool_size_normal)
    print('pool_size_retry:', pool_size_retry)
    print('back_cnt:', back_cnt)
    i = 1
    while i < len(points):
        #if i == 100: break
        last_point = points[i-1]
        n_link = get_move_link_num(points[i-1], points[i])
        if n_link == 1:
            next_configs = get_next_1_move_configs(configs, points[i], pool_size)
        elif n_link == 2:
            next_configs = get_next_2_move_configs(configs, points[i], pool_size)
        elif n_link == 3:
            next_configs = get_next_3_move_configs(configs, points[i], pool_size)

        if i > retry_i:
            second_time = False
            retry_i = 100000
            pool_size = pool_size_normal

        if len(next_configs) > 0:
            configs = next_configs
            configs_path.append((configs, i))
        else:
            print('no near next: ' + str(i) + ': ' + str(last_point) + ' => ' + str(points[i]))
            if back_cnt == 0 or second_time:
                print('second time: %d' % i)
                configs_path.extend(get_next_n_move_path(configs, points[i], pool_size, image))
                configs = configs_path[-1][0]
                retry_i = 100000
                second_time = False
                pool_size = pool_size_normal
            else:
                retry_i = i
                second_time = True
                pool_size = pool_size_retry
                j = len(configs_path)-1
                cnt = 0
                while j > 0 and configs_path[j-1][1] != -1:
                    j -= 1
                    cnt += 1
                    configs_path.pop()
                    if cnt == back_cnt:
                        break
                configs, i = configs_path[-1]
                print('back %d to %d' % (cnt, i))


        if i%1000 == 0:
            print('%d #config: %d' % (i, len(configs)), flush=True)
        i += 1

    path.extend(get_path_from_backlink(configs_path)[1:])
    return path


def split_points(points):
    for i in range(len(points)):
        if points[i] == (128, 128):
            idx = i
            break
    points_forward = points[:idx]
    points_backward = points[idx:]
    points_backward.reverse()
    print('split_points() to ', len(points_forward), len(points_backward))
    return points_forward, points_backward


def main():
    random.seed(1)
    df_image = pd.read_csv('download/image.csv')
    image = santatools.df_to_image(df_image)
    points = load_tour('LKH/tour_full')
    points.append((0,0))

    points_forward, points_backward = split_points(points)
    path_forward = get_path(points_forward, image)
    path_backward = get_path(points_backward, image)
    path_backward.reverse()
    path = path_forward + path_backward

    print(len(path))
    path = run_remove(path)
    path = run_remove(path)

    cost = santatools.total_cost(path, image)
    print('cost: %f' % cost)
    cost_reconfig = santatools.total_cost_reconfig(path, image)
    print('cost_reconfig: %f, cost_image: %f' % (cost_reconfig, cost - cost_reconfig))

    santatools.save_submission(path, 'sub_full.csv')

    santatools.check_path_valid(path)


if __name__ == '__main__':
    main()

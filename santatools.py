#!/usr/bin/env python

from functools import *
import numpy as np


def get_order_ary_map(imin, imax, jmin, jmax, blocked, start_ary, end_ary):
    idx2ary = dict()
    ary2idx = dict()
    idx = 1
    for i in range(imin, imax+1):
        for j in range(jmin, jmax+1):
            if not blocked(i, j):
                idx2ary[idx] = (i,j)
                ary2idx[(i,j)] = idx
                idx += 1

    start_idx = ary2idx[start_ary]

    idx2ary[start_idx] = idx2ary[1]
    ary2idx[idx2ary[1]] = start_idx
    idx2ary[1] = start_ary
    ary2idx[start_ary] = 1

    end_idx = ary2idx[end_ary]

    idx2ary[end_idx] = idx2ary[len(idx2ary)]
    ary2idx[idx2ary[len(idx2ary)]] = end_idx
    idx2ary[len(idx2ary)] = end_ary
    ary2idx[end_ary] = len(idx2ary)

    ary2idx = dict()
    for k,v in idx2ary.items():
        ary2idx[v] = k

    return idx2ary, ary2idx


def get_idxary_map(quadrant):
    def blocked_ur(i, j):
        if (i >= 64 and i <= 128) and (j == 128 or j == 129):
            return True
        else:
            return False

    def blocked_none(i, j):
        return False

    config = {
        'ur': {'start_ary': (63,128),
               'end_ary': (128,256),
               'imin': 0, 'imax': 128,
               'jmin': 128, 'jmax': 256,
               'blocked': blocked_ur,
              },
        'lr': {'start_ary': (129, 256),
               'end_ary': (256,128),
               'imin': 129, 'imax': 256,
               'jmin': 128, 'jmax': 256,
               'blocked': blocked_none,
              },
        'll': {'start_ary': (256, 127),
               'end_ary': (128, 0),
               'imin': 128, 'imax': 256,
               'jmin': 0, 'jmax': 127,
               'blocked': blocked_none,
              },
        'ul': {'start_ary': (127, 0),
               'end_ary': (64,127),
               'imin': 0, 'imax': 127,
               'jmin': 0, 'jmax': 127,
               'blocked': blocked_none,
              }
    }

    return get_order_ary_map(
            config[quadrant]['imin'],
            config[quadrant]['imax'],
            config[quadrant]['jmin'],
            config[quadrant]['jmax'],
            config[quadrant]['blocked'],
            config[quadrant]['start_ary'],
            config[quadrant]['end_ary'],
            )


def get_position(config):
    return reduce(lambda p, q: (p[0] + q[0], p[1] + q[1]), config, (0, 0))


def dump_path(path):
    psrc = get_position(path[0])
    pdst = get_position(path[-1])
    print('==== dump_path ====')
    print(psrc, '=>', pdst, 'len %d' % len(path))
    for config in path:
        print(config)
    

# Functions to map between cartesian coordinates and array indexes
def cartesian_to_array(x, y, shape):
    m, n = shape[:2]
    i = (n - 1) // 2 - y
    j = (n - 1) // 2 + x
    if i < 0 or i >= m or j < 0 or j >= n:
        raise ValueError("Coordinates not within given dimensions.")
    return i, j


def check_path_valid(path):
    origin = [(64, 0), (-32, 0), (-16, 0), (-8, 0), (-4, 0), (-2, 0), (-1, 0), (-1, 0)]
    vector_len = [max(abs(v[0]), abs(v[1])) for v in origin]
    visited = np.zeros([257, 257]) # one = unvisited pixel; 0 = visited pixel
    total = 1
    visited[(128, 128)] = 1
    assert path[0] == origin
    assert path[-1] == origin
    for i in range(1, len(path)):
        config = path[i]
        for j in range(len(config)):
            assert abs(config[j][0]) == vector_len[j] or abs(config[j][1]) == vector_len[j]
        last_config = path[i-1]
        for j in range(len(config)):
            assert abs(config[j][0]-last_config[j][0]) + abs(config[j][1]-last_config[j][1]) <= 1
        point = cartesian_to_array(*get_position(config), visited.shape)
        if visited[point] == 0:
            visited[point] = 1
            total += 1
    print('total visit:', total)
    print('path len:', len(path))
    for i in range(257):
        for j in range(257):
            if visited[(i, j)] == 0:
                pass
                #print('miss: ', (i, j))
    assert total == 257*257



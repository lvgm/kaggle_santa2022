#!/usr/bin/env python

import numpy as np
import pandas as pd
import santatools


def generate_tsp(fname_image, fname_tsp):
    df_image = pd.read_csv(fname_image)
    image = santatools.df_to_image(df_image)

    idx2ary, ary2idx = santatools.get_idxary_map_full()

    f = open(fname_tsp, 'w')
    lines = [
        'NAME : test',
        'COMMENT : test problem',
        'TYPE : TSP',
        'DIMENSION : %d' % (257*257),
        'EDGE_WEIGHT_TYPE : SPECIAL',
        'EDGE_DATA_FORMAT : EDGE_LIST',
        'EDGE_DATA_SECTION',
        '',
    ]
    f.write('\n'.join(lines))
    sq2 = np.sqrt(2)
    sq3 = np.sqrt(3)
    cut_length = 32
    for i in range(256+1):
        for j in range(256+1):
            y = 128 - i
            x = j - 128
            # do not cross dangerous line (0,0)->[(0,32), (0,-32), (32,0), (-32,0)]
            # can't generate best tour, but eaiser for reconfiguration path
            if j+1 <= 256 and (x != -1 or abs(y) > cut_length):
                f.write('%d %d %d\n' % (ary2idx[(i,j)], ary2idx[(i,j+1)], 1000*(1+santatools.color_cost((i,j), (i,j+1), image))))
            if i+1 <= 256 and (y != 0 or abs(x) > cut_length):
                f.write('%d %d %d\n' % (ary2idx[(i,j)], ary2idx[(i+1,j)], 1000*(1+santatools.color_cost((i,j), (i+1,j), image))))
            if j+1 <= 256 and i+1 <= 256:
                if (y != 0 or abs(x) > cut_length) and (x != -1 or abs(y) > cut_length):
                    f.write('%d %d %d\n' % (ary2idx[(i,j)], ary2idx[(i+1,j+1)], 1000*(sq2 + (santatools.color_cost((i,j), (i+1,j+1), image)))))
                    f.write('%d %d %d\n' % (ary2idx[(i+1,j)], ary2idx[(i,j+1)], 1000*(sq2 + (santatools.color_cost((i+1,j), (i,j+1), image)))))

            if j+2 <= 256 and (x != -2 and x != -1 or abs(y) > cut_length):
                f.write('%d %d %d\n' % (ary2idx[(i,j)], ary2idx[(i,j+2)], 1000*(sq2+santatools.color_cost((i,j), (i,j+2), image))))
            if i+2 <= 256 and (y != 0 and y != 1 or abs(x) > cut_length):
                f.write('%d %d %d\n' % (ary2idx[(i,j)], ary2idx[(i+2,j)], 1000*(sq2+santatools.color_cost((i,j), (i+2,j), image))))

            if (j+2 <= 256 and ((x != -2 and x != -1) or abs(y) > cut_length)) and (i+1 <= 256 and (y != 0 or abs(x) > cut_length)):
                f.write('%d %d %d\n' % (ary2idx[(i,j)], ary2idx[(i+1,j+2)], 1000*(sq3+santatools.color_cost((i,j), (i+1,j+2), image))))
                f.write('%d %d %d\n' % (ary2idx[(i+1,j)], ary2idx[(i,j+2)], 1000*(sq3+santatools.color_cost((i+1,j), (i,j+2), image))))

            if (i+2 <= 256 and (y != 0 and y != 1 or abs(x) > cut_length)) and (j+1 <= 256 and (x != -1 or abs(y) > cut_length)) :
                f.write('%d %d %d\n' % (ary2idx[(i,j)], ary2idx[(i+2,j+1)], 1000*(sq3+santatools.color_cost((i,j), (i+2,j+1), image))))
                f.write('%d %d %d\n' % (ary2idx[(i+2,j)], ary2idx[(i,j+1)], 1000*(sq3+santatools.color_cost((i+2,j), (i,j+1), image))))

    f.write('-1\nEOF\n')
    f.close()


def generate_par(fname):
    f = open(fname, 'w')
    lines = [
        'PROBLEM_FILE = image_full.tsp',
        'MAX_TRIALS = 1',
        'MOVE_TYPE = 5',
        'PATCHING_C = 3',
        'PATCHING_A = 2',
        'RUNS = 1',
        'TRACE_LEVEL = 2',
        'INITIAL_PERIOD = 1000',
        'TIME_LIMIT = 1000',
        'TOUR_FILE = tour_full',
        '',
    ]
    f.write('\n'.join(lines))
    f.close()

generate_par('LKH/image_full.par')
generate_tsp('./download/image.csv', 'LKH/image_full.tsp')


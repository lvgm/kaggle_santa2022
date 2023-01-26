#!/usr/bin/env python

import numpy as np
import pandas as pd
#import sys
import santatools


def generate_tsp_common(image, fname, idx2ary, ary2idx, quadrant):
    config = {
        'ur': {'i2': False, 'j2': True, },
        'lr': {'i2': True, 'j2': False, },
        'll': {'i2': False, 'j2': True, },
        'ul': {'i2': True, 'j2': False, }
    }

    f = open(fname, 'w')
    lines = [
        'NAME : test',
        'COMMENT : test problem',
        'TYPE : TSP',
        'DIMENSION : %d' % len(idx2ary),
        'EDGE_WEIGHT_TYPE : SPECIAL',
        'EDGE_DATA_FORMAT : EDGE_LIST',
        'EDGE_DATA_SECTION',
        '',
    ]
    f.write('\n'.join(lines))

    q2 = np.sqrt(2)
    q3 = np.sqrt(3)

    for idx in range(1, len(idx2ary)+1):
        src = idx2ary[idx]

        i1j0 = (src[0]+1, src[1])
        if i1j0 in ary2idx:
            f.write('%d %d %d\n' % (idx, ary2idx[i1j0], 1000*(1+santatools.color_cost(src, i1j0, image))))

        i0j1 = (src[0], src[1]+1)
        if i0j1 in ary2idx:
            f.write('%d %d %d\n' % (idx, ary2idx[i0j1], 1000*(1+santatools.color_cost(src, i0j1, image))))

        i1j1 = (src[0]+1, src[1]+1)
        if i1j1 in ary2idx:
            f.write('%d %d %d\n' % (idx, ary2idx[i1j1], 1000*(q2+santatools.color_cost(src, i1j1, image))))

        i1j_1 = (src[0]+1, src[1]-1)
        if i1j_1 in ary2idx:
            f.write('%d %d %d\n' % (idx, ary2idx[i1j_1], 1000*(q2+santatools.color_cost(src, i1j_1, image))))

        if config[quadrant]['j2'] == True:
            i0j2 = (src[0], src[1]+2)
            if i0j2 in ary2idx:
                f.write('%d %d %d\n' % (idx, ary2idx[i0j2], 1000*(q2+santatools.color_cost(src, i0j2, image))))

            i1j2 = (src[0]+1, src[1]+2)
            if i1j2 in ary2idx:
                f.write('%d %d %d\n' % (idx, ary2idx[i1j2], 1000*(q3+stantatools.color_cost(src, i1j2, image))))

            i1j_2 = (src[0]+1, src[1]-2)
            if i1j_2 in ary2idx:
                f.write('%d %d %d\n' % (idx, ary2idx[i1j_2], 1000*(q3+stantatools.color_cost(src, i1j_2, image))))

        if config[quadrant]['i2'] == True:
            i2j0 = (src[0]+2, src[1])
            if i2j0 in ary2idx:
                f.write('%d %d %d\n' % (idx, ary2idx[i2j0], 1000*(q2+stantatools.color_cost(src, i2j0, image))))

            i2j1 = (src[0]+2, src[1]+1)
            if i2j1 in ary2idx:
                f.write('%d %d %d\n' % (idx, ary2idx[i2j1], 1000*(q3+stantatools.color_cost(src, i2j1, image))))

            i2j_1 = (src[0]+2, src[1]-1)
            if i2j_1 in ary2idx:
                f.write('%d %d %d\n' % (idx, ary2idx[i2j_1], 1000*(q3+stantatools.color_cost(src, i2j_1, image))))

    f.write('%d %d %d\n' % (1, len(idx2ary), 1))
    f.write('-1\nEOF\n')


def generate_tsp_upper_right(image, fname, quadrant):
    idx2ary, ary2idx = santatools.get_idxary_map_4q(quadrant)
    generate_tsp_common(image, fname, idx2ary, ary2idx, quadrant)


def generate_tsp_lower_right(image, fname, quadrant):
    idx2ary, ary2idx = santatools.get_idxary_map_4q(quadrant)
    generate_tsp_common(image, fname, idx2ary, ary2idx, quadrant)


def generate_tsp_lower_left(image, fname, quadrant):
    idx2ary, ary2idx = santatools.get_idxary_map_4q(quadrant)
    generate_tsp_common(image, fname, idx2ary, ary2idx, quadrant)


def generate_tsp_upper_left(image, fname, quadrant):
    idx2ary, ary2idx = santatools.get_idxary_map_4q(quadrant)
    generate_tsp_common(image, fname, idx2ary, ary2idx, quadrant)


def generate_par():
    for quadrant in ['ur', 'lr', 'll', 'ul']:
        f = open('LKH/image_%s.par' % quadrant, 'w')
        lines = [
			'PROBLEM_FILE = image_%s.tsp' % quadrant,
			'MAX_TRIALS = 2',
			'MOVE_TYPE = 5',
			'PATCHING_C = 3',
			'PATCHING_A = 2',
			'RUNS = 2',
            'INITIAL_PERIOD = 1000',
			'TRACE_LEVEL = 1',
			'TOUR_FILE = tour_%s' % quadrant,
            '',
		]
        f.write('\n'.join(lines))
        f.close()


def generate_tsp():
    df_image = pd.read_csv('./download/image.csv')
    image = santatools.df_to_image(df_image)
    generate_tsp_upper_right(image, 'LKH/image_ur.tsp', 'ur')
    generate_tsp_lower_right(image, 'LKH/image_lr.tsp', 'lr')
    generate_tsp_lower_left(image, 'LKH/image_ll.tsp', 'll')
    generate_tsp_upper_left(image, 'LKH/image_ul.tsp', 'ul')


generate_par()
generate_tsp()


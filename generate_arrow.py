#!/usr/bin/env python


def load_sub(fname):
    points = []
    f = open(fname)
    for line in f:
        line = line.rstrip('\n')
        if line == 'configuration':
            continue
        config = [c.split() for c in line.split(';')]
        x = sum([int(c[0]) for c in config])
        y = sum([int(c[1]) for c in config])
        points.append((x,y))
    f.close()
    return points


def save_arrow(points, fname):
    fw = open(fname, 'w')
    fw.write('x,y,dx,dy\n')
    lastx = lasty = -999
    for p in points:
        x, y = p[0], p[1]
        if lastx != -999:
            fw.write('%d,%d,%d,%d\n' % (lastx, lasty, x-lastx, y-lasty))
        lastx, lasty = x, y
    fw.write('%d,%d,%d,%d\n' % (lastx, lasty, 0, 0))
    fw.close()


def main():
    points = load_sub('sub_4q.csv')
    save_arrow(points, 'arrow.csv')

if __name__ == '__main__':
    main()

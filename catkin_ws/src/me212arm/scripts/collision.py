#!/usr/bin/python

from planner import fk1, fk
import numpy as np

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def in_collision(q, obstacle_segs):
    arm_segments = [( (0,0), fk1(q) ), ( fk1(q), fk(q))]
    
    for seg1 in arm_segments:
        for seg2 in obstacle_segs:
            if intersect(seg1[0], seg1[1], seg2[0], seg2[1]):
                return True
    return False

if __name__=="__main__":
    
    obstacle_segs = [ [[0.2,0.2], [0.4,0.2]] ]  # line segs ((x1,z1)--(x2,z2))
    print in_collision( [0,0], obstacle_segs)
    print in_collision( [-0.45709828817786735, -1.4971869034039356], obstacle_segs)

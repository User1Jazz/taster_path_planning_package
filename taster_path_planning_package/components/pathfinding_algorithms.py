from .path_planning_basics import Point
#import numpy as np
#from scipy.spatial import Delaunay

"""
Midpoint algorithm
"""
def calculate_midpoint(right_cone, left_cone):
    midpoint = [((left_cone.x + right_cone.x) / 2), ((left_cone.y + right_cone.y) / 2)]
    return midpoint

"""
Diagonal Intersection Algorithm
"""
def get_intersection_point(p1, p2, p3, p4):
    # Calculate the slopes (m1, m2) of the two lines
    m1 = (p2.y - p1.y) / (p2.x - p1.x) if p2.x != p1.x else float('inf')
    m2 = (p4.y - p3.y) / (p4.x - p3.x) if p4.x != p3.x else float('inf')
    
    # If the slopes are equal, the lines are parallel (or coincident) and have no intersection
    if m1 == m2:
        return None  # No intersection
    
    # Calculate the y-intercepts (b1, b2) of the lines
    b1 = p1.y - m1 * p1.x
    b2 = p3.y - m2 * p3.x
    
    # If one of the lines is vertical, calculate x based on the other line's equation
    if m1 == float('inf'):
        x = p1.x
        y = m2 * x + b2
    elif m2 == float('inf'):
        x = p3.x
        y = m1 * x + b1
    else:
        # Calculate the intersection point (x, y)
        x = (b2 - b1) / (m1 - m2)
        y = m1 * x + b1
    
    return Point(x, y)

def calculate_centroid(point_1, point_2, point_3):
    # Calculate the centroid coordinates
    centroid_x = (point_1.x + point_2.x + point_3.x) / 3
    centroid_y = (point_1.y + point_2.y + point_3.y) / 3
    return Point(centroid_x, centroid_y)
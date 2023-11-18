from math import sin, cos, dist
import numpy as np

def cartesian_to_screen(coords, screen_size):
    """Convert list of (x,y) points from coordinate system with origin at center and y increasing up to system with origin at top left and y increasing down"""
    return [(coord[0] + screen_size[0]/2, -coord[1] + screen_size[1]/2) for coord in coords]

def screen_to_cartesian(coords, screen_size):
    """Convert list of (x,y) points from coordinate system with origin at top left and y increasing down to system with origin at center and y increasing up"""
    return [(coord[0] - screen_size[0]/2, -coord[1] + screen_size[1]/2) for coord in coords]

def rotate_about_midpoint(points, angle):
    """Rotate an array of (x,y) points about their midpoint"""
    xbar = 0
    ybar = 0

    for point in points:
        xbar += point[0]
        ybar += point[1]

    xbar /= len(points)
    ybar /= len(points)

    new_points = []

    for point in points:
        new_points.append((xbar + cos(angle) * (point[0] - xbar) - sin(angle) * (point[1] - ybar),
                           ybar + sin(angle) * (point[0] - xbar) + cos(angle) * (point[1] - ybar)))

    return new_points

def get_nearby_boids(x, y, distance, boids):
    """Get boids within a particular distance of a point (x,y)"""
    return [b for b in boids if dist((x,y), (b.pos[0],b.pos[1])) <= distance]

def get_visible_protected_boids(x, y, visible_range, protected_range, boids):
    """Get boids within two given distances of a point (x,y), return as separate lists (visible, protected)"""
    visible = []
    protected = []
    for b in boids:
        if dist((x,y), (b.pos[0],b.pos[1])) <= visible_range:
            visible.append(b)
        if dist((x,y), (b.pos[0],b.pos[1])) <= protected_range:
            protected.append(b)
    return visible, protected

def get_colliding_obstacles(x, y, obstacles):
    """Get obstacles that are within trigger distance of a point (x,y)"""
    return [o for o in obstacles if dist((x,y), (o.pos[0], o.pos[1])) <= o.r * o.buffer]

def norm(vec):
    """Compute the L2-norm"""
    return np.sum(vec**2)**0.5

def clamp_norm(vec, min_s, max_s):
    """Constrain the norm of a vector within the range [min_s, max_s]"""
    vnorm = norm(vec)
    normalized = vec / vnorm
    if vnorm < min_s:
        return normalized * min_s
    elif vnorm > max_s:
        return normalized * max_s
    return vec
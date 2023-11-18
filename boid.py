import pygame as pg
import math
import numpy as np
from utilities import *

class Boid:
    def __init__(self, x, y, vx, vy, tick_scale):
        self.pos = np.array([x, y], dtype=np.float64) # Position
        self.v = np.array([vx, vy], dtype=np.float64) # Velocity
        self.tick_scale = tick_scale # How much to scale velocity changes

    def draw(self, surface, screen_size, shape=np.array([(-1,-2), (1,-2), (0,2)]), color=(0,0,0)):
        """Draw boid"""
        # Compute the direction of the boid's motion
        if self.v[0] != 0:
            angle = math.atan(self.v[1]/self.v[0]) # Basic trig: angle of point (x,y) is atan(y/x)
        else:
            angle = -math.pi/2 * (self.v[1]/abs(self.v[1])) # Manage directly vertical motion based on sign of vy
        angle += math.pi / 2 # Set 0 point to 90 degrees
        if self.v[0] != 0 and self.v[0]/abs(self.v[0]) == 1: # Correct for limited domain of atan
            angle += math.pi
        pg.draw.polygon(surface, color,
                        cartesian_to_screen(
                            rotate_about_midpoint([(self.pos[0] + dx, self.pos[1] + dy) # Rotate polygon points by angle
                                                   for (dx, dy) in shape],
                                                  angle),
                            screen_size))

    def step(self):
        self.pos += self.v / self.tick_scale
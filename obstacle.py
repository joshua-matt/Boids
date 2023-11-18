from utilities import *
import numpy as np
import pygame as pg

class Obstacle:
    def __init__(self, x, y, r, buffer):
        self.pos = np.array([x,y]) # Position
        self.r = r # Radius
        self.buffer = buffer # Multiple of radius at which to trigger boid collisions

    def draw(self, surface, screen_size, color=(255,0,0)):
        """Draw obstacle"""
        pg.draw.circle(surface, color,
                       cartesian_to_screen([(self.pos[0], self.pos[1])], screen_size)[0],
                       self.r)


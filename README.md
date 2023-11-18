# Boids
## Overview
This repository contains an interactive pygame implementation of the [Boids](https://en.wikipedia.org/wiki/Boids) artificial life program, which uses three simple rules to create emergent flock-like behavior:
1. Separation: avoid crowding boids within a given protected range
2. Alignment: move towards the average velocity of boids within a given visible range (larger than the protected range)
3. Cohesion: move towards the average position of boids within a given visible range

My implementation of the program also contains obstacles, which the boids steer away from.
## Files
- **main.py** runs and renders the interactive simulation
- **boid.py** contains the Boid class
- **obstacle.py** contains the Obstacle class
- **utilities.py** contains miscellaneous methods involved in rendering, fetching objects, and computing boid velocities

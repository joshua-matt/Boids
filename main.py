import pygame as pg # Graphics display, user interaction
from boid import * # Boid class
from obstacle import * # Obstacle class
from utilities import * # Miscellaneous graphics, other functions
import random as rn # Random initializations for positions, velocities
import math # Trig functions

SCREEN_SIZE = [500,500]
WHITE = (255,255,255)
BLACK = (0,0,0)
BOID_SHAPE = np.array([(-1,-2), (1,-2), (0,2)]) # Vertices of polygon for boid shape
BOID_SCALE = 2 # How much to scale up BOID_SHAPE

N_BOIDS = 50 # Number of boids to start out
VISIBLE_RANGE = 100 # Pixel range for cohesion and centering
PROTECTED_RANGE = 10 # Pixel range for separation
EDGE_BUFFER = 50 # How many pixels from edge to trigger edge avoidance

N_OBSTACLE = 0 # Number of obstacles to start out
OBSTACLE_RADIUS = 20
OBSTACLE_BUFFER = 2 # Fraction of obstacle radius to start avoiding from

ALIGNMENT_FACTOR = 0.05 # How much to match visible velocities
COHESION_FACTOR = 0.0005 # How much to match visible positions
SEPARATION_FACTOR = 0.1 # How much to move away from close boids
EDGE_FACTOR = 1.5 # How hard to avoid the edges
OBSTACLE_FACTOR = 0.15 # How hard to avoid the obstacles

INIT_SPEED = 25 # Initial speed of all boids
MIN_SPEED = 20
MAX_SPEED = 30
TICK_SCALE = 10 # How much to divide velocities by. Higher values mean slower simulations

CLICK_FUNCTION = 1 # 0 = create boid, 1 = create obstacle

boids = []
obstacles = []

for i in range(N_BOIDS): # Initialize boids with random position, velocity
    angle = rn.random() * math.pi
    vx = math.cos(angle) * INIT_SPEED
    vy = math.sin(angle) * INIT_SPEED

    boids.append(Boid(rn.randint(-SCREEN_SIZE[0]/2 + EDGE_BUFFER, SCREEN_SIZE[0]/2 - EDGE_BUFFER), # x
                      rn.randint(-SCREEN_SIZE[1]/2 + EDGE_BUFFER, SCREEN_SIZE[1]/2 - EDGE_BUFFER), # y
                      vx, # vx
                      vy, # vy
                      TICK_SCALE))

for i in range(N_OBSTACLE): # Initialize obstacles with random position
    obstacles.append(Obstacle(rn.randint(-SCREEN_SIZE[0]/2 + OBSTACLE_RADIUS, SCREEN_SIZE[0]/2 - OBSTACLE_RADIUS), # x
                              rn.randint(-SCREEN_SIZE[1]/2 + OBSTACLE_RADIUS, SCREEN_SIZE[1]/2 - OBSTACLE_RADIUS), # y
                              OBSTACLE_RADIUS,
                              OBSTACLE_BUFFER))

pg.init()
pg.display.set_caption("Boids") # Set window title
screen = pg.display.set_mode(SCREEN_SIZE) # Display surface

running = True
while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
        if event.type == pg.MOUSEBUTTONUP:
            mouse_x, mouse_y = screen_to_cartesian([pg.mouse.get_pos()], SCREEN_SIZE)[0] # Get mouse coords relative to center of screen
            if CLICK_FUNCTION == 0: # Create boid at mouse
                angle = rn.random() * math.pi
                vx = math.cos(angle) * INIT_SPEED
                vy = math.sin(angle) * INIT_SPEED

                boids.append(Boid(mouse_x,
                                  mouse_y,
                                  vx,
                                  vy,
                                  TICK_SCALE))
            elif CLICK_FUNCTION == 1: # Create obstacle at mouse
                obstacles.append(Obstacle(mouse_x,
                                          mouse_y,
                                          OBSTACLE_RADIUS,
                                          OBSTACLE_BUFFER))

    screen.fill(WHITE) # White background

    ### DETERMINE NEW BOID VELOCITIES
    # Three effects:
    #   1. Move towards average velocity of boids within VISIBLE_RANGE (Alignment)
    #   2. Move towards average position of boids within VISIBLE_RANGE (Cohesion)
    #   3. Move away from average position of boids within PROTECTED_RANGE (Separation)

    velocity_shifts = [np.zeros(2) for i in range(len(boids))]

    for i, boid in enumerate(boids):
        visible, protected = get_visible_protected_boids(boid.pos[0], boid.pos[1], VISIBLE_RANGE, PROTECTED_RANGE, boids)
        close_obstacles = get_colliding_obstacles(boid.pos[0], boid.pos[1], obstacles)

        # Edge avoidance
        if boid.pos[0] > SCREEN_SIZE[0] / 2 - EDGE_BUFFER:
            velocity_shifts[i][0] -= EDGE_FACTOR
        if boid.pos[0] < -SCREEN_SIZE[0] / 2 + EDGE_BUFFER:
            velocity_shifts[i][0] += EDGE_FACTOR
        if boid.pos[1] > SCREEN_SIZE[1] / 2 - EDGE_BUFFER:
            velocity_shifts[i][1] -= EDGE_FACTOR
        if boid.pos[1] < -SCREEN_SIZE[1] / 2 + EDGE_BUFFER:
            velocity_shifts[i][1] += EDGE_FACTOR

        # Obstacle avoidance
        if close_obstacles != []:
            average_obs_pos = np.mean(np.vstack([o.pos for o in close_obstacles]), axis=0)
            velocity_shifts[i] -= (average_obs_pos - boid.pos) * OBSTACLE_FACTOR

        # Alignment (match average visible velocities)
        average_vel = np.mean(np.vstack([b.v for b in visible]), axis=0)
        velocity_shifts[i] += (average_vel - boid.v) * ALIGNMENT_FACTOR

        # Centering (match average visible position)
        average_pos = np.mean(np.vstack([b.pos for b in visible]), axis=0)
        velocity_shifts[i] += (average_pos - boid.pos) * COHESION_FACTOR

        # Separation (move away from close boids)
        average_pos_prot = np.mean(np.vstack([b.pos for b in protected]), axis=0)
        velocity_shifts[i] -= (average_pos_prot - boid.pos) * SEPARATION_FACTOR  # -= moves *away* from center

    for i, boid in enumerate(boids): # Apply velocity changes, step the simulation forward, and draw
        boid.v += velocity_shifts[i]
        boid.v = clamp_norm(boid.v, MIN_SPEED, MAX_SPEED)
        boid.step()
        boid.draw(screen, SCREEN_SIZE, shape=BOID_SHAPE*BOID_SCALE, color=BLACK)

    for obs in obstacles:
        obs.draw(screen, SCREEN_SIZE)

    pg.display.flip() # Update the display
pg.quit()

### TODO: increase efficiency to allow for more boids
### TODO: fix obstacles to that boids never hit them
###    it seems that a percent-based buffer is not enough to avoid visual collisions with obstacles
### TODO: add in-simulation tuning of parameters
### TODO: add switchable mouse functionality: avoid/go to/nothing mouse, add obstacle/boid on-click
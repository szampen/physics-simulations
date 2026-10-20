import math
import random
import pygame
import sys

pygame.init()

width, height = 1280, 768
screen = pygame.display.set_caption("Ball simulation")
window = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

sim_min_width = 20.0
cScale = min(width, height) / sim_min_width # how many pixels are equal to 1 meter
sim_width = width / cScale # width in meters (scaled)
sim_height = height / cScale # height in meter (scaled)

gravity = {"x": 0.0, "y": -10.0}
time_step = 1.0 / 60.0
air_drag = 0.1 # air resistance slowing the ball down by 10% in 1 second
restitution = 0.9 # ball retains 90% of velocity after bounce

# Scaling functions to establish new coordinate system - (0,0) at the bottom left corner instead of upper one

def c_x(x):
    return int(x * cScale)

def c_y(y):
    return int(height - y * cScale)

def screen_to_sim(px, py):
    sim_x = px / cScale
    sim_y = (height - py) / cScale
    return sim_x, sim_y

class Ball:
    def __init__(self, posX, posY, velX, velY, radius, color):
        self.posX = posX
        self.posY = posY
        self.velX = velX
        self.velY = velY
        self.radius = radius
        self.color = color

    def simulate(self):
        # Air drag impact on velocity
        self.velX -= self.velX * air_drag * time_step
        self.velY -= self.velY * air_drag * time_step

        # Gravity impact on velocity - v_{n+1} = v_n + a * delta_t
        self.velX += gravity["x"] * time_step
        self.velY += gravity["y"] * time_step

        # Position update - x_{n+1} = x_n + v_{n+1} * delta_t
        self.posX += self.velX * time_step
        self.posY += self.velY * time_step

        # Collisions with "box" edges

        # Left wall
        if self.posX - self.radius < 0.0:
            self.posX = self.radius
            self.velX = -self.velX * restitution # reversing the velocity vector and reducing it by restitution

        # Right wall
        if self.posX + self.radius > sim_width:
            self.posX = sim_width - self.radius
            self.velX = -self.velX * restitution

        # Bottom wall
        if self.posY - self.radius < 0.0:
            self.posY = self.radius
            self.velY = -self.velY * restitution

        # Top wall
        if self.posY + self.radius > sim_height:
            self.posY = sim_height - self.radius
            self.velY = -self.velY * restitution

    def draw(self, surface):
        center_x = c_x(self.posX)
        center_y = c_y(self.posY)
        pixel_radius = int(self.radius * cScale)

        pygame.draw.circle(surface, self.color, (center_x, center_y), pixel_radius)

    # mx, my - mouse cursor coordinates
    def check_click(self, mx, my):
        dx = mx - self.posX
        dy = my - self.posY
        distance = math.hypot(dx, dy) # euclidean distance between cursor and ball

        if distance <= self.radius:
            self.velY = 14.0
            self.velX += (self.posX - mx) * 10.0 # if you click on the left of the center of the ball then it adds a vector of velocity to the right
            return True # one ball can be clicked
        return False

    def check_obstacle_collision(self, obstacle):
        dx = self.posX - obstacle.posX
        dy = self.posY - obstacle.posY
        dist = math.hypot(dx, dy) # euclidean distance between ball and obstacle
        min_dist = self.radius + obstacle.radius # r + R

        # dist between x0 and x1 < r + R
        if dist < min_dist and dist > 0:
            # normal vector to determine the direction of bounce of the obstacle; its length is always equal to one
            nx = dx/dist
            ny = dy/dist

            # ball out of obstacle if it gets there bcs of discrete time - prevents ball getting stuck
            self.posX = obstacle.posX + nx * min_dist
            self.posY = obstacle.posY + ny * min_dist

            # scalar product that gives us the proportion of velocity that is a impact of the ball on obstacle
            # dot < 0 => ball is going in the way of the obstacle
            dot = self.velX * nx + self.velY * ny
            if dot < 0:
                self.velX = (self.velX - 2 * dot * nx) * restitution
                self.velY = (self.velY - 2 * dot * ny) * restitution

            # dot * nx - the velocity of the impact from x side
            # dot * ny - the velocity of the impact from y side
            # 2 * dot * nx - one for the absorbing of the movement to the inside of obstacle, the second one for bounce back
            # symmetric bounce

class Obstacle:
    def __init__(self, posX, posY, radius):
        self.posX = posX
        self.posY = posY
        self.radius = radius

    def draw(self, surface):
        center_x = c_x(self.posX)
        center_y = c_y(self.posY)
        pixel_radius = int(self.radius * cScale)

        pygame.draw.circle(surface, (120, 120, 120), (center_x, center_y), pixel_radius)
        pygame.draw.circle(surface, (0, 0, 0), (center_x, center_y), pixel_radius, 3) # border


def resolve_balls_collision(balls):
    for i in range(len(balls)):
        for j in range(i+1, len(balls)):
            b1 = balls[i]
            b2 = balls[j]

            dx = b2.posX - b1.posX
            dy = b2.posY - b1.posY
            dist = math.hypot(dx, dy)
            min_dist = b1.radius + b2.radius

            if dist < min_dist and dist > 0:
                # normal vector to determine the direction of bounce (ball 2 out of ball 1)
                nx = dx/dist
                ny = dy/dist

                # when balls overlap we push them away in opposite directions
                # 0.5 * overlap bcs balls have the same mass, so they "work" the same amount
                overlap = min_dist - dist
                b1.posX -= nx * overlap * 0.5
                b1.posY -= ny * overlap * 0.5
                b2.posX += nx * overlap * 0.5
                b2.posY += ny * overlap * 0.5

                # vector of relative velocity between the objects
                # 1) b1_vX = 50, b2_vX = -50 => k_vX = 50 - (-50) = 100
                # 2) b1_vX = 50, b2_vX = 40 => k_vX = 50 - 40 = 10
                kx = b1.velX - b2.velX
                ky = b1.velY - b2.velY
                p = nx * kx + ny * ky # dot equivalent

                # conservation of momentum => proportional modification of velocity for both balls
                if p > 0:
                    b1.velX -= p * nx * restitution
                    b1.velY -= p * ny * restitution
                    b2.velX += p * nx * restitution
                    b2.velY += p * ny * restitution

balls = []
obstacles = [
    Obstacle(5.0, 5.0, 1.4),
    Obstacle(15.0, 14.0, 1.6),
    Obstacle(25.0, 8.0, 1.8)
]
colors = [
    (230, 57, 70),
    (241, 250, 238),
    (168, 218, 220),
    (69, 123, 157),
    (29, 53, 87),
    (255, 183, 3),
    (251, 133, 0),
]

for _ in range(10):
    radius = 0.5
    x = random.uniform(radius, sim_width - radius)
    y = random.uniform(sim_height/2, sim_height - radius)
    velX = random.uniform(-15, 15)
    velY = random.uniform(0, 15)
    color = random.choice(colors)

    balls.append(Ball(x, y, velX, velY, radius, color))


running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # clicking left mouse button triggers pygame event
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            px, py = pygame.mouse.get_pos() # pixel mouse position
            mx, my = screen_to_sim(px, py) # metric mouse position

            for ball in balls:
                if ball.check_click(mx, my):
                    break

    for ball in balls:
        ball.simulate()
        for obs in obstacles:
            ball.check_obstacle_collision(obs)

    resolve_balls_collision(balls)

    window.fill((30,30,35))

    for obs in obstacles:
        obs.draw(window)

    for ball in balls:
        ball.draw(window)

    pygame.display.flip() # displays a new scene with all the objects already drawn
    clock.tick(60) # freezes, so it wont generate things faster than 60 frames per second

pygame.quit()
sys.exit()
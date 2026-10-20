import math
import random
import pygame

pygame.init()

width, height = 1280, 768
pygame.display.set_caption("PBD")
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()

font = pygame.font.SysFont("verdana", 16)
button_rect = pygame.Rect(20, 20, 100, 40)
button_color = (96, 96, 96)
text_color = (255, 255, 255)

sim_min_width = 2.0
cScale = min(width, height) / sim_min_width
sim_width = width / cScale
sim_height = height / cScale

def c_x(x):
    return int(x * cScale)

def c_y(y):
    return int(height - y * cScale)

class Vector2:
    def __init__(self, x = 0.0, y = 0.0):
        self.x = x
        self.y = y

    def set_vec(self,v):
        self.x = v.x
        self.y = v.y

    def clone(self):
        return Vector2(self.x, self.y)

    def add(self, v, s = 1.0):
        self.x += v.x * s
        self.y += v.y * s
        return self

    def add_vector(self, a, b):
        self.x = a.x + b.x
        self.y = a.y + b.y
        return self

    def subtract(self, v, s = 1.0):
        self.x -= v.x * s
        self.y -= v.y * s
        return self

    def subtract_vector(self, a, b):
        self.x = a.x - b.x
        self.y = a.y - b.y
        return self

    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def scale(self, s):
        self.x *= s
        self.y *= s
        return self

    def dot(self, v):
        return self.x * v.x + self.y * v.y

    def perp(self):
        return Vector2(-self.y, self.x)

class Bead:
    def __init__(self, radius, mass, pos):
        self.radius = radius
        self.mass = mass
        self.pos = pos.clone()
        self.prevPos = pos.clone()
        self.vel = Vector2()

    def start_step(self, dt, gravity):
        self.vel.add(gravity, dt)
        self.prevPos.set_vec(self.pos)
        self.pos.add(self.vel, dt)

    def keep_on_wire(self, center, radius):
        dir_vec = Vector2().subtract_vector(self.pos, center)
        length = dir_vec.length()
        if length == 0.0:
            return
        dir_vec.scale(1.0 / length)
        lambda_val = radius - length
        self.pos.add(dir_vec, lambda_val)

    def end_step(self, dt):
        self.vel.subtract_vector(self.pos, self.prevPos)
        self.vel.scale(1.0 / dt)

physics_scene = {
    "gravity": Vector2(0.0, -10.0),
    "dt": 1.0 / 60.0,
    "num_steps": 100,
    "wire_center": Vector2(sim_width / 2.0, sim_height / 2.0),
    "wire_radius": sim_min_width * 0.4,
    "beads": []
}

def setup_scene():
    physics_scene["beads"] = []
    num_beads = 5

    r = 0.1
    angle = 0.0
    for i in range(num_beads):
        mass = math.pi * r * r
        pos = Vector2(
            physics_scene["wire_center"].x + physics_scene["wire_radius"] * math.cos(angle),
            physics_scene["wire_center"].y + physics_scene["wire_radius"] * math.sin(angle)
        )
        physics_scene["beads"].append(Bead(r, mass, pos))
        angle += math.pi / num_beads
        r = 0.05 + random.random() * 0.1

def handle_bead_bead_collision(bead1, bead2):
    restitution = 1.0
    dir_vec = Vector2().subtract_vector(bead2.pos, bead1.pos)
    d = dir_vec.length()

    # no collision, if the distance longer than sum of both radius
    if d == 0.0 or d > bead1.radius + bead2.radius:
        return

    dir_vec.scale(1.0 / d)

    # position correction
    corr = (bead1.radius + bead2.radius - d) / 2.0
    bead1.pos.add(dir_vec, -corr)
    bead2.pos.add(dir_vec, corr)

    # conservation of momentum
    v1 = bead1.vel.dot(dir_vec)
    v2 = bead2.vel.dot(dir_vec)
    m1 = bead1.mass
    m2 = bead2.mass

    new_v1 = (m1 * v1 + m2 * v2 - m2 * (v1 - v2) * restitution) / (m1 + m2)
    new_v2 = (m1 * v1 + m2 * v2 - m1 * (v2 - v1) * restitution) / (m1 + m2)

    bead1.vel.add(dir_vec, new_v1 - v1)
    bead2.vel.add(dir_vec, new_v2 - v2)

def simulate():
    sdt = physics_scene["dt"] / physics_scene["num_steps"]

    # substeps
    for step in range(physics_scene["num_steps"]):
        for bead in physics_scene["beads"]:
            bead.start_step(sdt, physics_scene["gravity"])

        for bead in physics_scene["beads"]:
            bead.keep_on_wire(physics_scene["wire_center"], physics_scene["wire_radius"])

        for bead in physics_scene["beads"]:
            bead.end_step(sdt)

        # collision check
        for i in range(len(physics_scene["beads"])):
            for j in range(i):
                handle_bead_bead_collision(physics_scene["beads"][i], physics_scene["beads"][j])

def draw_circle(surface, color, pos, radius, filled):
    center = (c_x(pos.x), c_y(pos.y))
    pixel_radius = int(cScale * radius)
    circle_width = 0 if filled else 2
    pygame.draw.circle(surface, color, center, pixel_radius, circle_width)

def draw():
    screen.fill((255, 255 ,255)) # white background

    pygame.draw.rect(screen, button_color, button_rect)
    text_surf = font.render("Restart", True, text_color)
    text_rect = text_surf.get_rect(center=button_rect.center)
    screen.blit(text_surf, text_rect)

    # main circle - without filling
    draw_circle(screen, (255, 0, 0), physics_scene["wire_center"], physics_scene["wire_radius"], False)

    # beads
    for bead in physics_scene["beads"]:
        draw_circle(screen, (255, 0, 0), bead.pos, bead.radius, True)

    pygame.display.flip()


setup_scene()
running = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                if button_rect.collidepoint(event.pos):
                    setup_scene()

    simulate()
    draw()

    clock.tick(60)

pygame.quit()
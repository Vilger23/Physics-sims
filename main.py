import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math
import pygame as pg 
import random

class v:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z

    def __add__(self, other):
        return v(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return v(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, other):
        return v(self.x*other, self.y*other, self.z*other)

    def __truediv__(self, other):
        return v(self.x/other, self.y/other, self.z/other)

    def abs(self):
        return self.dot(self)

    def magnitude(self):
        return math.sqrt(self.abs())

    def normalized(self):
        mag = math.sqrt(self.abs())
        return self*(1/mag)

    def dot(self, other):
        return self.x*other.x + self.y * other.y + self.z * other.z

    def xyz(self):
        return self.x, self.y, self.z

class Object:
    def __init__(self, system, pos, v_0, m = 0, r = 0):
        self.system = system
        system.create_new_object(self)
        self.pos = pos
        self.v = v_0
        self.m = m
        self.r = r

    def get_force(self):
        forces = []
        for obj in self.system.objects:
            if obj == self:
                continue
            forces.append(self.system.calculate_gravitational_force(self, obj))

        F = forces[0] + forces[1]
        return F

    def update_velocity(self, dt):
        F = self.get_force()
        self.v = self.v + F*(dt/self.m)

    def update_position(self, dt):
        self.update_velocity(dt)
        self.pos = self.pos + self.v*dt


class System:
    def __init__(self):
        self.G = 6.6743e-11
        self.objects = []

    def create_new_object(self, obj):
        self.objects.append(obj)

    def calculate_gravitational_force(self, obj1, obj2):
        mag = (self.G*obj1.m*obj2.m)/(self.get_dist(obj1, obj2)**2)
        V = obj2.pos - obj1.pos
        V = V.normalized()
        F = V*mag
        return F

    def get_dist(self, obj1, obj2):
        x1, y1, z1 = obj1.pos.xyz()
        x2, y2, z2 = obj2.pos.xyz()
        dist = ( (x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2 )**0.5
        return dist

    def update_objects(self, dt):
        for obj in self.objects:
            obj.update_position(dt)


plt.ion()
fig = plt.figure(figsize=(8,8))

ax = fig.add_subplot(111, projection='3d')




def main():
    screen = setup()
    helocentric = System()
    earth = Object(helocentric, v(1.4989e11, 0, 0), v(0, 3e4, 0), m = 5.972e24, r = 6_371_000)
    moon = Object(helocentric, v(3.8e8+1.4989e11, 0, 0), v(0, 3e4 + 1.022e3, 0), m = 7.3476e22, r = 0)
    sun = Object(helocentric, v(0,                  0, 0), v(0, 0, 0), m = 1.989e30, r = 0)
    start = 0
    elapsed = 0
    dt = 10
    while True:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
                exit() 

        elapsed += dt
        helocentric.update_objects(dt)
        if elapsed > 360*24:

            elapsed = 0
            e_p = earth.pos*(1/20_000_000_00)
            m_p = moon.pos*(1/20_000_000_00)
            s_p = sun.pos*(1/20_000_000_00)

            

            plt.clf()
            ex, ey, ez = e_p.xyz()
            ax = fig.add_subplot(111, projection='3d')
            ax.axes.set_xlim3d(left=-100, right=+100) 
            ax.axes.set_ylim3d(bottom=-100, top=+100) 
            ax.axes.set_zlim3d(bottom=-100, top=+100)

            ax.scatter(*e_p.xyz())
            ax.scatter(*m_p.xyz())
            ax.scatter(*s_p.xyz())
            
            plt.draw()


def draw(points, scr):
    scr.fill(pg.Color(0, 0, 0))
    r = (-10, 10)
    for p in points:
        x, y, z = p.xyz()
        pg.draw.circle(scr, pg.Color(255, 255, 255), ((x+10)*25, (y+10)*25), 10)
def setup():
    scr_w = 500
    scr_h = 500
    pg.init()

    screen = pg.display.set_mode((scr_w, scr_h))
    screen.fill(pg.Color(0, 0, 0))
    pg.display.set_caption('A* pathfinding')
    return screen


main()




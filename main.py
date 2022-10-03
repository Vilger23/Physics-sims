import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import math
import pygame as pg 
import random
from vectors import v


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
    helocentric = System()
    earth = Object(helocentric, v(1.4989e11, 0, 0), v(0, 3e4, 0), m = 5.972e24, r = 6_371_000)
    moon = Object(helocentric, v(3.8e8+1.4989e11, 0, 0), v(0, 3e4 + 1.022e3, 0), m = 7.3476e22, r = 0)
    sun = Object(helocentric, v(0,                  0, 0), v(0, 0, 0), m = 1.989e30, r = 0)
    start = 0
    elapsed = 0
    dt = 10
    i = 0
    while i < 100:
        elapsed += dt
        helocentric.update_objects(dt)
        if elapsed > 360*24:
            i+=1


if __name__ == '__main__':
    main()
import tkinter as tk
from functools import reduce
from PIL import Image, ImageTk
import math
from vectors import *



class Sphere:
    def __init__(self, r, PosV, ColV, shininess, Reflected = 0.5, spec = v(1, 1, 1)):
        self.center = PosV
        self.r = r 
        self.Bcolor = ColV
        self.reflected = Reflected
        self.spec = spec
        self.shininess = shininess
        self.ambient = ColV * 0.1
        self.light = [v(0,0,0)]

    def update_pos(self, PosV):
        self.center = PosV

    def get_color(self, p_rf):
        return self.Bcolor

    def intersect(self, O, D, to_far):
        b = 2 * D.dot(O - self.center)
        c = self.center.abs() + O.abs() - 2 * self.center.dot(O) - (self.r * self.r)
        disc = (b ** 2) - (4 * c)
        sq = np.sqrt(np.maximum(0, disc))
        h0 = (-b - sq) / 2
        h1 = (-b + sq) / 2
        h = np.where((h0 > 0) & (h0 < h1), h0, h1)
        pred = (disc > 0) & (h > 0)
        return np.where(pred, h, to_far)

    def rgb(self, cam, scene, origin, Vect, t, reflections, current_ir = 1):
        P_rf = origin + Vect*t
        N = (P_rf-self.center).normalized()
        color = self.ambient
        g = v(0,0,0)
        for lit in self.light:
            L = (lit - P_rf).normalized()  
            I = (cam.pos - P_rf).normalized()
            
            epsilon = 0.00001
            nudged = P_rf + L*epsilon

            intensity = np.maximum(N.dot(L), 0)

            shadow_rays = [s.intersect(nudged, L, cam.to_far) for s in scene]

            nearest_intersection = reduce(np.minimum, shadow_rays)

            SeeLight = nearest_intersection > (lit - P_rf).magnitude()

            g += self.get_color(P_rf) * intensity* SeeLight

        color += g*(1/len(self.light))

        if reflections < 1 and self.reflected != 0:
            Rv = (Vect - N * 2 * Vect.dot(N)).normalized()
            r_term = self.reflected
            color += trace(scene, nudged, Rv, reflections + 1)*r_term 
        #h = (L + I).normalized()
        #color += self.spec * np.power(np.clip(N.dot(h), 0, 1), self.shininess/4)* SeeLight
        return color




class Cam:
    def __init__(self, origin, V_N, width, aspect_ratio, FOV = math.pi/2, to_far = 1e39):
        self.width = width
        self.r = aspect_ratio[0]/aspect_ratio[1]
        self.height = int(width/self.r)
        
        self.origin = origin
        self.dist = 1/math.tan(FOV/2)
        self.to_far = to_far
        self.focus = v(0, 0, 0)

    def transformation_matrix(self):
        Vx, Vy, Vz = self.V_N.xyz()

        theta = math.asin(-Vz)

        phi = math.acos(  np.clip(Vx/math.cos(theta), -1, 1) )
        phi2 = ( - math.acos(np.clip(Vx/math.cos(theta), -1, 1)) )

        phi_y_1 = math.asin(Vy/math.cos(theta))%(2*math.pi)
        phi_y_2 = (math.pi-math.asin(Vy/math.cos(theta)))%(2*math.pi)

        diff1 = min(abs(phi-phi_y_1), abs(phi-phi_y_2))
        diff2 = min(abs(phi2-phi_y_1), abs(phi2-phi_y_2))
        phi = phi if diff1<diff2 else phi2

        Ry = np.array([
            [ math.cos(theta), 0, math.sin(theta)],
            [ 0              , 1,               0],
            [-math.sin(theta), 0, math.cos(theta)],
            ])

        Rz = np.array([
            [math.cos(phi), -math.sin(phi),  0],
            [math.sin(phi),  math.cos(phi),  0],
            [0            ,  0,              1],
            ])

        '''Rz2 = np.array([
            [math.cos(phi2), -math.sin(phi2),  0],
            [math.sin(phi2),  math.cos(phi2),  0],
            [0            ,  0,              1],
            ])

        diff1 = (self.V_N-v(*Rz@Ry@np.array([1,0,0]))).magnitude()
        diff2 = (self.V_N-v(*Rz2@Ry@np.array([1,0,0]))).magnitude()

        Rz = Rz if diff1 < diff2 else Rz2'''
        return Ry, Rz

    def configure_cam(self):
        self.V_N = ( self.focus - self.origin).normalized()
        Vx, Vy, Vz = self.V_N.xyz()
        theta = math.acos(Vx)
        deg = math.pi/2

        Ry, Rz = self.transformation_matrix()

        #print((self.V_N - v(*Rz@Ry@np.array([1, 0, 0]))).xyz())

        x = np.full((1, self.width*self.height), 0)
        y = np.tile(np.linspace(-1, 1, self.width), self.height)
        z = np.repeat(np.linspace(1/self.r, -1/self.r, self.height), self.width)

        self.x, self.y, self.z = (v(*Rz@Ry@np.vstack( (x,y,z) )) + self.origin).xyz()



        self.pos = self.V_N*-self.dist + self.origin

    def trace(self, scene, reflections = 0):
        origin = self.pos
        vect = (v(self.x, self.y, self.z) - self.pos).normalized()

        t_vals = [s.intersect(origin, vect, self.to_far) for s in scene]
        closests = reduce(np.minimum, t_vals)
        color = v(0, 0, 0.1)
        for (s, t) in zip(scene, t_vals):
            seen = (t == closests) & (closests != self.to_far)
            if np.any(seen):
                pt = extract(seen, t)
                pOr = origin.extract(seen)
                pVe = vect.extract(seen)
                pC = s.rgb(self, scene, pOr, pVe, pt, reflections)
                color += pC.set_rgb(seen)
        return color


    def render(self, scene, root, label):
        self.configure_cam()
        color = self.trace(scene)
        rgb = [Image.fromarray((255*np.clip(c, 0, 1).reshape((self.height, self.width))).astype(np.uint8), "L") for c in color.xyz()]
        img = Image.merge("RGB", rgb)#.resize((int(width/2), int(height/2)))
        IMAGE = ImageTk.PhotoImage(image=img)

        label.config(image=IMAGE)
        label.pack()
        root.update()

def fibonacci_sphere(obj, scene, points=10):
    light_points = []
    r = obj.r*1.5
    origin = obj.center

    phi = math.pi * (3. - math.sqrt(5.))  # golden angle in radians

    for i in range(points):
        y = r - (i / float(points - 1)) * (2*r)  # y goes from r to -r
        radius = math.sqrt(r * r - y * y)  # radius at y

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius
        z = math.sin(theta) * radius

        light_points.append(v(x,y,z)+origin)
    for obj in scene:
        obj.light = light_points                               











'''topleft     = v(*(Rz@Ry@np.array([0, -1,  1]))) + self.origin
topright    = v(*(Rz@Ry@np.array([0,  1,  1]))) + self.origin
bottomleft  = v(*(Rz@Ry@np.array([0, -1, -1]))) + self.origin
bottomright = v(*(Rz@Ry@np.array([0,  1, -1]))) + self.origin

ox = np.linspace(topleft.x, bottomleft.x, self.height).reshape(-1, 1)
oy = np.linspace(topleft.y, bottomleft.y, self.height).reshape(-1, 1)
oz = np.linspace(topleft.z, bottomleft.z, self.height).reshape(-1, 1)

tx = np.linspace(topright.x, bottomright.x, self.height).reshape(-1, 1)
ty = np.linspace(topright.y, bottomright.y, self.height).reshape(-1, 1)
tz = np.linspace(topright.z, bottomright.z, self.height).reshape(-1, 1)
c = np.linspace(0, 1, self.width)
self.x = ox + (tx - ox) * c
self.y = oy + (ty - oy) * c
self.z = oz + (tz - oz) * c'''
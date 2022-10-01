import tkinter as tk
from functools import reduce
import numpy as np 
from PIL import Image, ImageTk
import numbers
import math

class Light:
    P = None

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
        return np.sqrt(self.abs())
    def normalized(self):
        mag = np.sqrt(self.abs())
        return self * (1.0 / np.where(mag == 0, 1, mag))
    def dot(self, other):
        return self.x*other.x + self.y * other.y + self.z * other.z
    def extract(self, mask):
        return v(extract(mask, self.x), extract(mask, self.y), extract(mask, self.z))
    def set_rgb(self, mask):
        color = v(np.zeros(mask.shape), np.zeros(mask.shape), np.zeros(mask.shape))
        np.place(color.x, mask, self.x)
        np.place(color.y, mask, self.y)
        np.place(color.z, mask, self.z)
        return color
    def xyz(self):
        return self.x, self.y, self.z

def extract(Bool, obj):
    if isinstance(obj, numbers.Number):
        return obj
    else:
        return np.extract(Bool, obj)

class Sphere:
    def __init__(self, r, PosV, ColV, shininess, Reflected = 0.5, spec = v(1, 1, 1)):
        self.center = PosV
        self.r = r 
        self.Bcolor = ColV
        self.reflected = Reflected
        self.spec = spec
        self.shininess = shininess
        self.ambient = ColV * 0.1

    def update_pos(self, PosV):
        self.center = PosV

    def get_color(self, p_rf):
        return self.Bcolor

    def intersect(self, O, D):
        b = 2 * D.dot(O - self.center)
        c = self.center.abs() + O.abs() - 2 * self.center.dot(O) - (self.r * self.r)
        disc = (b ** 2) - (4 * c)
        sq = np.sqrt(np.maximum(0, disc))
        h0 = (-b - sq) / 2
        h1 = (-b + sq) / 2
        h = np.where((h0 > 0) & (h0 < h1), h0, h1)
        pred = (disc > 0) & (h > 0)
        return np.where(pred, h, to_far)

    def rgb(self, scene, origin, Vect, t, reflections, current_ir = 1):
        P_rf = origin + Vect*t
        N = (P_rf-self.center).normalized()
        L = (Light.P - P_rf).normalized()  
        I = (cam.pos - P_rf).normalized()
        color = self.ambient
        epsilon = 0.00001
        nudged = P_rf + L*epsilon

        intensity = np.maximum(N.dot(L), 0)
        shadow_rays = [s.intersect(nudged, L) for s in scene]
        nearest_intersection = reduce(np.minimum, shadow_rays)
        SeeLight = nearest_intersection > (Light.P - P_rf).magnitude()
        color += self.get_color(P_rf) * intensity* SeeLight 

        if reflections < 1 and self.reflected != 0:
            Rv = (Vect - N * 2 * Vect.dot(N)).normalized()
            r_term = self.reflected
            color += trace(scene, nudged, Rv, reflections + 1)*r_term #(self.reflected + (1 - self.reflected)*((1-dot_prod)**5))

        h = (L + I).normalized()
        color += self.spec * np.power(np.clip(N.dot(h), 0, 1), self.shininess/4) #* SeeLight
        return color


def trace(scene, origin, Vect, reflections = 0):
    t_vals = [s.intersect(origin, Vect) for s in scene]
    closests = reduce(np.minimum, t_vals)
    color = v(0, 0, 0)
    for (s, t) in zip(scene, t_vals):
        seen = (t == closests) & (closests != to_far)
        if np.any(seen):
            pt = extract(seen, t)
            pOr = origin.extract(seen)
            pVe = Vect.extract(seen)
            pC = s.rgb(scene, pOr, pVe, pt, reflections)
            color += pC.set_rgb(seen)
    return color

class Cam:
    def __init__(self, origin, V_N, width, aspect_ratio, FOV):
        self.origin = origin
        self.width = width
        self.r = aspect_ratio[0]/aspect_ratio[1]
        self.height = int(width/self.r)

        self.FOV = FOV

    def configure_cam(self):
        self.V_N = (earth.center - self.origin).normalized()
        Vx, Vy, Vz = self.V_N.xyz()
        theta = math.acos(Vx)
        deg = math.pi/2

        inner = math.cos(deg-math.acos(Vy)) if Vy < 0 else math.sin(deg-math.acos(Vy))

        rotationV = v(math.cos(deg-math.acos(Vx)), inner, 0)
        rotationV.normalized()
        

        tiltVz = math.sin(deg+math.asin(Vz))*(1/self.r)
        tiltVx = math.sqrt((1/self.r)**2 - tiltVz**2)*math.cos(theta)
        tiltVy = math.sqrt(1-tiltVx**2-tiltVz**2)

        tiltV = v(tiltVx, tiltVy, tiltVz)
        print(rotationV.xyz())
        print(rotationV.dot(self.V_N))
        topleft = tiltV - rotationV          + self.origin
        topright = tiltV + rotationV         + self.origin
        bottomleft = tiltV*(-1) - rotationV  + self.origin
        bottomright = tiltV*(-1) + rotationV + self.origin



        ox = np.linspace(topleft.x, bottomleft.x, self.height).reshape(-1, 1)
        oy = np.linspace(topleft.y, bottomleft.y, self.height).reshape(-1, 1)
        oz = np.linspace(topleft.z, bottomleft.z, self.height).reshape(-1, 1)

        tx = np.linspace(topright.x, bottomright.x, self.height).reshape(-1, 1)
        ty = np.linspace(topright.y, bottomright.y, self.height).reshape(-1, 1)
        tz = np.linspace(topright.z, bottomright.z, self.height).reshape(-1, 1)
        c = np.linspace(0, 1, self.width)
        self.x = ox + (tx - ox) * c
        self.y = oy + (ty - oy) * c
        self.z = oz + (tz - oz) * c

        dist = 1/math.tan(self.FOV/2)
        self.pos = self.V_N*-dist


    def render(self):
        self.configure_cam()
        Vect = (v(self.x, self.y, self.z) - self.pos).normalized()
        color = trace(scene, self.pos, Vect)
        rgb = [Image.fromarray((255*np.clip(c, 0, 1).reshape((height, width))).astype(np.uint8), "L") for c in color.xyz()]
        img = Image.merge("RGB", rgb).resize((int(width/2), int(height/2)))
        IMAGE = ImageTk.PhotoImage(image=img)

        label.config(image=IMAGE)
        label.pack()
        root.update()


Light.P = v(5, 5, 0)
to_far = 10.0**39

width = 400
aspect_ratio = 4, 4
r = aspect_ratio[0]/aspect_ratio[1]
height = int(width/r)

scene = [
    Sphere(2, v(15, 0, 0), v(1, 0.8, 0), 20, 0),

    Sphere(1, v(10, 0, 0), v(0, 0.3, 0.9), 100, 0, v(0,0,0))
]
sun, earth = scene

cam = Cam(v(0, 0, 0), v(1, 0, 0), 400, (4,4), math.pi/3)

root = tk.Tk()
label = tk.Label(root)
cam.render()
i=0
frames = 50
while True:
    i += 1
    theta = np.pi*2/frames
    Light.P = (cam.pos-sun.center).normalized()*(sun.r+1)
    earth.center = v(np.cos(theta*i)*5+15,np.sin(theta*i)*5, 0)
    cam.render()

root.bind('<Button-1>', lambda x : mouseclick(x, scene))
root.mainloop()

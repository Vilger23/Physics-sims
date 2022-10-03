import tkinter as tk
from functools import reduce
from PIL import Image, ImageTk
import math
from vectors import *
from test import *

def visual_setup():
    width = 300
    aspect_ratio = 4, 4
    r = aspect_ratio[0]/aspect_ratio[1]
    height = int(width/r)

    scene = [
        Sphere(2, v(0, 0, 0), v(10, 3, 0),   20, 0),

        Sphere(1, v(5, 0, 0), v(0, 0.3, 2), 100, 0, v(0,0,0))
        ]

    sun, earth = scene
    fibonacci_sphere(sun, scene, 8)
    cam = Cam(v(0, 0, 5), v(3, 0, 0), width, (4,4))
    return scene, cam
    visual_main_loop(scene, cam)

def main():
    scene, cam = visual_setup()
    root = tk.Tk()
    label = tk.Label(root)
    sun, earth = scene
    cam.render(scene, root, label)
    i=0
    frames = 50
    while True:
        i += 1
        theta = np.pi*2/frames
        earth.center = v(np.cos(0.2*theta*i)*23,np.sin(0.2*theta*i)*23, 0)
        cam.focus = (sun.center-earth.center)*(1/2)+earth.center
        cam.origin = cam.focus + v(0, 0, earth.r*20)
        cam.render(scene, root, label)

    #root.bind('<Button-1>', lambda x : mouseclick(x, scene))
    root.mainloop()

if __name__ == '__main__':
    main()
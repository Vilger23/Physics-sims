import numpy as np
import numbers

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
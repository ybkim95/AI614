import numpy as np
from node import Node

class Line:
    def __init__(self, p0, p1):
        if type(p0) == Node:
            p0 = (p0.x, p0.y)
        if type(p1) == Node:
            p1 = (p1.x, p1.y)
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn

class Line1():
    def __init__(self, p0, p1):
        if type(p0) == Node:
            p0 = (p0.x, p0.y)
        else:
            for i in p0:
                if type(i) == Node:
                    p0 = (i.x, i.y)
        if type(p1) == Node:
            p1 = (p1.x, p1.y)
        else:
            for i in p1:
                if type(i) == Node:
                    p1 = (i.x, i.y)

         
        self.p = np.array(p0, dtype=np.float32)
        self.dirn = np.array(p1, dtype=np.float32) - np.array(p0, dtype=np.float32)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn /= self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn
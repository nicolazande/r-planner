import math
import numpy as np
import os
import sys

sys.path.append(os.curdir)

import env

#definisco nodo
class Node:
    def __init__(self, S):
        self.S = np.array(S)
        self.delta = 0.
        self.parent = None

#uso per tutti i planner
class Utils:

    #costruttore utils
    def __init__(self, p_safe = 0):
        self.env = env.Env()
        self.delta = 1
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    #aggiorno ostacoli
    def update_obs(self, obs_cir, obs_bound, obs_rec):
        self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec

    #get vertici ostacoli
    def get_obs_vertex(self, node):
        delta = self.delta
        obs_list = []

        for (ox, oy, w, h) in self.obs_rectangle:
            vertex_list = [[ox - delta, oy - delta],
                           [ox + w + delta, oy - delta],
                           [ox + w + delta, oy + h + delta],
                           [ox - delta, oy + h + delta]]
            obs_list.append(vertex_list)

        return obs_list

    #controllo intersezioni in mezzo a X_near e X_new su cerchio
    def is_intersect_rec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]] #rotazione positiva pi/2 di vettore X_start-->X_goal
        div = np.dot(v2, v3) #area tra v3 su v2
        if div == 0: #v3==v2 --> non penetra
            return False
        t1 = np.linalg.norm(np.cross(v2, v1)) / div #prodotto_vettoriale / prodotto_scalare
        t2 = np.dot(v1, v3) / div
        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1])) #mod
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True
        return False

    #controllo intersezioni in mezzo a X_near e X_new su cerchio
    def is_intersect_circle(self, o, d, a, r):
        d2 = np.dot(d, d)
        if d2 == 0:
            return False
        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2
        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Node(a)) <= r:
                return True
        return False

    #check collisioni tra X_near e X_new
    def is_collision(self, start, end):
        #controllo se X_start e X_goal sono in collisione
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True
        #controllo se tra X_goal e X_start ci sono collisioni
        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex(start)
        for (v1, v2, v3, v4) in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True
        for (x, y, r) in self.obs_circle:
            if self.is_intersect_circle(o, d, [x, y, 0], r):
                return True
        
        return False

    #controllo se X_near e X_new sono all'interno di Ostacoli
    def is_inside_obs(self, node):        
        delta = self.delta

        for (x, y, r) in self.obs_circle:
            if math.hypot(node.S[0] - x, node.S[1] - y) <= r + delta:
                return True

        for (x, y, w, h) in self.obs_rectangle:
            if 0 <= node.S[0] - (x - delta) <= w + 2 * delta \
                    and 0 <= node.S[1] - (y - delta) <= h + 2 * delta:
                return True

        return False


    #get direzione tra near e next
    @staticmethod
    def get_ray(start, end):
        orig = [start.S[0], start.S[1]]
        direc = [end.S[0] - start.S[0], end.S[1] - start.S[1]]
        return orig, direc

    #get distanza tra due punti
    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.S[0] - start.S[0], end.S[1] - start.S[1])

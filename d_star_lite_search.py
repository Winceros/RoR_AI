__author__ = 'Ildar_Gilmutdinov'

__author__ = 'Ildar_Gilmutdinov'

import numpy as np
from heapq import *
from math import atan,atan2,pi,degrees,radians,tan,cos,sin,ceil

class DstarLiteSearch:

    cost = 0
    gx,gy = 0,0
    start,goal = 0,0
    x_len,y_len = 0,0

    def __init__(self,start,goal,costmap,gx,gy):
        self.cost = costmap
        self.gx, self.gy = gx,gy
        self.x_len, self.y_len = costmap.shape
        self.start = start
        self.goal = goal

    def g(self,node,cost):
        #x,y = round(node[0]),round(node[1])
        x,y = node[0],node[1]
        yaw = radians(node[2])
        if self.cost[x][y] > tan(radians(30)):
            return 1000000
        # what coefficient should be there?
        yaw_v = np.array([np.cos(yaw),np.sin(yaw)])
        grad_v = np.array([self.gx[x][y],self.gy[x][y]])
        cos_proj_angle = np.vdot(np.array([np.cos(yaw),np.sin(yaw)]),np.array([self.gx[x][y],self.gy[x][y]]))\
                        / ( (cos(yaw)**2 + sin(yaw)**2)**0.5 * (self.gx[x,y]**2 + self.gy[x,y]**2)**0.5 )
        cost_proj = self.cost[x][y] * cos_proj_angle
        #cost_proj = self.cost[x][y]
        g = cost + 5*cost_proj
        return g

    def h(self,node):
        h = ((node[0]-self.goal[0])**2+(node[1]-self.goal[1])**2)**0.5
        return h


    def get_successors(self,state):

        moves_yaw = np.array([0,-45,45]) # fi steps
        states = np.ndarray(shape=(3,3))
        states[:,0] = np.array([1,0,1])
        states[:,1] = np.array([1,-1,1])
        states[:,2] = np.array([1,1,1])
        #print states

        translate_mat = np.mat('1 0 {}; 0 1 {}; 0 0 1'.format(state[0],state[1]))

        for state2,yaw in zip(states.T,moves_yaw):
            #rot_mat = np.mat('{} {} 0; {} {} 0; 0 0 1'.format(np.cos(radians(yaw)),-np.sin(radians(yaw)),np.sin(radians(yaw)),np.cos(radians(yaw))))
            #transform_mat = translate_mat.dot(rot_mat)
            #transform_mat = rot_mat.dot(translate_mat)

            new_state = np.array([state2[0],state2[1],1])
            rot_mat = np.mat('{} {} 0; {} {} 0; 0 0 1'.format(np.cos(radians(state[2])),\
                                                              -np.sin(radians(state[2])),\
                                                              np.sin(radians(state[2])),\
                                                              np.cos(radians(state[2]))))
            new_state = rot_mat.dot(new_state)
            new_state = translate_mat.dot(new_state.T)

            new_state = new_state.tolist()
            posx = round(new_state[0][0])
            posy = round(new_state[1][0])
            if posx<0 or posx>=self.x_len or posy<0 or posy>=self.y_len:
                continue
            else:
                #yield (posx,posy,(yaw+state[2])%360,(state2[0],state2[1],yaw))
                yield (posx,posy,(yaw+state[2])%360)


    def d_star_lite_search(self):
        path = [self.start]
        queue = []
        heappush(queue, ( self.h(self.start) + self.g(self.start,0) , (path,0)))
        closed_set = set()
        i = 0
        while True:
            f_val,(path,cost) = heappop(queue)
            if cost>=1000000:
                print 'fail'
                break
            node = path[-1]
            i += 1
            #print i
            if abs(node[0]-self.goal[0])<=1 and abs(node[1]-self.goal[1])<=1:# or i>3000:
                for x in path:
                    print x
                print len(path)-1
                print cost
                return path

            if (node[0],node[1]) not in closed_set:
                closed_set.add((node[0],node[1]))
                for succ in self.get_successors(node):
                    if succ not in path:
                        path_new = path[:] + [succ]
                        g_val = self.g(succ,cost)
                        heappush(queue, ( self.h(succ) + g_val , (path_new,g_val)))

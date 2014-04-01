import numpy as np
from mayavi import mlab
from matplotlib import cm
import random
from heapq import *
from math import atan,atan2,pi,degrees,radians,tan,cos,sin,ceil
import sys

class PlannerAstar:

    x,y,z,yaw = 0,0,0,0
    gx,gy,Gt = 0,0,0
    s0,sg = 0,0
    x_len, y_len = 0,0
    path = []

    def readData(self,filename,step = 1):
        with open(filename) as f:
            x_len,y_len,step = map(int,f.readline().split(','))
            self.z = np.zeros(shape=(x_len,y_len))
            for i in xrange(0,x_len):
                data = f.readline()
                data = data[1:]
                data = map(float,data.split(','))
                self.z[i:,] = data
            self.z = self.z.transpose()
            self.x,self.y = np.mgrid[0:x_len:1,0:x_len:1]
            self.x_len, self.y_len = x_len, y_len
        #f = open('C:\Users\Ildar_Gilmutdinov\Documents\Python projects\RoR_AI\sdfanumpy_map.txt','w')
        #sys.stdout = f
        #print self.z
        #for i in xrange(0,x_len*x_len,x_len):
        #    print data[i:i+x_len]
        #sys.stdout = sys.__stdout__
        #f.close()
        return step

    def setStartAndGoal(self,xs,ys,yaw,goal = None):
        self.s0 = (xs,ys,yaw)
        if goal == None:
            self.sg = (self.x_len-5,30)
        else:
            self.sg = goal

    def calcGradient(self):
        self.gx, self.gy = np.gradient(self.z)
        self.Gt = (self.gx**2 + self.gy**2)**0.5

    def g(self,node,cost):
        #x,y = round(node[0]),round(node[1])
        x,y = node[0],node[1]
        if self.gx[x][y]>tan(radians(30)) or self.gy[x][y]>tan(radians(30)):
            return 1000000
        g = cost + 10*self.Gt[x][y]
        return g

    def h(self,node):
        h = ((node[0]-self.sg[0])**2+(node[1]-self.sg[1])**2)**0.5
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
            rot_mat = np.mat('{} {} 0; {} {} 0; 0 0 1'.format(np.cos(radians(state[2])),-np.sin(radians(state[2])),np.sin(radians(state[2])),np.cos(radians(state[2]))))
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


    def a_star_search(self,start,goal,get_successors):
        path = [start]
        queue = []
        heappush(queue, ( self.h(start) + self.g(start,0) , (path,0)))
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
            if abs(node[0]-goal[0])<=1 and abs(node[1]-goal[1])<=1:# or i>3000:
                for x in path:
                    print x
                print len(path)-1
                return path

            if node not in closed_set:
                closed_set.add(node)
                for succ in self.get_successors(node):
                    if succ not in path:
                        path_new = path[:] + [succ]
                        g_val = self.g(succ,cost)
                        heappush(queue, ( self.h(succ) + g_val , (path_new,g_val)))

    def planPath(self):
        self.calcGradient()
        self.path = self.a_star_search(self.s0,self.sg,self.get_successors)
        return self.path

    def showPath(self):
        px, py, pz, dirx,diry,dirz = [],[],[],[],[],[]

        for node in self.path:
            px.append(node[0])
            py.append(node[1])
            pz.append(self.z[node[0]][node[1]])
            dirx.append(cos(radians(node[2])))
            diry.append(sin(radians(node[2])))
            dirz.append(0)

        px = np.array(px)
        py = np.array(py)
        pz = np.array(pz)
        dirx = np.array(dirx)
        diry = np.array(diry)
        dirz = np.array(dirz)

        mlab.quiver3d(px,py,pz,dirx,diry,dirz,color=(0,0,0),scale_factor=1)

        self.calcGradient()

        #mlab.points3d(px,py,pz,color=(0,0,0),scale_factor=1)
        mlab.quiver3d(self.x,self.y,self.z,self.gx,self.gy,self.Gt,color=(1,0,0),scale_factor=1)

        mlab.surf(self.x, self.y, self.z, representation='wireframe')
        mlab.show()
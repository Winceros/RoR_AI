import numpy as np
from mayavi import mlab
from matplotlib import cm
import random
from heapq import *
from math import atan,atan2,pi,degrees,radians,tan,cos,sin,ceil
import sys
from a_star_search import AstarSearch

class GlobalPlanner:

    x,y,z,yaw = 0,0,0,0
    Gt = 0
    roughMap = 0
    gx,gy = 0,0
    s0,sg = 0,0
    x_len, y_len = 0,0
    path = []

    def readDataFromTxt(self,filename,step = 1):
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

    def readDataFromRaw(self,filename,step = 5):
        map = np.fromfile(filename,dtype=np.uint16)
        map.shape = (513,513)
        maxheight = 62
        self.x_len = self.y_len = len(map)
        self.z = map[0:self.x_len:step,0:self.y_len:step].astype(float)
        self.x,self.y = np.mgrid[0:self.x_len/step + 1,0:self.y_len/step + 1]
        self.z /= (self.z.max()/maxheight)*step
        return step

    def setStartAndGoal(self,xs,ys,yaw,goal = None):
        self.s0 = (xs,ys,yaw)
        if goal == None:
            #self.sg = (self.x_len-5,30)
            self.sg = (95,33)
            #self.sg = (5,80)
        else:
            self.sg = goal

    def calcGradient(self):
        self.gx, self.gy = np.gradient(self.z)
        self.Gt = (self.gx**2 + self.gy**2)**0.5


    def calcRoughness(self):
        step = 5
        self.roughMap = np.ndarray(shape=(self.x_len/step+1,self.x_len/step+1))
        for i in xrange(0,self.x_len,5):
            for i2 in xrange(0,self.x_len,5):
                self.roughMap[i/5,i2/5] = np.sum(self.Gt[i:i+5,i2:i2+5])
        return self.roughMap

    def planPath(self):
        self.calcGradient()
        self.calcRoughness()
        aStarSearch = AstarSearch(self.s0,self.sg,self.Gt,self.gx,self.gy,self.roughMap)
        #while True:
        #    if costs changed : d star
        self.path = aStarSearch.a_star_search()
        return self.path

    def showPath(self):
        px, py, pz, dirx,diry,dirz = [],[],[],[],[],[]

        print self.path
        print len(self.z)

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
        mlab.quiver3d(self.x,self.y,self.z,self.gx,self.gy,self.Gt,color=(1,0,0),scale_factor=1)
        mlab.quiver3d(px,py,pz,dirx,diry,dirz,color=(0,0,0),scale_factor=1)

        mlab.surf(self.x, self.y, self.z, representation='wireframe')
        mlab.show()
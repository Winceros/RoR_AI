__author__ = 'Ildar_Gilmutdinov'

from math import atan2,sin,cos,degrees,pi,radians,tan

class Executor:
    path = []
    maxvelo = 0
    cur_subg = 0
    subg_gen = 0
    step = 0
    alpha0 = 0
    psi0 = 0
    psidotlist = []

    def __init__(self,path,step,maxvelo,alpha0):
        self.path = path
        self.maxvelo = maxvelo
        self.step = step
        self.alpha0 = alpha0
        self.psidotlist = []
        self.subg_gen = (subg for subg in self.path)
        self.cur_subg = self.subg_gen.next()

    def dist(self,x0,y0,x1,y1):
        return ((x1-x0)**2+(y1-y0)**2)**0.5

    def getSubGoalState(self,state):
        x = 1.0*state['x']/self.step
        y = 1.0*state['z']/self.step
        xg = self.cur_subg[0]
        yg = self.cur_subg[1]
        alpha_desired = degrees( atan2( (yg-y),(xg-x+0.00000001) ) )
        if alpha_desired < 0 :
            alpha_desired = 180 + (180-(-alpha_desired))
        print 'cur yaw =',alpha_desired
        alpha = state['yaw']+90
        if alpha < 0 :
            alpha = 270 + (90-(-alpha))
        print 'yaw desired =',alpha

        while self.dist(x,y,xg,yg) <=1 or (self.dist(x,y,xg,yg) <=3 and abs(alpha - alpha_desired) > 90  ):
            try:
                self.cur_subg = self.subg_gen.next()
            except StopIteration:
                return (0.0,0.0,0.0)
            xg = self.cur_subg[0]
            yg = self.cur_subg[1]
            alpha_desired = degrees( atan2( (yg-y),(xg-x+0.00000001) ) )
            if alpha_desired < 0 :
                alpha_desired = 180 + (180-(-alpha_desired))
        return xg,yg,alpha_desired

        # dx/dt = v * cos(fi+psi)
        # dy/dt = v * sin(fi+psi)
        # dfi/dt = v/L * sin(psi)
        # dpsi/dt = u
        # inputs : v,u
        #
        # x = x0 + dx/dt*dt
        # y = y0 + dy/dt*dt
        # fi = fi0 + dfi/dt*dt
        # psi = psi0 + dpsi/dt*dt
        #
        # x = x0 + v * cos(fi+psi) * dt
        # y = y0 + v * sin(fi+psi) * dt
        # fi = fi0 + v/L * sin(psi) * dt
        # psi = psi0 + dpsi/dt*dt

    def goToGoal(self,state):

        xg,yg,alpha_desired = self.getSubGoalState(state)
        alpha = state['yaw']+90
        if alpha < 0 :
            alpha = 270 + (90-(-alpha))
        v = state['v']

        w = alpha - self.alpha0
        self.alpha0 = alpha

        #psi_dot = state['psi'] - self.psi0
        #self.psidotlist.append(psi_dot)
        #self.psi0 = state['psi']

        if v < self.maxvelo:
            u_a = 1.0
        else:
            u_a = 0.0

        if abs((alpha + w) - alpha_desired) < 2:
            u_fi = 0.0
        else:
            #u_a = 0.01
            if alpha > alpha_desired:
                if alpha - alpha_desired > 180:
                    u_fi = -1.0
                else:
                    u_fi = 1.0
            else:
                if alpha_desired - alpha > 180:
                    u_fi = 1.0
                else:
                    u_fi = -1.0

        return u_a, u_fi

    def obstacleAvoidance(self,state):
        alpha = state['yaw']+90
        if alpha < 0 :
            alpha = 270 + (90-(-alpha))
        v = state['v']

        if v < self.maxvelo:
            u_a = 1.0
        else:
            u_a = 0.0

        threatDeg = state['threatDeg']
        if threatDeg > 0:
            u_fi = -1.0
        else:
            u_fi = 1.0

        return u_a, u_fi

    def getControlVector(self,state):

        if state['threatDist'] < 0:
            return self.obstacleAvoidance(state)
        else:
            return self.goToGoal(state)
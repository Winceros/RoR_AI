import time
import sys
import random
import zmq
import global_planner_a_star
import local_planner
import math

class Server:
    context = 0
    sock = 0
    state = {}
    path = []
    executor = 0

    def logPlannedPath(self,step):
        f = open('planned_path.txt','w')
        sys.stdout = f
        print step
        for node in self.path:
            print '{},{}'.format(node[1],node[0])
        sys.stdout = sys.__stdout__
        f.close()

    def __init__(self,host,port):
        self.context = zmq.Context()
        self.sock = self.context.socket(zmq.REP)
        self.sock.bind('tcp://'+host+':'+str(port))
        print 'Listening port :',port

    def close(self):
        return 0

    def caseRequest(self,request):
        if request == 'plan_path':
            self.planGlobalPath()
        elif request == 'take_cur_state_and_give_control_vector':
            self.receiveCurStateAndSendControlVector()

    def listenRequest(self):
        while True:
            request = self.sock.recv()
            self.caseRequest(request)

    def planGlobalPath(self):
        print '----PLANNING-PATH-----------'
        self.sock.send('ok')
        msg = self.sock.recv()
        # !!!!
        z_start,y_start,x_start,yaw,pitch,roll,psi = map(float,msg.split(','))
        yaw += 90
        if yaw < 0 :
            yaw = 270 + (90-(-yaw))
        planner = global_planner_a_star.PlannerAstar()
        step = planner.readData('map.txt')
        z_start,x_start = int(z_start/step),int(x_start/step)
        planner.setStartAndGoal(x_start,z_start,yaw)
        #self.path = planner.planPath()
        #planner.showPath()
        # for DEBUG :
        #self.path = [(x_start-2,z_start+2)]
        #self.path = [(x_start+2,z_start+2)]
        self.path = [(x_start-4,z_start+4)]
        #self.path = [(x_start+2,z_start-2),(x_start+2 -2,z_start-2 -2)]
        #self.path = [(x_start,z_start),(x_start+2,z_start+2)]
        #self.path = self.path[1::4]
        self.executor = local_planner.Executor(self.path,step,15,yaw)
        self.logPlannedPath(step)
        self.sock.send('ok')
        print '----PLANNED-----------------'

    def sendControlVector(self):
        print '----SENDING-CONTROL-VECTOR--'
        u_a, u_psi = self.executor.getControlVector(self.state)
        msg = '({0},{1})'.format(u_a,u_psi)
        self.sock.send('({0},{1})'.format(u_a,u_psi))
        print 'sent ',msg
        print '----SENT--------------------'

    def receiveCurState(self):
        print '----RECEIVING-CUR-STATE-----'
        self.sock.send('ok')
        msg = self.sock.recv()
        x,y,z,yaw,pitch,roll,v,psi,threatDeg,threatDist = map(float,msg.split(','))
        v = int(3.6*v*0.621371192)
        self.state['x'] = z # !!!!
        self.state['y'] = y
        self.state['z'] = x # !!!!
        self.state['yaw'] = yaw
        self.state['pitch'] = pitch
        self.state['roll'] = roll
        self.state['v'] = v
        self.state['psi'] = psi
        self.state['threatDeg'] = threatDeg
        self.state['threatDist'] = threatDist
        print 'pos = ({},{},{})'.format(x,y,z)
        print 'orientation = ({},{},{})'.format(yaw,pitch,roll)
        print 'speed = {}'.format(v)
        if threatDist > 0:
            print 'threatDist :',threatDist,'threatDeg :',threatDeg
        print '----RECEIVED----------------'

    def receiveCurStateAndSendControlVector(self):
        self.receiveCurState()
        self.sendControlVector()

srv = Server('127.0.0.1',1111)
srv.listenRequest()


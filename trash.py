__author__ = 'Ildar_Gilmutdinov'
import global_planner_a_star

import sys
from mayavi import mlab
import numpy as np

'''
with open('C:\Users\Ildar_Gilmutdinov\Documents\Python projects\RoR_AI\path_log.txt') as f:
    data = f.readline()
    data = data[:-1]
    data = map(float,data.split(','))

    fringe = []
    for i in range(0,len(data),2):
        fringe.append((data[i]/5,data[i+1]/5))

    step = planner.readData('C:\Users\Ildar_Gilmutdinov\Documents\Python projects\RoR_AI\map.txt')
    planner.fringe = fringe
    planner.showPath()
'''

planner = global_planner_a_star.PlannerAstar()
planner.readData('C:\Users\Ildar_Gilmutdinov\Documents\Python projects\RoR_AI\map.txt')
planner.setStartAndGoal(3,3,45)
planner.planPath()
planner.showPath()
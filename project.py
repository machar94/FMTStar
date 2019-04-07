#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import math
import IPython
import matplotlib.pyplot as plt
import os

##### Parameters #####
WORLD    = [-3.4, 3.4, -1.4, 1.4]
SAMPLES  = 200  
RADIUS   = 1.0
STEPSIZE = 0.1

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


def tuckarms(env, robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def placeRobot(env, robot, place):
    state = numpy.eye(4)
    state[0:2,3] = place

    with env:
        robot.SetTransform(state)


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    env.Load('data/mp_env.env.xml')
    time.sleep(0.1)
    robot = env.GetRobots()[0]
    tuckarms(env,robot);
        
    # The active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
    # robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y)
        
    state = robot.GetTransform()
    yaw = numpy.array([math.atan2(state[1,0], state[0,0])])
    startConfig = robot.GetTransform()[0:2,3].tolist()
    goalConfig  = [2.6,-1.3]

    RaveLoadPlugin('fmt/build/fmt')
    FMTPlanner = RaveCreateModule(env, 'FMT')
    env.AddModule(FMTPlanner,args='')
    
    startConfigStr = ' '.join([str(e) for e in startConfig])
    goalConfigStr  = ' '.join([str(e) for e in goalConfig])
    worldStr       = ' '.join([str(e) for e in WORLD])

    with env:
        FMTPlanner.SendCommand('init') 
        FMTPlanner.SendCommand('SetStart ' + startConfigStr)
        FMTPlanner.SendCommand('SetGoal ' + goalConfigStr)
        FMTPlanner.SendCommand('DefineWorld ' + worldStr)
        FMTPlanner.SendCommand('SetNumSamples ' + str(SAMPLES))
        FMTPlanner.SendCommand('SetRadius ' + str(RADIUS)) 
        FMTPlanner.SendCommand('SetStepSize ' + str(STEPSIZE)) 
        # FMTPlanner.SendCommand('PrintClass')
        FMTPlanner.SendCommand('Run')
        
    waitrobot(robot)

    raw_input("Press enter to exit...")

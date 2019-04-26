#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import math
import IPython
import matplotlib.pyplot as plt
import os
import numpy as np
import math

##### Parameters #####
WORLD    = [-8.4, 8.4, -2.4, 2.4] # Position
SAMPLES  = 2000 
RADIUS   = 0.5
STEPSIZE = 0.1
SEED     = 1
PLANNER  = "smart"
FWD_COLLISION_CHECK = 1
# TRIGGER1 = "2.0 Table3 3.1 0.1 90 Table4 4.2 0.0 0"
# TRIGGER2 = "-2.0 Table1 -0.3 0.7 90 Table2 0.5 -1.2 0 "
GOAL_CONFIG  = [math.radians(-90), 7.0, 2.3]
FILENAME = "stats.txt"

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def writeToFile(timep1, reuse2, timep2, reuse3, timep3):
    f_handle = file(FILENAME, 'a')
    np.savetxt(f_handle, timep1, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, timep2, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, timep3, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, reuse2, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, reuse3, delimiter=',', fmt='%1.2f')

    try:
        os.remove(FILENAME)
    except OSError:
        pass


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


def tuckarms(env, robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996])
        robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)


def placeRobot(env, robot, place):
    with env:
        robot.SetTransform(place)


def resetEnv(env, tables):
    with env:
        env.GetKinBody('Table1').SetTransform(tables[0])
        env.GetKinBody('Table2').SetTransform(tables[1])
        env.GetKinBody('Table3').SetTransform(tables[2])
        env.GetKinBody('Table4').SetTransform(tables[3])


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    env.Load('./envs/large-chamber.env.xml')
    time.sleep(0.1)
    robot = env.GetRobots()[0]
    
    RaveLoadPlugin('fmt/build/fmt')
    FMTPlanner = RaveCreateModule(env, 'FMT')
    env.AddModule(FMTPlanner,args='')
    tuckarms(env,robot);

    # The active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
    # robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
    jointnames = ['r_shoulder_pan_joint']
    jointindices = [robot.GetJoint(name).GetDOFIndex() for name in jointnames]
    # print(jointindices)
    robot.SetActiveDOFs(jointindices,DOFAffine.X|DOFAffine.Y)
    startConfig = [math.radians(-90), -8.4, -2.4] # r_shoulder, x, y
    robot.SetActiveDOFValues(startConfig)
    robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    # print('Active DOF: ', robot.GetActiveDOF())

    # tables = []
    # tables.append(env.GetKinBody('Table1').GetTransform())
    # tables.append(env.GetKinBody('Table2').GetTransform())
    # tables.append(env.GetKinBody('Table3').GetTransform())
    # tables.append(env.GetKinBody('Table4').GetTransform())
    # startConfig = robot.GetTransform()[0:2,3].tolist()
    # startConfig.append(robot.GetJoint('r_shoulder_pan_joint').GetValues()[0])
    startConfigStr = ' '.join([str(e) for e in startConfig])

    # first 0, 1 are lower and upper limits
    # second 0 are the limits or r_shoulder
    WORLD.insert(0, robot.GetActiveDOFLimits()[0][0])
    WORLD.insert(1, robot.GetActiveDOFLimits()[1][0])
    worldStr       = ' '.join([str(e) for e in WORLD])
    
    goalConfigStr  = ' '.join([str(e) for e in GOAL_CONFIG])

    with env:
        FMTPlanner.SendCommand('init') 
        FMTPlanner.SendCommand('SetStart ' + startConfigStr)
        FMTPlanner.SendCommand('SetGoal ' + goalConfigStr)
        FMTPlanner.SendCommand('DefineWorld ' + worldStr)
        FMTPlanner.SendCommand('SetNumSamples ' + str(SAMPLES))
        FMTPlanner.SendCommand('SetRadius ' + str(RADIUS)) 
        FMTPlanner.SendCommand('SetStepSize ' + str(STEPSIZE)) 
        FMTPlanner.SendCommand('SetPlanner ' + PLANNER) 
        FMTPlanner.SendCommand('SetFwdCollisionCheck ' + str(FWD_COLLISION_CHECK))
        FMTPlanner.SendCommand('SetSeed ' + str(SEED)) 
        # FMTPlanner.SendCommand('CreateTrigger ' + str(TRIGGER1))
        # FMTPlanner.SendCommand('CreateTrigger ' + str(TRIGGER2))
        FMTPlanner.SendCommand('PrintClass')
        # FMTPlanner.SendCommand('Run')
    result = FMTPlanner.SendCommand('RunWithReplan')

    waitrobot(robot)

    raw_input("Press enter to exit...")

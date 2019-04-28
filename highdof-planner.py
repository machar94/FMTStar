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
SAMPLES  = 4000 
RADIUS   = 0.5
STEPSIZE = 0.1
SEED     = 108
PLANNER  = "naive"
FWD_COLLISION_CHECK = 1
# TRIGGER1 = "2.0 Table3 3.1 0.1 90 Table4 4.2 0.0 0"
# TRIGGER2 = "-2.0 Table1 -0.3 0.7 90 Table2 0.5 -1.2 0 "
TRIGGER3 = "-8.0 Table1 -7.0 -1.0 0 Table2 -4.5 .9 90"
TRIGGER2 = "-3.0 Table3 1 -1.7 0 Table4 -0.5 -1.0 90"
TRIGGER1 =  "3.0 Table6 4.0 -0.5 90 Table5 5.5 1.6 90"
GOAL_CONFIG  = [math.radians(-90), math.radians(90), 7.0, 2.3]
FILENAME = "stats.txt"

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def writeToFile(sn1, st1, pt1, pn1, sn2, st2, pt2, pn2, sn3, st3, pt3, pn3, sn4, st4, pt4, pn4):
    try:
        os.remove(FILENAME)
    except OSError:
        pass
    
    f_handle = file(FILENAME, 'a')
    np.savetxt(f_handle, sn1, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, st1, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, pt1, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, pn1, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, sn2, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, st2, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, pt2, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, pn2, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, sn3, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, st3, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, pt3, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, pn3, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, sn4, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, st4, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, pt4, delimiter=',', fmt='%1.2f')
    np.savetxt(f_handle, pn4, delimiter=',', fmt='%1.2f')


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
        env.GetKinBody('Table5').SetTransform(tables[4])
        env.GetKinBody('Table6').SetTransform(tables[5])


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
    jointnames = ['r_shoulder_pan_joint', 'l_shoulder_pan_joint']
    jointindices = [robot.GetJoint(name).GetDOFIndex() for name in jointnames]
    print(jointindices)
    robot.SetActiveDOFs(jointindices,DOFAffine.X|DOFAffine.Y)
    startConfig = [math.radians(-90), math.radians(90), -8.4, -2.4] # r_shoulder, x, y
    robot.SetActiveDOFValues(startConfig)
    robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    # print('Active DOF: ', robot.GetActiveDOF())

    robStartState = robot.GetTransform()
    tables = []
    tables.append(env.GetKinBody('Table1').GetTransform())
    tables.append(env.GetKinBody('Table2').GetTransform())
    tables.append(env.GetKinBody('Table3').GetTransform())
    tables.append(env.GetKinBody('Table4').GetTransform())
    tables.append(env.GetKinBody('Table5').GetTransform())
    tables.append(env.GetKinBody('Table6').GetTransform())

    # startConfig = robot.GetTransform()[0:2,3].tolist()
    # startConfig.append(robot.GetJoint('r_shoulder_pan_joint').GetValues()[0])
    startConfigStr = ' '.join([str(e) for e in startConfig])

    # first 0, 1 are lower and upper limits
    # second 0 are the limits or r_shoulder_pan
    # third 1 are the limits of l_shoulder_pan
    WORLD.insert(0, robot.GetActiveDOFLimits()[0][0])
    WORLD.insert(1, robot.GetActiveDOFLimits()[1][0])
    WORLD.insert(2, robot.GetActiveDOFLimits()[0][1])
    WORLD.insert(3, robot.GetActiveDOFLimits()[1][1])
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
        # FMTPlanner.SendCommand('SetSeed ' + str(SEED)) 
        # FMTPlanner.SendCommand('CreateTrigger ' + str(TRIGGER1))
        # FMTPlanner.SendCommand('CreateTrigger ' + str(TRIGGER2))
        # FMTPlanner.SendCommand('CreateTrigger ' + str(TRIGGER3))
        # FMTPlanner.SendCommand('PrintClass')
        # FMTPlanner.SendCommand('Run')
    # result = FMTPlanner.SendCommand('RunWithReplan')

    file = open('stats.txt','w')
    otherFile = open('otherFile.txt', 'w')

    # startNodes1 = np.empty(shape=[1,0])
    # sampleTime1 = np.empty(shape=[1,0])
    # planTime1 = np.empty(shape=[1,0])
    # pathNodes1 = np.empty(shape=[1,0])
    # startNodes2 = np.empty(shape=[1,0])
    # sampleTime2 = np.empty(shape=[1,0])
    # planTime2 = np.empty(shape=[1,0])
    # pathNodes2 = np.empty(shape=[1,0])
    # startNodes3 = np.empty(shape=[1,0])
    # sampleTime3 = np.empty(shape=[1,0])
    # planTime3 = np.empty(shape=[1,0])
    # pathNodes3 = np.empty(shape=[1,0])
    # startNodes4 = np.empty(shape=[1,0])
    # sampleTime4 = np.empty(shape=[1,0])
    # planTime4 = np.empty(shape=[1,0])
    # pathNodes4 = np.empty(shape=[1,0])

    numLoops = 0
    while (numLoops < 13):
        placeRobot(env, robot, robStartState)
        resetEnv(env, tables)

        print "Working on replan: " + str(numLoops + 1)
        print "\nSeed: %d\n" % SEED
        FMTPlanner.SendCommand('SetSeed ' + str(SEED))
        FMTPlanner.SendCommand('SetNumSamples ' + str(SAMPLES))
        FMTPlanner.SendCommand('CreateTrigger ' + str(TRIGGER1))
        FMTPlanner.SendCommand('CreateTrigger ' + str(TRIGGER2))
        FMTPlanner.SendCommand('CreateTrigger ' + str(TRIGGER3))
        result = FMTPlanner.SendCommand('RunWithReplan')
        SEED = SEED + 1
        data = [double(val) for val in result.split()]
        if (len(data)) == 16:
            file.write(str(SEED) + ' ' + result + '\n')
            numLoops = numLoops + 1
        else:
            print "Skipping Seed: " + str(SEED - 1)
            otherFile.write(str(SEED) + ' ' + result + '\n')
        

    #     print(data)

    #     if len(data) == 16:
    #         startNodes1 = np.append(startNodes1, [[data[0]]], axis=1)
    #         sampleTime1 = np.append(sampleTime1, [[data[1]]], axis=1)
    #         planTime1   = np.append(planTime1,   [[data[2]]], axis=1)
    #         pathNodes1  = np.append(pathNodes1,  [[data[3]]], axis=1)
    #         startNodes2 = np.append(startNodes2, [[data[4]]], axis=1)
    #         sampleTime2 = np.append(sampleTime2, [[data[5]]], axis=1)
    #         planTime2   = np.append(planTime2,   [[data[6]]], axis=1)
    #         pathNodes2  = np.append(pathNodes2,  [[data[7]]], axis=1)
    #         startNodes3 = np.append(startNodes3, [[data[8]]], axis=1)
    #         sampleTime3 = np.append(sampleTime3, [[data[9]]], axis=1)
    #         planTime3   = np.append(planTime3,   [[data[10]]], axis=1)
    #         pathNodes3  = np.append(pathNodes3,  [[data[11]]], axis=1)
    #         startNodes4 = np.append(startNodes4, [[data[12]]], axis=1)
    #         sampleTime4 = np.append(sampleTime4, [[data[13]]], axis=1)
    #         planTime4   = np.append(planTime4,   [[data[14]]], axis=1)
    #         pathNodes4  = np.append(pathNodes4,  [[data[15]]], axis=1)
    #         numLoops = numLoops + 1
    #         print "Working on replan: " + str(numLoops)
        
    #     else:
    #         print "Skipping Seed: " + str(SEED - 1)

    # writeToFile(startNodes1, sampleTime1, planTime1, pathNodes1, startNodes2, sampleTime2, planTime2, pathNodes2, startNodes3, sampleTime3, planTime3, pathNodes3, startNodes4, sampleTime4, planTime4, pathNodes4) 

    file.close()
    otherFile.close()

    waitrobot(robot)

    raw_input("Press enter to exit...")

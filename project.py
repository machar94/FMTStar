#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import math
import IPython
import matplotlib.pyplot as plt
import os

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)


def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)
    robot = env.GetRobots()[0]
    tuckarms(env,robot);
        
    # The active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        
    state = robot.GetTransform()
    yaw = numpy.array([math.atan2(state[1,0], state[0,0])])
    startConfig = robot.GetTransform()[0:2,3].tolist()
    startConfig.append(yaw[0])
    goalConfig  = [2.6,-1.3, 3*math.pi/2]

    RaveLoadPlugin('fmt/build/fmt')
    FMTPlanner = RaveCreateModule(env, 'FMT')
    env.AddModule(FMTPlanner,args='')

    with env:
        FMTPlanner.SendCommand('init')      
        
    waitrobot(robot)

    raw_input("Press enter to exit...")

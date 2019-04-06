#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/fmt')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    FMT = RaveCreateModule(env,'FMT')
    print FMT.SendCommand('help')
finally:
    RaveDestroy()

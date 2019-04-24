import pandas as pd
from pandas import DataFrame
import pylab
import matplotlib.pyplot as plot
pd.set_option('display.width',200)

simtime = pd.read_csv("test.txt", header=None, prefix="V")
simtime.rows = ['Plan 1 Time (ms)', 'Plan 2 Reused Nodes', 
                   'Plan 2 Time (ms)', 'Plan 3 Reused Nodes',
                   'Plan 3 Time (ms)']

print(simtime.transpose())
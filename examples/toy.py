import imp
import numpy as np
# Utility functions to initialize the problem
from odp.Grid import Grid
from odp.Shapes import *

# Specify the  file that includes dynamic systems
from odp.dynamics import DubinsCapture
from odp.dynamics import DubinsCar4D2
from odp.dynamics import DubinsCar
# Plot options
from odp.Plots import PlotOptions
from odp.Plots import plot_isosurface, plot_valuefunction

# Solver core
from odp.solver import HJSolver, computeSpatDerivArray


import math

""" USER INTERFACES
- Define grid
- Generate initial values for grid using shape functions
- Time length for computations
- Initialize plotting option
- Call HJSolver function
"""


# Third scenario with Reach-Avoid set
g = Grid(np.array([-1.1, -1.1, -math.pi]), np.array([1.1, 1.1, math.pi]), 3, np.array([101, 101, 101]), [2])

# Reachable set
#goal = CylinderShape(g, [2], np.zeros(3), 0.1)
goal = learnedShape(g)

# Look-back length and time step
lookback_length = 100*0.05 #1.5
t_step = 0.05

small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

my_car = DubinsCar(uMode="max", dMode="min", wMax = 1.25, speed=1.0)


po2 = PlotOptions(do_plot=True, plot_type="set", plotDims=[0,1,2],
                  slicesCut=[], save_fig=True, filename='plot.png')

"""
Assign one of the following strings to `TargetSetMode` to specify the characteristics of computation
"TargetSetMode":
{
"none" -> compute Backward Reachable Set,
"minVWithV0" -> min V with V0 (compute Backward Reachable Tube),
"maxVWithV0" -> max V with V0,
"maxVWithVInit" -> compute max V over time,
"minVWithVInit" -> compute min V over time,
"minVWithVTarget" -> min V with target set (if target set is different from initial V0)
"maxVWithVTarget" -> max V with target set (if target set is different from initial V0)
}

(optional)
Please specify this mode if you would like to add another target set, which can be an obstacle set
for solving a reach-avoid problem
"ObstacleSetMode":
{
"minVWithObstacle" -> min with obstacle set,
"maxVWithObstacle" -> max with obstacle set
}
"""

compMethods = { "TargetSetMode": "minVWithVTarget",
                }
# HJSolver(dynamics object, grid, initial value function, time length, system objectives, plotting options)
result = HJSolver(my_car, g, goal, tau, compMethods, po2, saveAllTimeSteps=False, accuracy='medium' )

lin = np.linspace(-np.pi, np.pi, num=40,endpoint=True)
thetas = [np.pi/6, np.pi/3, np.pi/2]
for theta in thetas:
    diff_lin = np.abs(lin-theta)
    idx = np.argmin(diff_lin)
    diff = lin[idx]-theta
    if diff > 0:
        idx2 = idx-1
        diff2 = theta - lin[idx2]
        w2 = diff /(diff+diff2)
    else:
        idx2 = idx+1
        diff2 = lin[idx2] - theta
        w2 = -diff / (-diff + diff2)
    w1 = 1-w2
    print((w1*lin[idx] + w2*lin[idx2])-theta)
np.save('LS_BRT_v05_w0833.npy', result)

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons,TextBox,CheckButtons
import matplotlib.patches as mpatches
from datetime import datetime
import os

from xmlrpc.client import Boolean
import numpy as np

import matplotlib.pyplot as plt
from IPython.display import display, clear_output



import package_Class
from package_Class import *


#Simulation Instance
SIM = Simulation(2000,1,0)

# Path for Every Signal
SP = Path(SIM,{0: 0, 5: 60 ,1000: 0, SIM.TSim: 60})
DV = Path(SIM,{0: 0, 500: 50, 1500:0, SIM.TSim: 0} )
MAN = Path(SIM, {0: 0, 800:0, 900:0,SIM.TSim: 0})

G = Graph(SIM)

Delay_SP = Delay(SIM)

Delay_SP.Delay_RT(SP.Signal,1)

#G.show(DV.Signal,'SP Delay','Graph')
G.show(Delay_SP.MV,'SP Delay','Graph')
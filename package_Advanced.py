import numpy as np

import matplotlib.pyplot as plt
from IPython.display import display, clear_output

import package_DBR
from package_DBR import *

#-----------------------------------
def LeadLag_RT(MV,Kp,TLead,TLag,Ts,PV,PVInit=0,method='EDB'):
    
    """
    The function "FOPDT" DOES NOT need to be included in a "for or while loop": this block is for offline use.
    
    :MV: input vector
    :Kp: process gain
    :TLead: lead time Constant [s]
    :TLag: lag time Constant [s]
    :Ts: sampling period [s]
    :PV: output vector
    :MVInit: (optional: default value is 0)    
    :PVInit: (optional: default value is 0)
    :method: discretisation method (optional: default value is 'EBD')
        EBD: Euler Backward difference
        EFD: Euler Forward difference
        TRAP: Trapezoïdal method
        
    This function apends a value to the output vector "PV"        
    
    """    
    
    if (TLag != 0):
        K = Ts/TLag
        if len(PV) == 0:
            PV.append(PVInit)
        else: # MV[k+1] is MV[-1] and MV[k] is MV[-2]
            if method == 'EBD':
                PV.append((1/(1+K))*PV[-1] + (K*Kp/(1+K))*((1+TLead/Ts)*MV[-1]- (TLead/Ts)*MV[-2]))
            elif method == 'EFD':
                PV.append((1-K)*PV[-1] + K*Kp((TLead/Ts)*MV[-1])+(1-TLead/Ts)*MV[-2])
            elif method == 'TRAP':
                "PV.append((1/(2*T+Ts))*((2*T-Ts)*PV[-1] + Kp*Ts*(MV[-1] + MV[-2])))"            
            else:
                PV.append((1/(1+K))*PV[-1] + (K*Kp/(1+K))*((1+TLead/Ts)*MV[-1]- (TLead/Ts)*MV[-2]))
    else:
        PV.append(Kp*MV[-1])

def PID_RT(MV,Kp,TLead,TLag,Ts,PV,PVInit=0,method='EDB'):
    
    """
    The function "LeadLag_RT" needs to be included in a "for or while loop".
    
    :MV: input vector
    :Kp: process gain
    :T: lag time constant [s]
    :Ts: sampling period [s]
    :PV: output vector
    :PVInit: (optional: default value is 0)
    :method: discretisation method (optional: default value is 'EBD')
        EBD: Euler Backward difference
        EFD: Euler Forward difference
        TRAP: Trapezoïdal method
    
    The function "FO_RT" appends a value to the output vector "PV".
    The appended value is obtained from a recurrent equation that depends on the discretisation method.
    """    
    
    if (T != 0):
        K = Ts/T
        if len(PV) == 0:
            PV.append(PVInit)
        else: # MV[k+1] is MV[-1] and MV[k] is MV[-2]
            if method == 'EBD':
                PV.append((1/(1+K))*PV[-1] + (K*Kp/(1+K))*MV[-1])
            elif method == 'EFD':
                PV.append((1-K)*PV[-1] + K*Kp*MV[-2])
            elif method == 'TRAP':
                PV.append((1/(2*T+Ts))*((2*T-Ts)*PV[-1] + Kp*Ts*(MV[-1] + MV[-2])))            
            else:
                PV.append((1/(1+K))*PV[-1] + (K*Kp/(1+K))*MV[-1])
    else:
        PV.append(Kp*MV[-1])

#-----------------------------------
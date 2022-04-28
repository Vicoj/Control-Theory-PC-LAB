import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors

import package_DBR
from package_DBR import myRound, SelectPath_RT, Delay_RT, FO_RT, FOPDT, SOPDT, FOPDT_cost, SOPDT_cost, Process, Bode



def PID_RT(SP, PV, Man, MVMan, MVFF, Kc, Ti, Td, alpha, Ts, MVMin, MVMax, MV, MVp, MVi, MVd, E, ManFF=False, PVInit=0, method='EBD-EBD'):
    if(len(PV)==0):
        E.append(SP[-1]-PVInit)
    else :
        E.append(SP[-1]-PV[-1])
    
    MVp.append(Kc*E[-1])
    
    if(len(MVi)>0):
        MVi.append(MVi[-1]+(Kc*Ts*E[-1])/Ti)
    else :
        MVi.append(0)
    
    Tfd = alpha*Td
    if(Td>0):
        if(len(MVd)!=0):
            if(len(E)==1):
                MVd.append((Tfd/(Tfd+Ts))*MVd[-1] + ((Kc*Td)/(Tfd+Ts))*(E[-1]))

            else:
                MVd.append(( Tfd / (Tfd+Ts) )*MVd[-1] + ( (Kc*Td) / (Tfd+Ts) ) *(E[-1]-E[-2]))
        else : MVd.append(0)
  

    MV.append(MVp[-1]+MVi[-1]+MVd[-1])
    
    return None
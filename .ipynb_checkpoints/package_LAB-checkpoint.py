import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from matplotlib import colors as mcolors

import package_DBR
from package_DBR import myRound, SelectPath_RT, Delay_RT, FO_RT, FOPDT, SOPDT, FOPDT_cost, SOPDT_cost, Process, Bode



def PID_RT(SP, PV, Man, MVMan, MVFF, Kc, Ti, Td, alpha, Ts, MVMin, MVMax, MV, MVp, MVi, MVd, E, OLP, ManFF=False, PVInit=0, method='EBD-EBD'):
    
    #calcul de l'erreur SP-PV
    
    if(not OLP):
        if(len(PV)==0):
            E.append(SP[-1]-PVInit)
        else :
            E.append(SP[-1]-PV[-1])
    else:
        E.append(SP[-1])
    
    #calcul de MVp
    
    MVp.append(Kc*E[-1])
    
    #calcul MVi
    
    if(len(MVi)>0):
        MVi.append(MVi[-1]+(Kc*Ts*E[-1])/Ti)
    else :
        MVi.append(0)
    
    #calcul MVd
    
    Tfd = alpha*Td
    if(Td>0):
        if(len(MVd)!=0):
            if(len(E)==1):
                MVd.append((Tfd/(Tfd+Ts))*MVd[-1] + ((Kc*Td)/(Tfd+Ts))*(E[-1]))
            else:
                MVd.append(( Tfd / (Tfd+Ts) )*MVd[-1] + ( (Kc*Td) / (Tfd+Ts) ) *(E[-1]-E[-2]))
        else :
            MVd.append(0)
        
    #calcul saturation, anti emballement, reset saturation integrateur
    
    #mode automatique
    if(not Man[-1]):
        #saturation
        if(MVp[-1]+MVi[-1]+MVd[-1] <MVMin) :
            MVi[-1] = MVMin - MVp[-1] - MVd[-1] #ecrasement valeur de MV
        elif (MVp[-1]+MVi[-1]+MVd[-1] >=MVMax) :
            MVi[-1] = MVMax - MVp[-1] - MVd[-1]
    MV.append(MVp[-1]+MVi[-1]+MVd[-1])
    
    #mode manuel
    else :
        if(ManFF):
            MVi[-1]=MVMan[-1]-MVp[-1]-MVd[-1]
        else:
            MVi[-1]=MVMan[-1]-MVp[-1]-MVd[-1]-MVFF[-1]
    
    return None

def IMC_tuning(MV,Kp,T,Ts,Tc,Theta, case="H", T1=0, T2=0, T3=0):
    Ti = 1
    Td = 1
    Kc = 1
    #à modifier
    if case = "G" :
        Kc = T/(Kp*Tc+Theta)
        Ti = T+Theta/2
        Td = 0
    if case = "H" :
        Kc = (T+Theta/2)/(Kp*Tc+Theta/2)
        Ti = T+Theta/2
        Td = (T*Theta)/(2*Tc+Theta)
    if case = "I" :
        Kc = (T1+T2-T3)/(Kp*Tc+Theta)
        Ti = T1+T2-T3
        Td = (T1*T2-(T1+T2-T3)*T3) /(T1+T2-T3)
    #calculer tau c à partir de gamma 
    
    
    return (Kc, Ti, Td)
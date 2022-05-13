from ast import Str
from xmlrpc.client import Boolean
import numpy as np

import matplotlib.pyplot as plt
from IPython.display import display, clear_output




class Simulation:
    def __init__(self,TSim,Ts,PVInit):
        self.TSim = TSim
        self.Ts = Ts
        self.N = int(TSim/Ts) + 1 
        self.PVInit = PVInit
        self.PV = []
        self.t = self.calc_t()

    def calc_t(self):
        t = []
        for i in range(0,self.N):
            t.append(i*self.Ts)

        return t

class Path:
    def __init__(self,S:Simulation,path):
        self.S = S
        self.path = path
        self.Signal = self.SelectPath_RT()

    def SelectPath_RT(self):
        signal = []
        for i in range(0,len(self.S.t)):
            for timeKey in self.path:
                if(self.S.t[i-1] >= timeKey):
                    timeKeyPrevious = timeKey    
            
            value = self.path[timeKeyPrevious]
            signal.append(value)
        
        return signal

class FirstOrder:
    def __init__(self,S:Simulation,gain,Time,Theta,point_fct):
        self.S = S
        self.K = gain
        self.T = Time
        self.Theta = Theta
        self.point_fct = point_fct

        self.MV = []

    def FO_RT(self,input,method):
        MV = []
        temp = input
        for i in range(0,len(self.S.t)):
            input = temp[:i]
            if (self.T != 0):
                KFO = self.S.Ts/self.T
                if len(MV) == 0:
                    MV.append(self.S.PVInit)
                else: # MV[k+1] is MV[-1] and MV[k] is MV[-2]
                    if method == 'EBD':
                        MV.append((1/(1+KFO))*MV[-1] + (KFO*self.K/(1+KFO))*input[-1])
                    elif method == 'EFD':
                        MV.append((1-KFO)*MV[-1] + KFO*self.K*input[-2])
                    elif method == 'TRAP':
                        MV.append((1/(2*self.T+self.S.Ts))*((2*self.T-self.S.Ts)*MV[-1] + self.K*self.S.Ts*(input[-1] + input[-2])))            
                    else:
                        MV.append((1/(1+KFO))*MV[-1] + (KFO*self.K/(1+KFO))*input[-1])
            else:
                MV.append(self.K*input[-1])
        self.MV = MV

class LeadLag:
    def __init__(self,S:Simulation, K,TLead,TLag):
        self.S = S
        self.K = K
        self.TLead = TLead
        self.TLag = TLag

        self.MV = []

    def LeadLag_RT(self,input,method):
        MV = []
        temp = input
        for i in range(0,len(self.S.t)):
            input = temp[:i+1]
            if (self.TLag != 0):
                KLL = self.S.Ts/self.TLag
                if len(MV) == 0:
                    MV.append(self.S.PVInit)
                else: # MV[k+1] is MV[-1] and MV[k] is MV[-2]
                    if method == 'EBD':
                        MV.append((1/(1+KLL))*MV[-1] + (KLL*self.K/(1+KLL))*((1+self.TLead/self.S.Ts)*input[-1]- (self.TLead/self.S.Ts)*input[-2]))
                    elif method == 'EFD':
                        MV.append((1-KLL)*MV[-1] + (KLL*self.K)*((self.TLead/self.S.Ts)*input[-1] +(1-(self.TLead/self.S.Ts))*input[-2]))
                    elif method == 'TRAP':
                        "MV.append((1/(2*T+Ts))*((2*T-Ts)*MV[-1] + Kp*Ts*(input[-1] + input[-2])))"            
                    else:
                        MV.append((1/(1+KLL))*MV[-1] + (KLL*self.K/(1+KLL))*((1+self.TLead/self.S.Ts)*input[-1]- (self.TLead/self.S.Ts)*input[-2]))
            else:
                MV.append(self.K*input[-1])
        self.MV = MV

class FeedForward:
    def __init__(self,S:Simulation):
        self.S = S


        self.MV = []
        self.MV_LL1 = []
        self.MV_LL2 = []
        self.MVFF_Delay = []

    def FF_RT(self,P:FirstOrder ,D:FirstOrder ,method):

        KFF = -(D.K/P.K)

        ThetaFF = np.max([D.Theta-P.Theta,0])
        PVFF = D.MV-D.point_fct*np.ones_like(D.MV)

        Delay_FF = Delay(self.S)
        Delay_FF.Delay_RT(PVFF,ThetaFF)

        LL1 = LeadLag(self.S,KFF,P.T,D.T)
        LL1.LeadLag_RT(Delay_FF.MV,method)

        LL2 = LeadLag(self.S,1,1,1)
        LL2.LeadLag_RT(LL1.MV,method)

        self.MV = LL2.MV

class PID_Controller:
    def __init__(self,S:Simulation,Kc,Ti,Td,alpha,MVMin,MVMax,OLP,ManFF):
        self.S = S
        self.Kc = Kc
        self.Ti = Ti
        self.Td = Td
        self.alpha = alpha
        self.MVMin = MVMin
        self.MVMax = MVMax
        self.OLP = OLP
        self.ManFF = ManFF

        self.MVMan = []

        self.MV = []
        self.MVP = []
        self.MVI = []
        self.MVD = []
        self.E = []

    def PID_RT(self,SP,PV,MVMan,Man,MVFF,method):
        #calcul de l'erreur SP-PV

        self.MV = []
        self.MVP = []
        self.MVI = []
        self.MVD = []
        self.E = []

        tempSP = SP
        tempPV = PV
        tempMVMan = MVMan
        for i in range(0,len(self.S.t)):

            SP = tempSP[:i+1]
            PV = tempPV[:i+1]
            MVMan = tempMVMan[:i+1]

            if(not self.OLP):
                if(len(PV)==0):
                    self.E.append(SP[-1]-self.S.PVInit)
                else :
                    self.E.append(SP[-1]-PV[-1])
            else:
                self.E.append(SP[-1])

            #calcul de MVp

            self.MVP.append(self.Kc*self.E[-1])

            #calcul MVi

            if(len(self.MVI)>0):
                self.MVI.append(self.MVI[-1]+(self.Kc*self.S.Ts*self.E[-1])/self.Ti)
            else :
                self.MVI.append(self.S.PVInit)

            #calcul MVd

            Tfd = self.alpha*self.Td
            if(self.Td>0):
                if(len(self.MVD)!=0):
                    if(len(self.E)==1):
                        self.MVD.append((Tfd/(Tfd+self.S.Ts))*self.MVD[-1] + ((self.Kc*self.Td)/(Tfd+self.S.Ts))*(self.E[-1]))
                    else:
                        self.MVD.append(( Tfd / (Tfd+self.S.Ts) )*self.MVD[-1] + ( (self.Kc*self.Td) / (Tfd+self.S.Ts) ) *(self.E[-1]-self.E[-2]))
                else :
                    self.MVD.append(self.S.PVInit)

            #calcul saturation, anti emballement, reset saturation integrateur

            #mode automatique
            if(not Man[-1]):
                #saturation
                if(self.MVP[-1]+self.MVI[-1]+self.MVD[-1] < self.MVMin) :
                    self.MVI[-1] = self.MVMin - self.MVP[-1] - self.MVD[-1] #ecrasement valeur de MV
                elif (self.MVP[-1]+self.MVI[-1]+self.MVD[-1] >=self.MVMax) :
                    self.MVI[-1] = self.MVMax - self.MVP[-1] - self.MVD[-1]
                self.MV.append(self.MVP[-1]+self.MVI[-1]+self.MVD[-1])

            #mode manuel
            else :
                if(self.ManFF):
                    self.MVI[-1]=MVMan[-1]-self.MVP[-1]-self.MVD[-1]
                else:
                    self.MVI[-1]=MVMan[-1]-self.MVP[-1]-self.MVD[-1]-MVFF[-1]

                self.MV.append(self.MVP[-1]+self.MVI[-1]+self.MVD[-1])
        

class Delay:
    def __init__(self,S:Simulation):
        self.S = S
        self.theta = 0
        self.MVInit = 0 # Par Default

        self.MV = []

    def Delay_RT(self,Inputt,theta):
        self.theta = theta
        input_delay = []

        for i in range(0,len(self.S.t)):
            Input = Inputt[:i]

            NDelay = int(np.ceil(theta/self.S.Ts))
            if NDelay > len(Input)-1:
                input_delay.append(self.MVInit)
            else:    
                input_delay.append(Input[-NDelay-1])

        self.MV = input_delay

class Signal:
    def __init__(self,Signal,name:Str(),color:Str()):
        self.Signal = Signal
        self.name = name
        self.color = color
class Graph:
    def __init__(self,S:Simulation,title):
        self.title = title
        self.S = S
        self.fig, self.ax1 = plt.subplots()
        self.fig.set_figheight(10)
        self.fig.set_figwidth(15)

    def show(self,signals:list()):
        for signal in signals:
            l1, = self.ax1.step(self.S.t,signal.Signal,signal.color,linewidth=2,label=signal.name,where='post')
            self.ax1.set_ylabel('Temperature de Chauffe [%]')
            self.ax1.set_xlabel('Temps [s]')
            self.ax1.set_title(self.title)
            self.ax1.legend(loc='best')

        plt.show()
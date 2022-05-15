from ast import Str
from xmlrpc.client import Boolean
import numpy as np
from matplotlib import colors as mcolors
from matplotlib.widgets import Slider, Button, RadioButtons,TextBox,CheckButtons
import matplotlib.pyplot as plt
from IPython.display import display, clear_output
from datetime import datetime
import os


class Simulation:
    def __init__(self,TSim,Ts,PVInit):
        self.TSim = TSim
        self.Ts = Ts
        self.N = int(TSim/Ts) + 1 
        self.PVInit = PVInit
        self.PV = []
        self.MV = []

        self.t = self.calc_t()

    def calc_t(self):
        t = []
        for i in range(0,self.N):
            t.append(i*self.Ts)

        return t

    def run(self):
        pass

class Path:
    def __init__(self,S:Simulation,path):
        self.S = S
        self.path = path
        self.Signal = []

    def RT(self,time):

        """
        The function "SelectPath_RT" needs to be included in a "for or while loop".

        :path: dictionary input describing a path in time. Example: path = {0: 0, 5: 1, 50: 2, 80: 3, 100: 3}
        :time: time vector.
        :signal: signal vector that is being constructed using the input "path" and the vector "time".

        The function "SelectPath_RT" takes the last element in the vector "time" and, given the input "path", it appends the correct value to the vector "signal".
        """    

        for timeKey in self.path:
            if(time[-1] >= timeKey):
                timeKeyPrevious = timeKey    
    
        value = self.path[timeKeyPrevious]
        self.Signal.append(value)

class FirstOrder:
    def __init__(self,S:Simulation,gain,Time,Theta,point_fct):
        self.S = S
        self.K = gain
        self.T = Time
        self.Theta = Theta
        self.point_fct = point_fct

        self.PV = []

    def RT(self,MV,method):

        """
        The function "FO_RT" needs to be included in a "for or while loop".

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

        if (self.T != 0):
            K = self.S.Ts/self.T
            if len(self.PV) == 0:
                self.PV.append(self.S.PVInit)
            else:
                if method == 'EBD':
                    self.PV.append((1/(1+K))*self.PV[-1] + (K*self.K/(1+K))*MV[-1])
                elif method == 'EFD':
                    self.PV.append((1-K)*self.PV[-1] + K*self.K*MV[-2])
                elif method == 'TRAP':
                    self.PV.append((1/(2*self.T+self.S.Ts))*((2*self.T-self.S.Ts)*self.PV[-1] + self.K*self.S.Ts*(MV[-1] + MV[-2])))            
                else:
                    self.PV.append((1/(1+K))*self.PV[-1] + (K*self.K/(1+K))*MV[-1])
        else:
            self.PV.append(self.K*MV[-1])

class SecondOrderPlusDelay:
    def __init__(self,S:Simulation,gain,Time,Theta,point_fct):
        self.S = S
        self.K = gain
        self.T = Time
        self.Theta = Theta
        self.point_fct = point_fct

        self.PV = []

    def RT(self,MV,method):

        """
        The function "FO_RT" needs to be included in a "for or while loop".

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

        if (self.T != 0):
            K = self.S.Ts/self.T
            if len(self.PV) == 0:
                self.PV.append(self.S.PVInit)
            else:
                if method == 'EBD':
                    self.PV.append((1/(1+K))*self.PV[-1] + (K*self.K/(1+K))*MV[-1])
                elif method == 'EFD':
                    self.PV.append((1-K)*self.PV[-1] + K*self.K*MV[-2])
                elif method == 'TRAP':
                    self.PV.append((1/(2*self.T+self.S.Ts))*((2*self.T-self.S.Ts)*self.PV[-1] + self.K*self.S.Ts*(MV[-1] + MV[-2])))            
                else:
                    self.PV.append((1/(1+K))*self.PV[-1] + (K*self.K/(1+K))*MV[-1])
        else:
            self.PV.append(self.K*MV[-1])

class LeadLag:
    def __init__(self,S:Simulation,K,TLead,TLag):
        self.S = S
        self.K = K
        self.TLead = TLead
        self.TLag = TLag

        self.PV = []

    def RT(self,MV,method):
        if (self.TLag != 0):
            K = self.S.Ts/self.TLag

            if len(self.PV) == 0:
                self.PV.append(self.S.PVInit)
            else: 
                if method == 'EBD':
                    self.PV.append((1/(1+K))*self.PV[-1] + (K*self.K/(1+K))*((1+self.TLead/self.S.Ts)*MV[-1]- (self.TLead/self.S.Ts)*MV[-2]))
                elif method == 'EFD':
                    self.PV.append((1-K)*self.PV[-1] + (K*self.K)*((self.TLead/self.S.Ts)*MV[-1] +(1-(self.TLead/self.S.Ts))*MV[-2]))
                elif method == 'TRAP':
                    pass
                else:
                    self.PV.append((1/(1+K))*self.PV[-1] + (K*self.K/(1+K))*((1+self.TLead/self.S.Ts)*MV[-1]- (self.TLead/self.S.Ts)*MV[-2]))
        else:
            self.PV.append(self.K*MV[-1])

class FeedForward:
    def __init__(self,S:Simulation,P:FirstOrder,D:FirstOrder):
        self.S = S
        self.P = P
        self.D = D

        self.Kd = D.K
        self.Kp = P.K

        self.T1p = P.T
        self.T1d = D.T
        self.T2p = 1
        self.T2d = 1

        self.DV0 = D.point_fct

        self.ThetaD = D.Theta
        self.ThetaP = P.Theta


        self.MVFF = []
        #Gain
        KFF = -(self.Kd/self.Kp) 

        #Delay
        thetaFF = np.max([self.ThetaD-self.ThetaP,0])
        self.delayFF = Delay(S,thetaFF)
        
        #leadLag
        self.LL1 = LeadLag(S,KFF,self.T1p,self.T1d)
        self.LL2 = LeadLag(S,1,self.T2p,self.T2d)

    def RT(self,DV):

        #Dephasage
        PVFF = DV-self.DV0*np.ones_like(DV)
        self.delayFF.RT(PVFF) 
    
        self.LL1.RT(self.delayFF.PV,'EBD')
        self.LL2.RT(self.LL1.PV,'EBD')
        
    
        self.MVFF.append(self.LL2.PV[-1])

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

        self.MVMan = [80]

        self.MVFB = []
        self.MVP = []
        self.MVI = []
        self.MVD = []
        self.E = []

    def RT(self,SP,PV,MAN,MVMan,MVFF,method):
        
    #calcul de l'erreur SP-PV
    
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
        if(not MAN[-1]):
            #saturation
            if(self.MVP[-1]+self.MVI[-1]+self.MVD[-1] <self.MVMin) :
                self.MVI[-1] = self.MVMin - self.MVP[-1] - self.MVD[-1] #ecrasement valeur de MV
            elif (self.MVP[-1]+self.MVI[-1]+self.MVD[-1] >=self.MVMax) :
                self.MVI[-1] = self.MVMax - self.MVP[-1] - self.MVD[-1]
            self.MVFB.append(self.MVP[-1]+self.MVI[-1]+self.MVD[-1])

        #mode manuel
        else :
            if(self.ManFF):
                self.MVI[-1]=MVMan[-1]-self.MVP[-1]-self.MVD[-1]
            else:
                self.MVI[-1]=MVMan[-1]-self.MVP[-1]-self.MVD[-1]-MVFF[-1]

            self.MVFB.append(self.MVP[-1]+self.MVI[-1]+self.MVD[-1])
        
class Delay:
    def __init__(self,S:Simulation,theta):
        self.S = S
        self.MVInit = 0 # Par Default
        self.theta = theta
        self.PV = []

    def RT(self,MV):

        """
        The function "Delay_RT" needs to be included in a "for or while loop".

        :MV: input vector
        :theta: delay [s]
        :Ts: sampling period [s]
        :MV_Delay: delayed input vector
        :MVInit: (optional: default value is 0)

        The function "Delay_RT" appends a value to the vector "MV_Delay".
        The appended value corresponds to the value in the vector "MV" "theta" seconds ago.
        If "theta" is not a multiple of "Ts", "theta" is replaced by Ts*int(np.ceil(theta/Ts)), i.e. the closest multiple of "Ts" larger than "theta".
        If the value of the vector "input" "theta" seconds ago is not defined, the value "MVInit" is used.
        """

        NDelay = int(np.ceil(self.theta/self.S.Ts))
        if NDelay > len(MV)-1:
            self.PV.append(self.MVInit)
        else:    
            self.PV.append(MV[-NDelay-1])

class Signal:
    def __init__(self,Signal,name:Str(),color:Str()):
        self.Signal = Signal
        self.name = name
        self.color = color

class Graph:
    def __init__(self,S:Simulation,title):

        self.title = title
        self.S = S
        

    def show(self,signals:list(),binSignals:list()):
        self.fig, self.ax = plt.subplots(2, gridspec_kw={'height_ratios': [1, 3]})
        self.signals = signals
        self.binSignals = binSignals

        for bin in binSignals:
            self.ax[0].step(self.S.t,bin.Signal,bin.color,linewidth=2,label=bin.name,where='post')
            self.ax[0].set_ylabel('Valeurs Binaires (On/Off)')
            self.ax[0].set_xlabel('Temps [s]')
            self.ax[0].set_title(self.title)
            self.ax[0].legend(loc='best')

        for signal in signals:
            self.ax[1].step(self.S.t,signal.Signal,signal.color,linewidth=2,label=signal.name,where='post')
            self.ax[1].set_ylabel('Temperature de Chauffe [%]')
            self.ax[1].set_xlabel('Temps [s]')
            #self.ax[1].set_ylim(-10,120)
            self.ax[1].legend(loc='best')

        plt.subplots_adjust(left=0.05, bottom=0.05, right = 0.8,top=0.95,hspace=0.064)
        # Buttons
        saveax = plt.axes([0.93, 0.15, 0.05, 0.04]) #4-tuple of floats *rect* = [left, bottom, width, height]
        button_save = Button(saveax, 'Save', hovercolor='0.975')
        button_save.on_clicked(self.save)

        namebox = plt.axes([0.83, 0.15, 0.1, 0.04])
        self.text_box = TextBox(namebox, 'Name :', initial='')

        closefig = plt.axes([0.83, 0.05, 0.1, 0.04])
        button_close = Button(closefig, 'Close', hovercolor='0.975')
        button_close.on_clicked(self.close)


        plt.get_current_fig_manager().full_screen_toggle()
        plt.show()


    def save(self,event):

        self.fig.savefig("Output/"+self.text_box.text)
        now = datetime.now()
        date_time = now.strftime("%Y-%m-%d-%Hh%M")
        t = np.array(self.S.t)
        data = [t.T]
        data_names ='t,'

        for sig in self.signals:
            arr = np.array(sig.Signal)
            data.append(arr.T)
            data_names += sig.name + ','

        for bin in self.binSignals:
            arr = np.array(bin.Signal)
            data.append(arr.T)
            data_names += bin.name + ','
        
        my_data = np.vstack((data))
        my_data = my_data.T


        nameFile = 'Data/PID_Graph_' + self.text_box.text + '_' + date_time + '.txt'
        if not os.path.exists('Data'):
            os.makedirs('Data')
        np.savetxt(nameFile,my_data,delimiter=',',header=data_names,comments='')
                   

    def close(self,event):
        plt.close()

    def Bode(self,P:FirstOrder, Show = True):
    
        """
        :P: Process as defined by the class "Process".
            Use the following command to define the default process which is simply a unit gain process:
                P = Process({})

            A delay, two lead time constants and 2 lag constants can be added.

            Use the following commands for a SOPDT process:
                P.parameters['Kp'] = 1.1
                P.parameters['Tlag1'] = 10.0
                P.parameters['Tlag2'] = 2.0
                P.parameters['theta'] = 2.0

            Use the following commands for a unit gain Lead-lag process:
                P.parameters['Tlag1'] = 10.0        
                P.parameters['Tlead1'] = 15.0        

        :omega: frequency vector (rad/s); generated by a command of the type "omega = np.logspace(-2, 2, 10000)".
        :Show: boolean value (optional: default value = True). If Show = True, the Bode diagram is shown. Otherwise Ps (P(j omega)) (vector of complex numbers) is returned.

        The function "Bode" generates the Bode diagram of the process P
        """     

        omega = np.logspace(-2, 2, 10000)
        s = 1j*omega
        
        Ptheta = np.exp(-P.Theta*s)
        PGain = P.K*np.ones_like(Ptheta)
        PLag1 = 1/(P.T*s + 1)
        PLag2 = 1/(1*s + 1)
        PLead1 = 1*s + 1
        PLead2 = 1*s + 1
        
        Ps = np.multiply(Ptheta,PGain)
        Ps = np.multiply(Ps,PLag1)
        Ps = np.multiply(Ps,PLag2)
        Ps = np.multiply(Ps,PLead1)
        Ps = np.multiply(Ps,PLead2)
        
        if Show == True:
        
            fig, (ax_gain, ax_phase) = plt.subplots(2,1)
            fig.set_figheight(12)
            fig.set_figwidth(22)

            # Gain part
            ax_gain.semilogx(omega,20*np.log10(np.abs(Ps)),label='P(s)')
            ax_gain.semilogx(omega,20*np.log10(np.abs(PGain)),label='Pgain')
            if P.Theta > 0:
                ax_gain.semilogx(omega,20*np.log10(np.abs(Ptheta)),label='Ptheta(s)')
            if P.T > 0:
                ax_gain.semilogx(omega,20*np.log10(np.abs(PLag1)),label='PLag1(s)')
            if 0 > 0:        
                ax_gain.semilogx(omega,20*np.log10(np.abs(PLag2)),label='PLag2(s)')
            if 0 > 0:        
                ax_gain.semilogx(omega,20*np.log10(np.abs(PLead1)),label='PLead1(s)')
            if 0 > 0:    
                ax_gain.semilogx(omega,20*np.log10(np.abs(PLead2)),label='PLead2(s)')    
            gain_min = np.min(20*np.log10(np.abs(Ps)/5))
            gain_max = np.max(20*np.log10(np.abs(Ps)*5))
            ax_gain.set_xlim([np.min(omega), np.max(omega)])
            ax_gain.set_ylim([gain_min, gain_max])
            ax_gain.set_ylabel('Amplitude |P| [db]')
            ax_gain.set_title('Bode plot of P')
            ax_gain.legend(loc='best')
        
            # Phase part
            ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(Ps)),label='P(s)')
            ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(PGain)),label='Pgain')
            if P.Theta > 0:    
                ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(Ptheta)),label='Ptheta(s)')
            if P.T > 0:        
                ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(PLag1)),label='PLag1(s)')
            if 1 > 0:        
                ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(PLag2)),label='PLag2(s)')
            if 1> 0:        
                ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(PLead1)),label='PLead1(s)')
            if 1 > 0:        
                ax_phase.semilogx(omega, (180/np.pi)*np.unwrap(np.angle(PLead2)),label='PLead2(s)')    
            ax_phase.set_xlim([np.min(omega), np.max(omega)])
            ph_min = np.min((180/np.pi)*np.unwrap(np.angle(Ps))) - 10
            ph_max = np.max((180/np.pi)*np.unwrap(np.angle(Ps))) + 10
            ax_phase.set_ylim([np.max([ph_min, -200]), ph_max])
            ax_phase.set_ylabel(r'Phase $\angle P$ [°]')
            ax_phase.legend(loc='best')

            plt.show()
        else:
            return Ps
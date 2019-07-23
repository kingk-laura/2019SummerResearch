import kivy
kivy.require('1.9.0')

from kivy.app import App
from kivy.uix.popup import Popup
from kivy.uix.widget import Widget
from kivy.properties import ObjectProperty
from kivy.factory import Factory
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.togglebutton import ToggleButton
from subprocess import call
from kivy.config import Config

import numpy
import FileDialog
import matplotlib.pyplot as plt

import shutil
import os
import threading

Config.set('graphics', 'width', '600');
Config.set('graphics', 'height', '600');
Config.set('input', 'mouse', 'mouse, multitouch_on_demand')

import subprocess

class PopupBox(Popup):
    pop_up_text = ObjectProperty()
    def update_pop_up_text(self, p_message):
        self.pop_up_text.text = p_message

class Sim(BoxLayout):

    def show_popup(self):
        self.pop_up = Factory.PopupBox()
        self.pop_up.update_pop_up_text('Simulation running, Please wait...')
        self.pop_up.open()

    def modify(self,num):
        f = open("./car3D.app/Contents/Java/data/config", "w")
        f.write(num)
        f.close()

    def copyFile(self):
        shutil.copy2('./rolldata.txt', './data/carMotion.txt')
        shutil.copy2('./roaddata.txt', './data/roaddata.txt')

    def matlabSim(self):

        import matlab.engine
        eng = matlab.engine.start_matlab()
        ret = eng.main(nargout=0)
        self.copyFile()
        self.pop_up.dismiss()


    def calculate(self, vmparam, msparam, m_uparam, lfparam, lrparam, lwparam, hsparam, hrparam, reffparam, iwheelparam, izparam, ksparam, ktparam, bsparam, caparam, gparam, cfparam, crparam, stcontrol, stacontrol, crcontrol):

        with open('vehicle_parameters.m', 'w') as f:
            f.write('m = ' + vmparam + ';\n')
            f.write('ms = ' + msparam + ';\n')
            f.write('m_u = ' + m_uparam + ';\n')
            f.write('m = ms + 4*m_u; \n')
            f.write('mass = ms + 4*m_u; \n')
            f.write('Lf = ' + lfparam + ';\n')
            f.write('Lr = ' + lrparam + ';\n')
            f.write('Lw = ' + lwparam + ';\n')
            f.write('Cs = 828*180/pi; \n')
            f.write('hs = ' + hsparam + ';\n')
            f.write('hr = ' + hrparam + ';\n')
            f.write('reff = ' + reffparam + ';\n')
            f.write('rwheel = reff; \n')
            f.write('Iwheel = ' + iwheelparam + ';\n')
            f.write('Iz = ' + izparam + ';\n')
            f.write('Iy = ms*Lf*Lr; \n')
            f.write('Ix = ms*Lw*Lw; \n')
            f.write('ks = ' + ksparam + ';\n')
            f.write('kt = ' + ktparam + ';\n')
            f.write('bs = ' + bsparam + ';\n')
            f.write('Ca = ' + caparam + ';\n')
            f.write('g = ' + gparam + ';\n')
            f.write('\n')
            f.write('Cf = ' + cfparam + ';\n')
            f.write('Cr = ' + crparam + ';\n')
            f.write('radius = 300; \n')
            f.write('active_steering_factor = 1.0; \n')
            f.write('psi_dot_bound = 0.3; \n')
            if (stcontrol == 'Steering control'):
                f.write('type_of_steering = 4; \n')
            elif (stcontrol == 'No Steering control'):
                f.write('type_of_steering = 0; \n')
            elif (stcontrol == 'Default Closed-Loop Steering control'):
                f.write('type_of_steering = 4; \n')
            elif (stcontrol == 'Step Steering Input'):
                f.write('type_of_steering = 1; \n')
            elif (stcontrol == 'Fish Hook Steering Input'):
                f.write('type_of_steering = 2; \n')
            elif (stcontrol == 'User-defined Steering control'):
                f.write('type_of_steering = 5; \n')
            else:
                f.write('type_of_steering = 1; \n')

            if (crcontrol == 'Cruise Control'):
                f.write('type_of_cruise_control = 2; \n')
            elif (crcontrol == 'No Cruise Control'):
                f.write('type_of_cruise_control = 1; \n')
            elif (crcontrol == 'Default Cruise Control'):
                f.write('type_of_cruise_control = 2; \n')
            elif (crcontrol == 'User-defined Cruise Control'):
                f.write('type_of_cruise_control = 3; \n')
            else:
                f.write('type_of_cruise_control = 2; \n')

            if (stacontrol == 'Stability Control'):
                f.write('type_of_ESC = 2; \n')
            elif (stacontrol == 'No Stability Control'):
                f.write('type_of_ESC = 1; \n')
            elif (stacontrol == 'Default Stability Control'):
                f.write('type_of_ESC = 2; \n')
            elif (stacontrol == 'User-defined Stability Control'):
                f.write('type_of_ESC = 3; \n')

            f.write('yes_to_plotting = 0; \n')



        self.show_popup()
        threading.Thread(target=self.matlabSim).start()
        # nthread.start()

        

    def plotxy(self, a, b):
        # threading.Thread(target=self.plotData, args = (a, b)).start()
        self.plotData(a,b)


    def plotTr(self):
        aa = [t for t in ToggleButton.get_widgets('tires') if t.state=='down'][0].text
        bb = [t for t in ToggleButton.get_widgets('opt') if t.state=='down'][0].text

        checkDic = {
                'Left Front': 0,
                'Right Front': 1,
                'Left Rear': 2,
                'Right Rear': 3,
                'Ftire long': 13,
                'Ftire lat': 9,
                'Slip ratio': 17,
                'Slip angle': 21
                }

        a = 0
        b = checkDic.get(bb) + checkDic.get(aa)

        f = open("./plotdata.txt")
        xl = []
        yl = []

        a = int(a)
        b = int(b)

        for line in f:
            line = line[:-1]
            ld = line.split(',')
            xl.append(float(ld[a]))
            yl.append(float(ld[b]))

        plt.plot(xl,yl)
        plt.xlabel(aa)
        plt.ylabel(bb)
        plt.show()



            

    def plotData(self, a, b):
        aa = a
        bb = b
        checkDic = {
                'time': 0,
                'x_global': 7,
                'y_global': 8,
                'yaw rate': 2,
                'vehicle speed': 1,
                'lateral acceleration': 6,
                'lateral velocity': 3,
                'slip angle': 4,
                'yaw angle': 5
                }
        a = checkDic.get(a)
        b = checkDic.get(b)

        f = open("./plotdata.txt")
        xl = []
        yl = []

        a = int(a)
        b = int(b)

        for line in f:
            line = line[:-1]
            ld = line.split(',')
            xl.append(float(ld[a]))
            yl.append(float(ld[b]))

        plt.plot(xl,yl)
        plt.xlabel(aa)
        plt.ylabel(bb)
        plt.show()

    def runCode(self, express):
        os.system('car3D')


class SimApp(App):
    title = 'Auto Car'

    def build(self):
        self.icon = 'icon.JPG'
        return Sim()

    def on_pause(self):
        return True

if __name__ in ('__main__', '__android__'):
    SimApp().run()


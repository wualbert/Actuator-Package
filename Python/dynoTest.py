# dynoTest.py
#=-=-=-=-=-=-=-=-=-=-=-=
# Class for dyno testing
# 2018 Albert Wu
import numpy as np
import time
from dyno_parameters import *

class DynoTest:
    def __init__(self,testFlexSEA,loadFlexSEA):
        '''

        :param testFlexSEA: The FlexSEA driving the tested motor
        :param loadFlexSEA: The FlexSEA driving the load motor
        '''
        self.testFlexSEA = testFlexSEA
        self.loadFlexSEA = loadFlexSEA
        self.lastChangeLoadTime = time.time()
        self.lastChangeTestTime = time.time()
        self.currentTestV = V_MIN
        self.currentLoadV = V_MIN
        self.done = False
        #TODO: write to CSV

    def run(self):
        '''
        Called at every timestep. Runs the test
        :return:
        '''
        if time.time()-self.lastChangeTime < CHANGE_LOAD_WAIT:
            #do nothing
            return
        else:
            #Read data
            self.testFlexSEA.readActPack(0, 2, DISP_DIV,False)
            self.testFlexSEA.readActPack(0, 2, DISP_DIV,False)
            self.lastChangeTime = time.time()

            #TODO: Dynamic wait time selection
            # Evaluate data
            velocity = self.testFlexSEA.myRigid.ex.enc_ang_vel[0]
            #TODO: read torque value

            if abs(velocity) < ZERO_SPEED_MARGIN:
                # Stalled. Test next curve
                self.changeTestV()
            else:
                # Change load voltage
                self.changeLoadV()

            #Store data
            self.writeData()

        return

    def changeLoadV(self):
        '''
        Sets the load motor to a new voltage
        :return:
        '''
        self.currentLoadV+=V_LOAD_STEP_SIZE
        if self.currentLoadV>self.currentTestV:
            #Reached maximum voltage
            print('Warning: Load Voltage > Test Voltage!')
        #send voltage command
        self.loadFlexSEA.setMotorVoltage(self.currentLoadV)
        self.lastChangeLoadTime = time.time()
        return

    def changeTestV(self):
        '''
        Sets the test motor to a new voltage
        Sets the load motor to 0 volt
        :return:
        '''
        self.currentLoadV=0     #clear load
        self.currentTestV+=V_STEP_SIZE

        if self.currentTestV>V_MAX:
            #Test has ended
            print('Reached Maximum Test Voltage')
            #TODO: wrap up test
        else:
            self.loadFlexSEA.setMotorVoltage(self.currentLoadV)
            self.testFlexSEA.setMotorVoltage(self.currentTestV)
        self.lastChangeLoadTime = time.time()
        self.lastChangeTestTime = time.time()
        return

    def writeData(self):
        #TODO: Record data
        pass

    def finish(self):
        #TODO: close csv, exit the program gracefully
        return
# FlexSEA_Dyno_Testing.py
#=-=-=-=-=-=-=-=-=-=-=-=
# Performs Dyno Testing
# 2018 Albert Wu


import serial
from time import perf_counter, sleep
import time
from pyFlexSEA import *
from dyno_parameters import *
from dynoTest import *
import os
import sys
import sched # python task scheduler library

# User setup:
#Get COM Ports
loadCOM, testCOM, junk = comPortFromFile().split(",")
flexSEAScheduler = sched.scheduler(perf_counter, sleep) # global scheduler

# This is called by the timer:
# lastTimeStamp = 0
# timeStamp = perf_counter()
# f = 1/refreshRate
# def timerEvent():
# 	global f
# 	global i
# 	# Read data & display it:
# 	#print(time.time())
# 	#i = readActPack(0, 2, displayDiv)
# 	#if i == 0:
# 	#	print('\nFSM State =', state)
# 	# Call state machine:
# 	#print(testFlexSEA.myRigid.mn.accel==loadFlexSEA.myRigid.mn.accel)
# 	print('test')
# 	testFlexSEA.readActPack(0, 2, displayDiv)
# 	sleep(0.1)
# 	print('load')
# 	loadFlexSEA.readActPack(0, 2, displayDiv)
# 	sleep(0.1)
# 	# if (testFlexSEA.myRigid.mn.accel.x==loadFlexSEA.myRigid.mn.accel.x and
# 	# 	  testFlexSEA.myRigid.mn.accel.y == loadFlexSEA.myRigid.mn.accel.y and
# 	# 	  testFlexSEA.myRigid.mn.accel.z == loadFlexSEA.myRigid.mn.accel.z):
# 	# 	print(testFlexSEA.myRigid.mn.gyro.x,'test')
# 	# 	print(loadFlexSEA.myRigid.mn.gyro.x,'load')
# 	# 	print('---')
#     #
# 	# f = f*0.99 + 0.01/(timeStamp-lastTimeStamp+0.00000001) # leaky integral refresh rate
# 	# if i == 0:
# 	# 	print('\nRefresh rate =', f)
# 	# 	print("--> press 'control+c' to quit")
#
# 	flexSEAScheduler.enter(refreshRate, 1, timerEvent) # adds itself back onto schedule, with priority 1

# this is called by the timer:
lastTimeStamp = 0
timeStamp = perf_counter()
f = 1 / REFRESH_RATE
def timerEvent():
	global lastTimeStamp
	global timeStamp
	global f
	lastTimeStamp = timeStamp
	timeStamp = perf_counter()
	testFlexSEA.syncActPack(0, 2, DISP_DIV, False)
	loadFlexSEA.syncActPack(0, 2, DISP_DIV, False)
	print(loadFlexSEA.printRigid())
	try:
		myDynoTest.run()
	except Exception as e:
		print('Exception in dynoTest():')
		print(e)


	# f = f*0.99 + 0.01/(timeStamp-lastTimeStamp) # leaky integral refresh rate
	# if i == 0:
	# 	print('\nRefresh rate =', f)
	# 	print("--> press 'control+c' to quit")

	flexSEAScheduler.enter(REFRESH_RATE, 1, timerEvent) # adds itself back onto schedule, with priority 1


#########################################################################################
#################### USER DEFINED PERSISTENT VARIABLES GO HERE ##########################

# global values:
# any variable you want to persist across loop iterations
# should be defined with the keyword 'global' inside a function, 
# and initialized outside of any functions
loop_counter = 0
top_speed = 0
max_accel_x = 0
min_accel_x = 0
was_motor_angel_equal_to_tau = False
tau = 6283185 #6.283185

############################ DYNO TESTING CODE ################################


########################### END OF DYNO TESTING CODE ###########################
################################################################################

# Housekeeping before we quit:
def beforeExiting():
	print('closing com')
	try:
		loadFlexSEA.setMotorVoltage(int(0))
		sleep(0.5)
		testFlexSEA.setMotorVoltage(int(0))
		sleep(0.5)
		loadFlexSEA.setControlMode(0)
		sleep(0.5)
		testFlexSEA.setControlMode(0)
		sleep(0.5)
		loadhser.close()
		testhser.close()
		sleep(0.5)
	except Exception as e:
		print(e)
	print('\n\nDone.\n')


print('\nDyno Testing')
print('====================================================\n')
try:
	# Set up serial for load FlexSEA
	loadhser = serial.Serial(loadCOM)
	print('Opened', loadhser.portstr, 'for load FlexSEA')
	print('Initializing Load FlexSEA stack...')
	#Initialize load FlexSEA
	loadFlexSEA = PyFlexSEA()
	loadFlexSEA.initPyFlexSEA()
	loadFlexSEA.setPyFlexSEASerialPort(loadhser) #Pass com handle to pyFlexSEA
except serial.SerialException:
	print('Load FlexSEA Initialization Failed!')
sleep(0.1)

try:
	# Set up serial for test FlexSEA
	testhser = serial.Serial(testCOM)
	print('Opened', testhser.portstr, 'for test FlexSEA')
	print('Initializing Test FlexSEA stack...')
	#Initialize load FlexSEA
	testFlexSEA = PyFlexSEA()
	testFlexSEA.initPyFlexSEA()
	testFlexSEA.setPyFlexSEASerialPort(testhser) #Pass com handle to pyFlexSEA
	#initialize test
	myDynoTest = DynoTest(testFlexSEA,loadFlexSEA)
except serial.SerialException:
	print('Test FlexSEA Initialization Failed!')


	#TODO: Set up serial for torque sensor


# Background: read Rigid and call FSM at 100Hz:
print('Starting the background comm...')
sleep(0.1)

# set control mode to voltage open loop
testFlexSEA.setControlMode(CTRL_OPEN)
sleep(0.5)
loadFlexSEA.setControlMode(CTRL_OPEN)
sleep(0.5)
# Main while() loop:
#===================
try:
	print('Test V set to 0')
	while True:
		flexSEAScheduler.enter(REFRESH_RATE, 1, timerEvent)
		flexSEAScheduler.run()
		sleep(60*60*24) # arbitrary sleep time
except (KeyboardInterrupt, SystemExit):
	beforeExiting()
	sys.exit()
except Exception:
	print('Exception in main while loop!')

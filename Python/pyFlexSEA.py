#Python wrapper for the FlexSEA import ctypes
from ctypes import *
from ctypes import cdll
from ctypes import _SimpleCData
import serial
from time import sleep
from pyFlexSEA_def import *
import os
import sys
import platform

class PyFlexSEA:
	def __init__(self):
		# adding currenty directory path
		self.dir_path = os.path.dirname(os.path.realpath(__file__))

		# Variables used to send packets:
		self.nb = c_ushort(0)
		self.packetIndex = c_ushort(0)
		self.arrLen = c_ubyte(10)
		self.commStr = (c_ubyte * COMM_STR_LEN)()
		self.cBytes = (c_uint8 * COMM_STR_LEN)()
		self.myRigid = rigid_s()
		self.myPocket = pocket_s()

		# Control variables:
		self.controlChannels = 2
		self.pCtrl = (c_uint8 * self.controlChannels)()
		self.pSetpoint = (c_int32 * self.controlChannels)()
		self.pSetGains = (c_uint8 * self.controlChannels)()
		self.pG0 = (c_int16 * self.controlChannels)()
		self.pG1 = (c_int16 * self.controlChannels)()
		self.pG2 = (c_int16 * self.controlChannels)()
		self.pG3 = (c_int16 * self.controlChannels)()
		self.pSystem = c_uint8(0)

		# Miscellaneous
		self.ri_offs = 0

		self.printDivider = 0

		#Controller index to string mapping
		self.mapCtrlText = { 0 : 'CTRL_NONE',
						1 : 'CTRL_OPEN',
						2 : 'CTRL_POSITION',
						3 : 'CTRL_CURRENT',
						4 : 'CTRL_IMPEDANCE',
						5 : 'CTRL_CUSTOM',
						6 : 'CTRL_MEASRES',}

	# Stack init and support functions:
	# =================================

	def initPyFlexSEA(self):
		# Init code:
		print('[pySerial Module]\n')
		global flexsea

		loadSucceeded = False
		is_64bits = sys.maxsize > 2 ** 32
		sysOS = platform.system().lower()

		libraries = []
		if ("win" in sysOS):
			if (is_64bits):
				libraries.append(self.dir_path + '/lib/FlexSEA-Stack-Plan_64')
			else:
				libraries.append(self.dir_path + '/lib/FlexSEA-Stack-Plan')

		elif ("darwin" in sysOS):
			print("FlexseaStack for mac does not exist")
		elif ("linux" in sysOS):
			libraries.append(self.dir_path + '/lib/libFlexSEA-Stack-Plan.so')  # Linux
		else:
			libraries.append(self.dir_path + '/lib/rpiFlexSEA-Stack-Plan.so')  # Raspbian

		for lib in libraries:
			try:
				flexsea = cdll.LoadLibrary(lib)
				loadSucceeded = True
				break
			except:
				pass

		if (not loadSucceeded):
			print("Failed to load shared library. Check library path\n")
		else:
			# Init stack:-
			flexsea.initFlexSEAStack_minimalist(FLEXSEA_PLAN_1);
			# Initialize control variables:
			self.initControlVariables()

		return loadSucceeded

	def initControlVariables(self):
		for i in range(0, self.controlChannels):
			self.pCtrl[i] = c_uint8(CTRL_NONE)
			self.pG0[i] = c_int16(0)
			self.pG1[i] = c_int16(0)
			self.pG2[i] = c_int16(0)
			self.pG3[i] = c_int16(0)
			self.pSetpoint[i] = c_int32(0)
			self.pSetGains[i] = c_uint8(KEEP)

	#Get serial port handle from host
	def setPyFlexSEASerialPort(self,s):
		self.hser = s

	#Read serial port from com.txt file
	def comPortFromFile(self):
		file = open("com.txt", "r")
		s = file.read()
		#print(s)
		return s

	#Did we receive new serial bytes?
	def serialBytesReady(self,timeout, b):
		i = 0
		while self.hser.in_waiting < b:
			i = i + 1
			if i > timeout:
				break
		return self.hser.in_waiting

	#Next communication offset
	def offs(self,min, max):
		self.ri_offs += 1
		if self.ri_offs > max:
			self.ri_offs = min
		return self.ri_offs

	#Clears the terminal - use before printing new values
	def clearTerminal(self):
		if sys.platform.lower().startswith('win'):
			os.system('cls') #Clear terminal (Win)
		elif sys.platform.lower().startswith('linux'):
			os.system('clear') #Clear terminal (Unix)

#ActPack user functions:
#=======================

#ActPack is used to read sensor values, and write controller options & setpoint
#minOffs & maxOffs control what offsets are read (0: IMU, joint enc., etc., 
# 1: motor ang/vel/acc, board state, etc., 2: genVar (used for 6-ch Strain), ...)
#printDiv: values will be displayed on the terminal every printDiv samples
	def readActPack(self,minOffs, maxOffs, printDiv, displayFlexSEA=True):

		self.requestReadActPack(self.offs(minOffs, maxOffs))
		bytes = self.serialBytesReady(100, COMM_STR_LEN)

		#s = self.hser.read(bytes)
		s = self.hser.read(COMM_STR_LEN) #Reading a fixed length for now
		for i in range(0,bytes-1):
			#print(s[i], end=' ')
			self.cBytes[i] = s[i]
		#print(']')

		ppFlag = c_uint8(0)
		ppFlag = flexsea.receiveFlexSEABytes(byref(self.cBytes), bytes, 1);
		if(ppFlag):
			#print('We parsed a packet: ', end='')
			cmd = c_uint8(0)
			type = c_uint8(0)
			flexsea.getSignatureOfLastPayloadParsed(byref(cmd), byref(type));
			#print('cmd:', cmd, 'type:', type)

			newActPackPacket = c_uint8(0)
			newActPackPacket = flexsea.newActPackRRpacketAvailable();

			if(newActPackPacket):
				#print('New Rigid packet(s) available\n')
				flexsea.getLastRigidData(byref(self.myRigid));

				if displayFlexSEA:
					i = self.printActPack(printDiv)
					return i
			#else:
				#print('This is not a Rigid packet')
		#else:
			#print('Invalid packet')

#Send Read Request ActPack:
	def requestReadActPack(self,offset):
		flexsea.ptx_cmd_actpack_rw(FLEXSEA_MANAGE_1, byref(self.nb), self.commStr, offset, self.pCtrl[0], self.pSetpoint[0], self.pSetGains[0], self.pG0[0], self.pG1[0], self.pG2[0], self.pG3[0], self.pSystem);
		self.hser.write(self.commStr)
		if(offset == 0 and self.pSetGains[0] == CHANGE):
			self.pSetGains[0] = c_uint8(KEEP)

#Use this function to enable or disable FSM2. Controller will be reset.
	def actPackFSM2(self,on):
		self.pCtrl[0] = c_uint8(CTRL_NONE)	#Disable controller
		if on:
			self.pSystem = c_uint8(SYS_NORMAL)
		else:
			self.pSystem = c_uint8(SYS_DISABLE_FSM2)

		flexsea.ptx_cmd_actpack_rw(FLEXSEA_MANAGE_1, byref(self.nb), self.commStr, c_uint8(0), self.pCtrl[0], self.pSetpoint[0], self.pG0[0], self.pG1[0], self.pG2[0], self.pG3[0], self.pSetGains[0], self.pSystem);
		self.hser.write(self.commStr)

#Sends a request to Execute. Make sure to disable FSM2 first.
	def findPoles(self,block):
		flexsea.ptx_cmd_calibration_mode_rw(FLEXSEA_EXECUTE_1, byref(self.nb), self.commStr, c_uint8(CALIBRATION_FIND_POLES))
		self.hser.write(self.commStr)
		if not block:
			return
		else:
			for s in range(60,0, -1):
				print(s,'seconds...')
				sleep(1)
			print('Ready!')

#Pocket functions:
#================

#Pocket is used to read sensor values, and write controller options & setpoint (FlexSEA-Pocket only)
#minOffs & maxOffs control what offsets are read 
# 0: IMU, voltages and other Mn + Re variables 
# 1: Right motor
# 2: Left motor
# 3: genVars
#printDiv: values will be displayed on the terminal every printDiv samples
	def readPocket(self,minOffs, maxOffs, printDiv, displayFlexSEA=True):

		self.requestReadPocket(self.offs(minOffs, maxOffs))
		bytes = self.serialBytesReady(100, COMM_STR_LEN)

		#s = self.hser.read(bytes)
		s = self.hser.read(COMM_STR_LEN) #Reading a fixed length for now
		for i in range(0,bytes-1):
			#print(s[i], end=' ')
			self.cBytes[i] = s[i]
		#print(']')

		ppFlag = c_uint8(0)
		#print("Bytes:", bytes)
		ppFlag = flexsea.receiveFlexSEABytes(byref(self.cBytes), bytes, 1);
		if(ppFlag):
			#print('We parsed a packet: ', end='')
			cmd = c_uint8(0)
			type = c_uint8(0)
			flexsea.getSignatureOfLastPayloadParsed(byref(cmd), byref(type));
			#print('cmd:', cmd, 'type:', type)

			newPocketPacket = c_uint8(0)
			newPocketPacket = flexsea.newPocketRRpacketAvailable();

			if(newPocketPacket):
				#print('New Rigid packet(s) available\n')
				flexsea.getLastPocketData(byref(self.self.myPocket));

				if displayFlexSEA:
					i = self.printPocket(printDiv)
					return i
			#else:
				#print('This is not a Rigid packet')
		#else:
			#print('Invalid packet')

#Send Read Request ActPack:
	def requestReadPocket(self,offset):
		flexsea.ptx_cmd_pocket_rw(FLEXSEA_MANAGE_1, byref(self.nb), self.commStr, offset, self.pCtrl[0], self.pSetpoint[0], self.pSetGains[0], self.pG0[0], self.pG1[0], self.pG2[0], self.pG3[0], self.pCtrl[1], self.pSetpoint[1], self.pSetGains[1], self.pG0[1], self.pG1[1], self.pG2[1], self.pG3[1], self.pSystem);
		self.hser.write(self.commStr)
		if(self.pSetGains[0] == CHANGE):
			self.pSetGains[0] = c_uint8(KEEP)
		if(self.pSetGains[1] == CHANGE):
			self.pSetGains[1] = c_uint8(KEEP)

#Set Control Mode:
	def setControlMode(self,ctrlMode, ch=0):
		self.pCtrl[ch] = c_uint8(ctrlMode)

#Set Motor Voltage:
	def setMotorVoltage(self, mV, ch=0):
		self.pSetpoint[ch] = c_int32(mV)

#Set Motor Current:
	def setMotorCurrent(self, cur, ch=0):
		self.pSetpoint[ch] = c_int32(cur)

#Set Position Setpoint (Position & Impedance controllers):
	def setPosition(self, p, ch=0):
		self.pSetpoint[ch] = c_int32(p)

#Set Impedance controller gains.
#z_k & z_b: Impedance K & B
#i_kp & i_ki: Current Proportional & Integral
	def setZGains(self,z_k, z_b, i_kp, i_ki, ch=0):
		self.pG0[ch] = c_int16(z_k)
		self.pG1[ch] = c_int16(z_b)
		self.pG2[ch] = c_int16(i_kp)
		self.pG3[ch] = c_int16(i_ki)
		self.pSetGains[ch] = c_uint8(CHANGE)

#Display functions:
#==================

#Print Rigid data:
	def printRigid(self):
		print('Rigid')
		print('Gyro X:          ', self.myRigid.mn.gyro.x)
		print('Gyro Y:          ', self.myRigid.mn.gyro.y)
		print('Gyro Z:          ', self.myRigid.mn.gyro.z)
		print('Accel X:         ', self.myRigid.mn.accel.x)
		print('Accel Y:         ', self.myRigid.mn.accel.y)
		print('Accel Z:         ', self.myRigid.mn.accel.z)
		print('Motor angle:     ', self.myRigid.ex.enc_ang[0])
		print('Motor velocity:  ', self.myRigid.ex.enc_ang_vel[0])
		print('Motor current:   ', self.myRigid.ex.mot_current)
		print('Joint angle:     ', self.myRigid.ex.joint_ang[0])
		print('Joint velocity:  ', self.myRigid.ex.joint_ang_vel[0])
		print('Joint Ang-Mot:   ', self.myRigid.ex.joint_ang_from_mot[0])
		print('+VB:             ', self.myRigid.re.vb)
		print('Battery current: ', self.myRigid.re.current)
		print('Temperature:     ', self.myRigid.re.temp)
		print('6-ch strain #0:  ', self.myRigid.mn.genVar[0])
		print('...              ')

# #Print Pocket data:
	def printPocket_s(self):
		print('Pocket')
		print('Gyro X:          ', self.myPocket.mn.gyro.x)
		print('Gyro Y:          ', self.myPocket.mn.gyro.y)
		print('Gyro Z:          ', self.myPocket.mn.gyro.z)
		print('Accel X:         ', self.myPocket.mn.accel.x)
		print('Accel Y:         ', self.myPocket.mn.accel.y)
		print('Accel Z:         ', self.myPocket.mn.accel.z)
		print('Analog[0]:       ', self.myPocket.mn.analog[0])
		print('Analog[1]:       ', self.myPocket.mn.analog[1])

		print('M1 Angle:        ', self.myPocket.ex[0].enc_ang[0])
		print('M1 Velocity:     ', self.myPocket.ex[0].enc_ang_vel[0])
		print('M1 Current:      ', self.myPocket.ex[0].mot_current)
		print('M1 Voltage:      ', self.myPocket.ex[0].mot_volt)
		print('M1 Strain:       ', self.myPocket.ex[0].strain)

		print('M2 Angle:        ', self.myPocket.ex[1].enc_ang[0])
		print('M2 Velocity:     ', self.myPocket.ex[1].enc_ang_vel[0])
		print('M2 Current:      ', self.myPocket.ex[1].mot_current)
		print('M2 Voltage:      ', self.myPocket.ex[1].mot_volt)
		print('M2 Strain:       ', self.myPocket.ex[1].strain)

		print('+VB:             ', self.myPocket.re.vb)
		print('Battery current: ', self.myPocket.re.current)
		print('Temperature:     ', self.myPocket.re.temp)
		print('Status:          ', self.myPocket.re.status)

		print('genVar[0]:       ', self.myPocket.mn.genVar[0])
		print('genVar[1]:       ', self.myPocket.mn.genVar[1])
		print('genVar[2]:       ', self.myPocket.mn.genVar[2])
		print('genVar[3]:       ', self.myPocket.mn.genVar[3])
		print('...              ')

#Print ActPack data (Rigid + controller info):
	def printActPack(self,div):
		if(self.printDiv(div) == 0):
			self.clearTerminal()
			self.printController(self.pCtrl[0], self.pSetpoint[0], self.pG0[0], self.pG1[0], self.pG2[0], self.pG3[0], self.pSetGains[0])
			self.printRigid()
			return 0
		return 1

#Print Pocket data (pocket_s + controller info):
	def printPocket(self,div):
		if(self.printDiv(div) == 0):
			self.clearTerminal()
			self.printController(self.pCtrl[0], self.pSetpoint[0], self.pG0[0], self.pG1[0], self.pG2[0], self.pG3[0], self.pSetGains[0])
			self.printController(self.pCtrl[1], self.pSetpoint[1], self.pG0[1], self.pG1[1], self.pG2[1], self.pG3[1], self.pSetGains[1])
			self.printPocket_s()
			return 0
		return 1

#Prints easy to read info about the controller
	def printController(self,ctrl, sp, g0, g1, g2, g3, sg):
		c = self.mapCtrlText[ctrl]
		s = sp
		print('\nController:', c, '|', 'Setpoint:', s)
		print('Gains: [', g0, ', ', g1, ', ', g2, ', ', g3, '] (', sg, ')\n')

#Timing dividers:
	def printDiv(self,div):
		self.printDivider += 1
		if self.printDivider > div:
			self.printDivider = 0
		return self.printDivider

''''
#Functions below this line are kept mostly for legacy reasons. Use with care.
#=============================================================================

#Send Read Request Rigid:
def requestReadRigid(offset):
	flexsea.ptx_cmd_rigid_r(FLEXSEA_MANAGE_1, byref(self.nb), self.commStr, offset);
	self.hser.write(self.commStr)

#Set Control Mode:
def setControlMode_manual(ctrlMode):
	flexsea.ptx_cmd_ctrl_mode_w(FLEXSEA_EXECUTE_1, byref(self.nb), self.commStr, ctrlMode);
	self.hser.write(self.commStr)

#Set Motor Voltage:
def setMotorVoltage_manual(mV):
	flexsea.ptx_cmd_ctrl_o_w(FLEXSEA_EXECUTE_1, byref(self.nb), self.commStr, mV);
	self.hser.write(self.commStr)
	
def readRigid():

	requestReadRigid(offs())
	bytes = serialBytesReady(100, COMM_STR_LEN)
	
	#s = self.hser.read(bytes)
	s = self.hser.read(COMM_STR_LEN) #Reading a fixed length for now
	for i in range(0,bytes-1):
		#print(s[i], end=' ')
		self.cBytes[i] = s[i]
	#print(']')

	ppFlag = c_uint8(0)
	ppFlag = flexsea.receiveFlexSEABytes(byref(self.cBytes), bytes, 1);
	if(ppFlag):
		#print('We parsed a packet: ', end='')
		cmd = c_uint8(0)
		type = c_uint8(0)
		flexsea.getSignatureOfLastPayloadParsed(byref(cmd), byref(type));
		#print('cmd:', cmd, 'type:', type)
		
		newRigidPacket = c_uint8(0)
		newRigidPacket = flexsea.newRigidRRpacketAvailable();
		
		if(newRigidPacket):
			#print('New Rigid packet(s) available\n')
			flexsea.getLastRigidData(byref(self.myRigid));
			printRigid()
		#else:
			#print('This is not a Rigid packet')
	#else:
		#print('Invalid packet')
'''
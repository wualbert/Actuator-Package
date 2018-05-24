# dyno_parameters.py
#=-=-=-=-=-=-=-=-=-=-=-=
# Parameters for dyno test
# 2018 Albert Wu

#display related
REFRESH_RATE = 0.01   # seconds, communication & FSM
DISP_DIV = 5       # We refresh the display every 50th packet

#Dyno Parameters
#All voltages in mV
#All times in seconds
#Parameters for the tested motor
V_MIN = 0
V_MAX = 4000
V_STEP_SIZE = 1000
ZERO_SPEED_MARGIN = 0

#Parameters for the load motor
V_LOAD_MIN = 0
V_LOAD_MAX = 2000
V_LOAD_STEP_SIZE = 500

#Waiting time
CHANGE_LOAD_WAIT = 1   #seconds
CHANGE_TEST_WAIT = 2
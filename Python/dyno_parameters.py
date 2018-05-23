# dyno_parameters.py
#=-=-=-=-=-=-=-=-=-=-=-=
# Parameters for dyno test
# 2018 Albert Wu

#display related
REFRESH_RATE = 0.005   # seconds, communication & FSM
DISP_DIV = 50       # We refresh the display every 50th packet

#Dyno Parameters
#Parameters for the tested motor
V_MIN = 0
V_MAX = 10
V_STEP_SIZE = 1
ZERO_SPEED_MARGIN = 1

#Parameters for the load motor
V_LOAD_MIN = 0
V_LOAD_MAX = 10
V_LOAD_STEP_SIZE = 1

#Waiting time
CHANGE_LOAD_WAIT = 1   #seconds
CHANGE_TEST_WAIT = 2
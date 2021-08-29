# -*- coding: utf-8 -*-

"""
state.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

This class is called from "sensor.py" and "image.py", and estimate state variables.
This class is factory class. Model (state type & estimation model) is selected by argment.

"""

from state_coplanarity import StateCoplanarity
from state_RBPF import StateRBPF
from state_IMU_KF import StateIMUKF
from state_IMU_PF import StateIMUPF

class State:

	def __init__(self):
		pass

	def getStateClass(self,stateType):
		if(stateType=="Coplanarity"):
			return StateCoplanarity()
		elif(stateType=="RBPF"):
			return StateRBPF()
		elif(stateType=="IMUKF"):
			return StateIMUKF()
		elif(stateType=="IMUPF"):
			state = StateIMUPF()
			state.initWithType("IMUPF")
			return state
		elif(stateType=="IMUPF2"):
			state = StateIMUPF()
			state.initWithType("IMUPF2")
			return state
		else:
			pass

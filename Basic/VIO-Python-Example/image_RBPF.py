# -*- coding: utf-8 -*-

"""
image.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

This class is called from "Main.py", and process image data.

methods:
	processData(data) <- called from "Main.py"
"""

import sys
from math import *
import cv2 as cv
import numpy as np
import Util
from keypoint import KeyPoint

class ImageRBPF:

	def __init__(self):
		pass

	def init(self):
		#state.py
		#self.state.init() #this is also called in sensor.py
		#variables
		pass

	def setState(self,_state):
		#state.py
		self.state = _state



	#Set new data and Execute all functions
	def processData(self,time_,data):

		#if nomatch then nothing to do
		if(data[0] == "nomatch"):
			#print("nomatch")
			self.state.setImageData(0.0,"nomatch")
			return

		time = (float(long(time_) / 1000.0))

		keypoints = []
		for d in data:
			if(d != ''):
				keypoints.append(KeyPoint(d))

		self.state.setImageData(time,keypoints)


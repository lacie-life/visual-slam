# -*- coding: utf-8 -*-

"""
keypoint.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

Class for key points pair between images
"""

class KeyPointPair:

	def __init__(self,data_):
		
		cx = 360.0 - 6.361694 # (image size X)/2 +  principal point X
		cy = 640.0 - 22.962158 # (image size Y)/2 +  principal point Y
		
		data = data_.split(':')
		
		self.prevIndex = int(data[0])
		self.index = int(data[1])
		self.x1 = float(data[2]) - cx
		self.y1 = float(data[3]) - cy
		self.x2 = float(data[4]) - cx
		self.y2 = float(data[5]) - cy
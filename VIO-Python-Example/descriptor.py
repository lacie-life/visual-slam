# -*- coding: utf-8 -*-

"""
descriptor.py

author: Keita Nagara　永良慶太 (University of Tokyo) <nagara.keita()gmail.com>

Class for descriptor in image
"""

class Descriptor:

	def __init__(self,data_):
		
		data__ = data_.split(',')
		self.data = []
		
		for d in data__:
			if(d != ''):
				self.data.append(int(d))
		
		
	def printData(self):
		
		for d in self.data:
			print(d),

		print(" ")
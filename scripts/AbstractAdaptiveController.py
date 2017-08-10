#!/usr/bin/env python

from abc import ABCMeta, abstractmethod, abstractproperty
import rospy
from geometry_msgs.msg import Vector3
from numpy import matrix
from numpy import linalg

class AbstractAdaptiveController:
	__metaclass__ = ABCMeta

	def updateRefTime(self):
		self.refTime = rospy.get_time()

	def __init__(self):
		pass

	@abstractproperty
	def Am(self):
		pass

	@abstractproperty
	def Bm(self):
		pass

	def getXm(self):
		return self.xm

	def setXm(self,val):
		self.xm = val

	xm = abstractproperty(getXm,setXm)

	@abstractmethod
	def stateCallback(self):
		pass

	def refCallback(self,data):
		dt = rospy.get_time() - self.refTime
		self.xm = self.xm + dt*(Am*self.xm+Bm*matrix([[data.x],[data.y],[data.z]]))
		self.r = matrix([[data.x],[data.y],[data.z]])
		self.updateRefTime()
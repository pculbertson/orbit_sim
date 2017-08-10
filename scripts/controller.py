#!/usr/bin/env python
from AbstractAdaptiveController import AbstractAdaptiveController
import rospy
import random
import sys
from geometry_msgs.msg import *
from gazebo_msgs.msg import LinkStates
from orbit_sim.msg import refSig
from numpy import matrix
from numpy import array
from scipy import linalg
from tf import transformations

class AdaptiveController():
	def __init__(self,argv):
		random.seed(10*int(argv[1]))
		rospy.init_node('controller',anonymous=True)
		#define constants
		rospy.loginfo("Hi there")
		self.xm = matrix([[0],[0],[0]])
		self.Am = matrix([[-.1, 0, 0],[0, -.1, 0],[0, 0, -.1]])
		self.Bm = matrix([[1,0,0],[0,1,0],[0,0,1]])
		self.Kx = .1*random.random()*matrix([[1,0,0],[0,1,0],[0,0,1]])
		self.Kr = .1*random.random()*matrix([[1,0,0],[0,1,0],[0,0,1]])
		self.J = .00001*random.random()*matrix([[0,0,0,0,0,1],[0,0,0,0,1,0],[0,0,0,1,0,0]])
		self.r = matrix([[0],[0],[0]])
		self.stateTime = rospy.get_time()
		self.robotNum = int(argv[1])
		self.gainx = 5e1*matrix([[1,0,0],[0,1,0],[0,0,1]])
		self.gainr = 5e1*matrix([[1,0,0],[0,1,0],[0,0,1]])
		self.gainJ = 1e1*matrix([[1,0,0],[0,1,0],[0,0,1]])
		self.P = linalg.solve_lyapunov(self.Am,-matrix([[1,0,0],[0,1,0],[0,0,1]]))
		rospy.loginfo(self.P)
		#setup subscribers
		rospy.Subscriber('/gazebo/link_states_drop',LinkStates,self.stateCallback)
		rospy.Subscriber('refSignal',refSig,self.refCallback)
		self.pubCom = rospy.Publisher('wrenchPlugin/wrenchCommands',Wrench,queue_size=10)
		self.pubx = rospy.Publisher('controllerState',Vector3,queue_size=10)
		self.pubxm = rospy.Publisher('modelState',Vector3,queue_size = 10)
		rospy.spin()

		

	def updateStateTime(self):
		self.stateTime = rospy.get_time()

	def fOmega(self,x):
		return matrix([[x.item(0)**2],[x.item(1)**2],[x.item(2)**2],[x.item(0)*x.item(1)],[x.item(0)*x.item(2)],[x.item(1)*x.item(2)]])

	def stateCallback(self,data):
		#parse message
		if self.stateTime == 0:
			self.updateStateTime()
		else:
			pose = data.pose[self.robotNum]
			twist = data.twist[self.robotNum]
			quat = array([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
			rotMat = transformations.quaternion_matrix(quat)[0:3,0:3]
			x = rotMat*matrix([[twist.angular.x],[twist.angular.y],[twist.angular.z]])
			#update parameters
			e = x-self.xm
			dt = rospy.get_time()-self.stateTime
			self.Kx = self.Kx + dt*self.gainx*(-self.Bm.T*self.P*e*x.T - .000001*self.Kx)
			self.Kr = self.Kr + dt*self.gainr*(-self.Bm.T*self.P*e*self.r.T- .000001*self.Kr)
			self.J = self.J + dt*self.gainJ*(-self.Bm.T*self.P*e*self.fOmega(x).T-.0000005*self.J)
			#rospy.loginfo(self.J)
			#rospy.loginfo(x)
			#apply reaction torque
			reaction = self.Kx*x + self.Kr*self.r + self.J*self.fOmega(x)
			rospy.loginfo(self.Kx)
			#send wrench command to plugin
			command_msg = Wrench(Vector3(0,0,0),Vector3(reaction.item(0),reaction.item(1),reaction.item(2)))
			self.pubCom.publish(command_msg)
			self.pubx.publish(Vector3(x.item(0),x.item(1),x.item(2)))
			self.pubxm.publish(Vector3(self.xm.item(0),self.xm.item(1),self.xm.item(2)))
			self.updateStateTime()


	def refCallback(self,data):
		self.xm = matrix([[data.xm.x],[data.xm.y],[data.xm.z]])
		self.r = matrix([[data.r.x],[data.r.y],[data.r.z]])

def doNothing():
	rospy.spin()

if __name__ == '__main__':
	AdaptiveController(sys.argv)

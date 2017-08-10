#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
from orbit_sim.msg import refSig
from numpy import math
from numpy import matrix
from scipy import signal

class RefNode():
	def __init__(self):
		self.pub = rospy.Publisher('refSignal',refSig,queue_size=0)
		rospy.init_node('refSignal',anonymous=True)
		self.rate = rospy.Rate(100000)
		self.refTime = rospy.get_time()
		self.Am = matrix([[-.1, 0, 0],[0, -.1, 0],[0, 0, -.1]])
		self.Bm = matrix([[1,0,0],[0,1,0],[0,0,1]])
		self.xm = matrix([[0],[0],[0]])

	def refOut(self):
		while not rospy.is_shutdown():
			if self.refTime == 0:
				self.refTime = rospy.get_time()
				self.rate.sleep()
			else:
				refVal= 0.5*math.sin(rospy.get_time())
				r = Vector3(refVal,refVal,refVal)
				dt = rospy.get_time()-self.refTime
				self.xm = self.xm + dt*(self.Am*self.xm+self.Bm*matrix([[r.x],[r.y],[r.z]]))
				msg = refSig(r,Vector3(self.xm.item(0),self.xm.item(1),self.xm.item(2)))
				self.pub.publish(msg)
				self.refTime = rospy.get_time()
				self.rate.sleep()

if __name__ == '__main__':
	try:
		rn = RefNode()
		rn.refOut()
	except rospy.ROSInterruptException:
		pass
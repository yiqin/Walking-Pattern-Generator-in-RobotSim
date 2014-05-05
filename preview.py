import sys
from robot import *
from robot import robotcollide
from geometry import vectorops
from geometry import collide
from geometry.glprogram import *

from geometry import so3 #####
from robot import ik #####

import string
import numpy as np
from PIL import Image
import time

dt = 0.005


class Hubo(GLRealtimeProgram):
	def __init__(self,world):
		GLRealtimeProgram.__init__(self,"Hubo constrainted COM w/ hand")
		self.world = world
		self.sim = Simulator(world)
		self.dt = dt
		#self.collider = robotcollide.WorldCollider(world)
	
		#! init config
		#self.qdes = [0.0]*63
		self.dqdes = [0.0]*63
		self.qdes = [0,0,-0.07,0,0,0,\
			0,0,\
			0,0,0,0.0,0,0,\
			0,0,0,\
			0,0,0,\
			0,0,0,\
			0,0,0,\
			0,0,0,\
			0,0,0,0.0,0,0,\
			0,0,0,\
			0,0,0,\
			0,0,0,\
			0,0,0,\
			0,0,0,\
			0,\
			0,0,0,0,0,0,\
			0,0,0,0,0,0]


		self.pdes = [0.0]*3
		#! mass and com
		self.mass = 0.0
		for i in xrange(self.world.robot(0).numLinks()):
			mi = self.world.robot(0).getLink(i).getMass().mass
			self.mass += mi
		self.com = self.q2pcom(self.qdes)		
		self.com_last = self.com
		

		#  use of Iterm
		self.eI = [0.0]*3
		self.Vcmded = [0.0]*3

		#! start simulate flag
		self.start = False
		#  set this to true to step through the program manually
		self.step = True
		self.sim.simulate(0.0001)
		
	def display(self):
		#draw the world
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		#to revert back to the simulation state
		self.sim.updateWorld() 	
		self.world.drawGL()
		glEnable(GL_DEPTH_TEST)
		glDisable(GL_LIGHTING)
		#draw lines
		glBegin(GL_LINES)
		glColor3f(1,1,1)
		glVertex3fv(vectorops.madd(self.com, [1,0,0], 1))
		glVertex3fv(vectorops.madd(self.com, [-1,0,0], 1))
		glVertex3fv(vectorops.madd(self.com, [0,1,0], 1))
		glVertex3fv(vectorops.madd(self.com, [0,-1,0], 1))
		glVertex3fv(vectorops.madd(self.com, [0,0,1], 1))
		glVertex3fv(vectorops.madd(self.com, [0,0,-1], 1))
		glColor3f(1,0,0)
		glVertex3fv(vectorops.madd(self.pdes, [1,0,0], 1))
		glVertex3fv(vectorops.madd(self.pdes, [-1,0,0], 1))
		glVertex3fv(vectorops.madd(self.pdes, [0,1,0], 1))
		glVertex3fv(vectorops.madd(self.pdes, [0,-1,0], 1))
		glVertex3fv(vectorops.madd(self.pdes, [0,0,1], 1))
		glVertex3fv(vectorops.madd(self.pdes, [0,0,-1], 1))
		glEnd()

	def q2pcom(self, q):
		self.world.robot(0).setConfig(q)
		com_n = (0.0, 0.0, 0.0)
		for i in xrange(self.world.robot(0).numLinks()):
			link = self.world.robot(0).getLink(i)
			comi = link.getWorldPosition(link.getMass().com)
			mi = link.getMass().mass
			com_n = vectorops.madd(com_n, comi, mi) 
		com = vectorops.div(com_n, self.mass)
		return com
		
	def Jcom(self, q):
		self.world.robot(0).setConfig(q)
		numLinks = self.world.robot(0).numLinks()
		Jcom = [[0.0]*numLinks] * 3
		#- Jcomi
		for i in xrange(numLinks):
			link = self.world.robot(0).getLink(i)
			mi = link.getMass().mass
			comi = link.getMass().com
			Ji = link.getPositionJacobian(comi)
			Ji = [vectorops.mul(Jii, mi) for Jii in Ji] 
			for j in xrange(3):
				Jcom[j] = vectorops.add(Jcom[j], Ji[j])
		Jcom = [vectorops.div(Jcomi, self.mass) for Jcomi in Jcom]
		return Jcom


	def solve_ik(self, robotlink, localpos, worldpos):

		self.linkindex = robotlink.index
		
		self.obj = IKObjective()
		self.obj.setFixedPoint(self.linkindex, localpos, worldpos)	
		self.s = IKSolver(robotlink.getRobot())
		self.s.add(self.obj)
		robotlink.getRobot().setConfig([0]*robotlink.getRobot().numLinks())
		(self.res, self.iters) = self.s.solve(100,1e-4)	
		
		#print "hello"
		print self.iters
		#time.sleep(1000)
		print np.shape(self.world.robot(0).getConfig())
		#time.sleep(1000)
		return self.world.robot(0).getConfig()



	def idle(self):
		if self.start:
			self.control_loop()
		elif not self.step:
			self.control_loop()



	def control_loop(self):
			robot = self.world.robot(0)
			controller = self.sim.getController(0)
			print "time"
			print self.ttotal

			q = controller.getSensedConfig()
			#dq = controller.getSensedVelocity()
			qcmd = controller.getCommandedConfig()	
			dqcmd = controller.getCommandedVelocity()

			self.com = self.q2pcom(q)
			#print self.qdes
			#print self.com
			com_cmded = self.q2pcom(qcmd)

#########################################
			# IK Solution
			self.lhlink =  self.world.robot(0).getLink(13)
			self.qdes = self.solve_ik(self.lhlink,[0., 0., 0.],[4, -2.216, 2])		


















#########################################
			### IK Solution

			#self.rxLhand = [0, 0.216, 0.57788, 0, 0.01144, -0.04614]
			#self.rxLhand = [0, -0.19316, -0.67, 0, 0.011442, -0.0461472]
			
			#print np.shape(robot.getLink(13).getJacobian([0., 0., 0.]))
			#print "xLhand"
			#print self.xLhanddirection
						
			#self.cnt = 1
			#self.errorIK = 10000
			#qupdate = self.qdes
			#while np.linalg.norm(self.errorIK) > 0.0001:
			#	self.world.robot(0).setConfig(qupdate)
			#	JLhand = robot.getLink(13).getJacobian([0., 0., 0.])
#
#
#				#xLhand = self.world.robot(0).getLink(13).getWorldPosition
#				xLhandlink = robot.getLink(13)
#				self.xLhandposition = xLhandlink.getLocalPosition(xLhandlink.getMass().com)
#				self.xLhanddirection = xLhandlink.getLocalDirection(xLhandlink.getMass().com)
#				self.xLhand = np.hstack((self.xLhandposition,self.xLhanddirection))
#
#				self.errorIK = vectorops.sub(self.rxLhand, self.xLhand)
#				self.errorIK = vectorops.div(self.errorIK,self.dt)
#
#				Jinv = np.linalg.pinv(JLhand, rcond=1e-3)
#				qdLhand = np.dot(np.linalg.pinv(JLhand),self.errorIK)
#				qupdate = vectorops.madd(qupdate, qdLhand,self.dt)
#	
#				self.cnt = self.cnt + 1;
#				print "iteration times: "
#				print self.cnt		
#				print self.xLhandposition				
#				print np.linalg.norm(self.errorIK)
#				#print np.shape(robot.getLink(13).getJacobian([0., 0., 0.]))
#				#time.sleep(1000)
#				#if self.cnt == 3:
#					#time.sleep(1000)
#



#########################################

#########################################

#-	v = [vcom, vRfoot, vLfoot]
			#vfoot = [0]*12
			#V = np.hstack((self.vcom, vfoot))
#########################################
			#Jinv = np.linalg.pinv(np.array(J), rcond=1e-3)
			#self.dqdes = np.dot(Jinv, np.array(V))

			limits = robot.getVelocityLimits()
			m = max(vectorops.div(self.dqdes, limits))
			if m > 1:
				for i in xrange(len(self.dqdes)):
					self.dqdes[i] /= m

			#self.qdes = vectorops.madd(qcmd, self.dqdes, self.dt)
#########################################
			controller.setPIDCommand(self.qdes, self.dqdes)
			self.sim.simulate(self.dt)

	def keyboardfunc(self,key,x,y):
		self.ttotal = 0.0
		if key == 'o':
			self.start = not self.start
			#turn off manual mode
			self.step = not self.step
		elif self.step:
			self.control_loop()
		glutPostRedisplay()


if __name__ == "__main__":
	world = WorldModel()
	res = None
	res = world.readFile("../hubo_files/hubo_plane.xml")
	if not res:
		raise RuntimeError("Unable to load model")
	task = Hubo(world)
	task.run()

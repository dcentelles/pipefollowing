#!/usr/bin/env python

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

#import service library
from std_srvs.srv import Empty
import math

class imageGrabber:

  ##Init function, create subscriber and required vars.
  def __init__(self):
    image_sub = rospy.Subscriber("/g500/camera1",Image,self.image_callback)
    self.bridge = CvBridge()
    self.height=-1
    self.width=-1
    self.channels=-1

    self.center = None
    self.dir = None
    self.giroALaDerecha = False #Hay un giro a la derecha
    self.hayVerticales = False  #Hay lineas verticales
    self.verticalesDetras = False #Dejando las verticales detras
    self.hayGiro = False #Hay giros
    self.aLaDerecha = False #tuberia vertical a la derecha
    self.centrado = True    #tuberia vertical centrada
    self.inclinacionDerecha = False #tuberia vertical ligeramente inclinada a la derecha
    self.enVertical = True #tuberia vertical completamente vertical

    self.grosor = None
  ##Image received -> process the image
  def image_callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      binar = cv2.inRange(cv_image,np.array([0,0,0]),np.array([40,255,40]))
      edges  = cv2.Canny(binar, 50, 150, apertureSize = 3)
      lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, 90, 10)
      line0 = None
      line1 = None
      self.height, self.width, self.channels = cv_image.shape
      self.imcenter = (self.width/2,self.height/2)

      vlines = []
      hlines = []
      dumbral = 0.5
      xvlines = 0
      yvlines = 0
      xhlines = 0
      yhlines = 0

      vdir = 0
      vxmin = 2000
      vxmax = -2000

      print "#########"
      if lines!=None:
		pos = 0
		for x1,y1,x2,y2 in lines[0]:
			cv2.line(cv_image,(x1,y1),(x2,y2),(0,0,255),2)
			dx = x2-x1
			dy = y2-y1
			dir = math.atan2(dy,dx)
			adir = math.fabs(dir)
			line = ((x1,y1),(x2,y2))
			if adir > dumbral:
				vlines.append(line)
				xvlines += x1+x2
				yvlines += y1+y2
				vdir += dir
				if x1 > vxmax:
					vxmax = x1
				if x2 > vxmax:
				  	vxmax = x2
				if x1 < vxmin:
					vxmin = x1
				if x2 < vxmin:
					vxmin = x2
			else:
				hlines.append(line)
				xhlines += x1+x2
				yhlines += y1+y2
	      
      vdumbral = int(self.height/3)
      
      lvlines = len(vlines)
      lhlines = len(hlines)

      tv = lvlines*2
      th = lhlines*2

      self.grosor = None
      if tv > 0:
      		self.grosor = math.fabs(vxmax - vxmin)
      		self.hayVerticales = True
		vdir = vdir / lvlines
		cvlines = (xvlines/tv,yvlines/tv)
		if vdir < -0.3:
			self.inclinacionDerecha = True
			self.enVertical = False
		elif vdir > 0.3:
			self.inclinacionDerecha = False
			self.enVertical = False
		elif vdir == 0:
			self.inclinacionDerecha = False
			self.enVertical = True


		xpipe = cvlines[0]
		ypipe = cvlines[1]
		ximg = self.imcenter[0]
		yimg = self.imcenter[1]
		dcenter = (xpipe-ximg, ypipe-yimg);
		dx = dcenter[0]
		dy = dcenter[1]
		if dy < vdumbral:
			self.verticalesDetras = False
		else:
			self.verticalesDetras = True
		if(dx > 5):
			self.aLaDerecha = True
			self.centrado = False
		elif dx < -5:
			self.aLaDerecha = False
			self.centrado = False
		else:
			self.aLaDerecha = False
			self.centrado = True
		cv2.circle(cv_image, cvlines, 5, (20,20,20),-1)
      else:
      	self.hayVerticales = False

      if th > 0:
		chlines = (xhlines/th,yhlines/th)
		xpipe = chlines[0]
		ypipe = chlines[1]
		ximg = self.imcenter[0]
		yimg = self.imcenter[1]
		dcenter = (xpipe-ximg, ypipe-yimg);
		dx = dcenter[0]
		dy = dcenter[1]
		if dy < vdumbral:
			self.hayGiro = True
			if(dx > 0):
				self.giroALaDerecha = True
			elif dx < 0:
				self.giroALaDerecha = False
	
		else:
			self.hayGiro = False
		cv2.circle(cv_image, chlines, 5, (100,100,255),-1)
      else:
      	self.hayGiro = False

      	
    except CvBridgeError as e:
      print(e)

    cv2.circle(cv_image, self.imcenter, 5, (100,100,100),-1)
    cv2.imshow("Image window", cv_image)
    cv2.imshow("Segmentation", binar)
    cv2.imshow("Edges", edges)
    cv2.waitKey(3)

  ##Return size of the image
  def getSize(self):
    return self.width,self.height

if __name__ == '__main__':
  #topic to command
  twist_topic="/g500/velocityCommand"
  #base velocity for the teleoperation (0.2 m/s) / (0.2rad/s)
  baseVelocity=0.5

  ##create the publisher
  rospy.init_node('pipeFollowing')
  pub = rospy.Publisher(twist_topic, TwistStamped,queue_size=1)
  
 # ##wait for benchmark init service
 # rospy.wait_for_service('/startBench')
 # start=rospy.ServiceProxy('/startBench', Empty)
 # 
 # ##wait for benchmark stop service
 # rospy.wait_for_service('/stopBench')
 # stop=rospy.ServiceProxy('/stopBench', Empty)

  #Create the imageGrabber
  IG=imageGrabber()
  
 # start()
  while not rospy.is_shutdown():
    msg = TwistStamped()

    #get width x height of the last received image
    imwidth,imheight=IG.getSize()

    msg.twist.linear.x=0
    msg.twist.linear.y=0
    msg.twist.linear.z=0
    msg.twist.angular.z=0.0
    if IG.grosor != None:
    	    print IG.grosor
	    if IG.grosor < 300:
		    if IG.grosor > 35:
			print "Subir"
			msg.twist.linear.z = -baseVelocity*0.6
		    elif IG.grosor < 30:
			print "Bajar"
			msg.twist.linear.z = baseVelocity*0.6
	    else:
	    	print "Bajar rapido"
		msg.twist.linear.z = baseVelocity*3
    if IG.hayGiro:
	if IG.hayVerticales:
		if IG.verticalesDetras:
			msg.twist.linear.x=0
			if IG.giroALaDerecha:
				print "(verticales detras) Girar a la derecha yendo lateralmente a la derecha"
				msg.twist.linear.y=baseVelocity
				msg.twist.angular.z=baseVelocity

			else:
				print "(verticales detras) Girar a la izquierda yendo lateralmente a a la izquierda"
				msg.twist.linear.y=-baseVelocity
				msg.twist.angular.z=-baseVelocity

		else:
			msg.twist.linear.x=baseVelocity*2
			msg.twist.linear.y=0
			if IG.giroALaDerecha:
				print "(verticales detras) Girar a la derecha yendo hacia adelante"
				msg.twist.angular.z=baseVelocity
			else:
				print "(verticales detras) Girar a la izquierda yendo hacia adelante"
				msg.twist.angular.z=-baseVelocity
	elif IG.giroALaDerecha:
		print "Girar a la derecha yendo lateralmente a la derecha"
		msg.twist.linear.y=baseVelocity
		msg.twist.angular.z=baseVelocity
	else:
		print "Girar a la izquierda yendo lateralmente a la izquierda"
		msg.twist.linear.y=-baseVelocity
		msg.twist.angular.z=-baseVelocity
		

    elif IG.hayVerticales:
    	msg.twist.linear.x=baseVelocity*2
	if IG.enVertical:
		if IG.centrado:
			print "Seguir recto"
		elif IG.aLaDerecha:
			print "seguir recto moviendose lateralmente a la derecha"
			msg.twist.linear.y=baseVelocity
		else:
			print "seguir recto moviendose lateralmente a la izquierda"
			msg.twist.linear.y=-baseVelocity
	elif IG.inclinacionDerecha:
		msg.twist.angular.z=baseVelocity
		if IG.centrado:
			print "Seguir recto girando a la derehca"
		elif IG.aLaDerecha:
			print "seguir recto moviendose lateralmente a la derecha girando a la derecha"

			msg.twist.linear.y=baseVelocity
		else:
			print "seguir recto moviendose lateralmente a la izquierda girando a la derecha"
			msg.twist.linear.y=-baseVelocity

	else:
		msg.twist.angular.z=-baseVelocity
		if IG.centrado:
			print "Seguir recto girando a la izquierda"
		elif IG.aLaDerecha:
			print "seguir recto moviendose lateralmente a la derecha girando a la izquierda"
			msg.twist.linear.y=baseVelocity

		else:
			print "seguir recto moviendose lateralmente a la izquierda girando a la izquierda"

			msg.twist.linear.y=-baseVelocity

	
    else:
	print "Subir e ir hacia atras"
	msg.twist.linear.z = -baseVelocity
	msg.twist.linear.x = -baseVelocity



    pub.publish(msg)
  #  if IG.enVertical:
  #  	msg.twist.linear.x = baseVelocity
  #  elif IG.inclinacionDerecha:
  #  	msg.twist.angular.z = baseVelocity
  #  else:
  #  	msg.twist.angular.z = -baseVelocity

  #  if IG.centrado:
  #  	pass
  #  elif IG.aLaDerecha:
  #  	msg.twist.linear.y = baseVelocity
  #  else:
  #  	msg.twist.linear.y = -baseVelocity

  #  pub.publish(msg)
    
    rospy.sleep(0.1)
  
  #stop()


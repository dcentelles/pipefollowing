#!/usr/bin/env python

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

#import service library
from std_srvs.srv import Empty
import math
from scipy.interpolate import interp1d

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
      dumbral = 0.9
      xvlines = 0
      yvlines = 0
      xhlines = 0
      yhlines = 0

      vdir = 0
      hdir = 0
      vxmin = 2000
      vxmax = -2000

      #print "#########"
      if lines is not None and len(lines)>0:
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
				hdir += dir
	      
      vdumbral = int(self.height/3)
      
      lvlines = len(vlines)
      lhlines = len(hlines)

      tv = lvlines*2
      th = lhlines*2

      self.grosor = cv2.countNonZero(binar)
      if tv > 0:
      		self.hayVerticales = True
		vdir = vdir / lvlines
		self.vdir = vdir
		cvlines = (xvlines/tv,yvlines/tv)
		if vdir < 0:
			self.inclinacionDerecha = True
			self.enVertical = False
		elif vdir > 0:
			self.inclinacionDerecha = False
			self.enVertical = False
		else:
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
		self.dx = dx
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

		if tv == 0 or self.verticalesDetras:
			dx = dcenter[0]
			dy = dcenter[1]
			if dy < vdumbral:
				self.hayGiro = True #Horizontales delante
				if(dx > 0):
					self.giroALaDerecha = True
				else:
					self.giroALaDerecha = False

			else:
				self.hayGiro = False
		else:
			self.hayGiro = False
#		else:
#			self.hayGiro = True
#			hdir = hdir / th
#			if hdir < 0:
#				self.giroALaDerecha = True
#				
#			elif hdir > 0:
#				self.giroALaDerecha = False

			
		cv2.circle(cv_image, chlines, 5, (100,100,255),-1)
      else:
      	self.hayGiro = False

      	
    except CvBridgeError as e:
      print(e)

    cv2.circle(cv_image, self.imcenter, 5, (100,100,100),-1)
    cv2.imshow("Image window", cv_image)
 #   cv2.imshow("Segmentation", binar)
 #   cv2.imshow("Edges", edges)
    cv2.waitKey(3)

  ##Return size of the image
  def getSize(self):
    return self.width,self.height


def getPower(dif, m):
	return float(m(dif))

import signal
import sys

stop = None

def signal_handler(signal, frame):
        print('You pressed Ctrl+C! Stoping benchmarch...')
	stop()
	print("Stoped!")
	
        sys.exit(0)

if __name__ == '__main__':
  global stop
  #topic to command
  thrusters_topic="/g500/thrusters_input"
  #base velocity for the teleoperation (0.2 m/s) / (0.2rad/s)
  baseVelocity=0.5

  ##create the publisher
  rospy.init_node('pipeFollowing')
  pub = rospy.Publisher(thrusters_topic, Float64MultiArray,queue_size=1)
  
 # ##wait for benchmark init service
  rospy.wait_for_service('/startBench')
  start=rospy.ServiceProxy('/startBench', Empty)
 # 
 # ##wait for benchmark stop service
  rospy.wait_for_service('/stopBench')
  stop=rospy.ServiceProxy('/stopBench', Empty)

  #Create the imageGrabber
  IG=imageGrabber()
 
  ymax = 200
  m = interp1d([0,ymax],[0,1])
  start()

  signal.signal(signal.SIGINT, signal_handler)
  zintegral = 0
  prezerror = 0

  while not rospy.is_shutdown():
    msg = Float64MultiArray()
    msg.data = [0,0,0,0,0]

    #get width x height of the last received image
    imwidth,imheight=IG.getSize()

    perdido = False

    if IG.hayGiro:
	desfase = 0.8
	msg.data[0] = -1
	msg.data[1] = -1

	if IG.giroALaDerecha:
	#	print "GIRO A LA DERECHA PRONUNCIADO"
		msg.data[1] = desfase
	else:
	#	print "GIRO A LA IZQUIERDA PRONUNCIADO"
		msg.data[0] = desfase

    elif IG.hayVerticales:
    	msg.data[0] = -baseVelocity
    	msg.data[1] = -baseVelocity
	#print IG.vdir
    	desfase = baseVelocity*0.8
	dx = min(math.fabs(IG.dx), ymax)
	if IG.centrado:
		pass

	elif IG.aLaDerecha:
		power = getPower(dx,m)
	#	print "Moviendose lateralmente a la derecha " + str(power)
		msg.data[4] = power
	else:
		power = getPower(dx,m)
	#	print "Moviendose lateralmente a la izquierda " +str(power)
		msg.data[4] = -power

	if IG.enVertical:
		pass

	elif IG.inclinacionDerecha:
	#	print "Hacia adelante girando a la derecha"
		msg.data[1] = desfase
	else:
	#	print "Hacia adelante girando a la izquierda"
		msg.data[0] = desfase

	
    else:
    	perdido = True
        zintegral = 0
        prezerror = 0
	#print "Subir e ir hacia atras"
	msg.data[0] = baseVelocity*0.5
    	msg.data[1] = baseVelocity*0.5

	msg.data[2] = baseVelocity*0.5
	msg.data[3] = baseVelocity*0.5


    kp = 0.2
    kd = 1
    ki = 0.01
    dt = 0.1
    zmax = 1000
    zmin = zmax * -1
    zm = interp1d([zmin,zmax],[-1,1])

    # double dt, double max, double min, double Kp, double Kd, double Ki 
    # ID(0.1, 100, -100, 0.1, 0.01, 0.5
    if not perdido and IG.grosor != None:

	    altura = 4250
            zerror = altura-IG.grosor
            
            pout = kp * zerror
            
            zintegral = zintegral + zerror*dt
            iout = ki * zintegral

            zderivative = (zerror - prezerror) / dt
            dout = kd * zderivative
            prezerror = zerror
            
            rrpower = pout + iout + dout
            rpower = rrpower
            if(rrpower > zmax):
                rpower = zmax
            elif(rrpower < zmin):
                rpower = zmin

	    power = getPower(rpower, zm) 
            power = power * -1
	
            #print("Zmax: {} T: {} V: {} zerror: {} P: {} I: {} D: {} RRPOWER: {} RPOWER: {} POWER: {}".format(zmax, altura, IG.grosor, zerror, pout, iout, dout, rrpower, rpower, power))
            
            print("Zmax: {} T: {} V: {} zerror: {} P: {} D: {} RRPOWER: {} RPOWER: {} POWER: {}".format(zmax, altura, IG.grosor, zerror, pout, dout, rrpower, rpower, power))
            msg.data[2] = power
	    msg.data[3] = power

    pub.publish(msg)
    
    rospy.sleep(dt)
  


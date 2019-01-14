#!/usr/bin/env python
from std_msgs.msg import Float64MultiArray
import termios, fcntl, sys, os
import rospy

#import service library
from std_srvs.srv import Empty

#topic to command
thrusters_topic="/g500/thrusters_input"
#base velocity for the teleoperation (0.5 m/s) / (0.5rad/s)
baseVelocity=1

#Console input variables to teleop it from the console
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

##create the publisher
pub = rospy.Publisher(thrusters_topic, Float64MultiArray,queue_size=1)
rospy.init_node('keyboardCommand')

##wait for benchmark init service
#rospy.wait_for_service('/startBench')
#start=rospy.ServiceProxy('/startBench', Empty)

##wait for benchmark stop service
#rospy.wait_for_service('/stopBench')
#stop=rospy.ServiceProxy('/stopBench', Empty)

#The try is necessary for the console input!
try:
    while not rospy.is_shutdown():
	msg = Float64MultiArray()
	msg.data = [0,0,0,0,0]
        try:
            c = sys.stdin.read(1)
            ##Depending on the character set the proper speeds
	    if c=='\n':
		start()
	  	print "Benchmarking Started!"
	    elif c==' ':
		stop()
		print "Benchmark finished!"
	    elif c=='w':
	    	msg.data[0] = -baseVelocity
	    	msg.data[1] = -baseVelocity
	    elif c=='s':
	    	msg.data[0] = baseVelocity
	    	msg.data[1] = baseVelocity
	    elif c=='a':
	    	msg.data[4] = -baseVelocity
	    elif c=='d':
	    	msg.data[4] = baseVelocity
	    elif c=='\x1b':  ##This means we are pressing an arrow!
		c2= sys.stdin.read(1)
		c2= sys.stdin.read(1)
	        if c2=='A':
			msg.data[2] = baseVelocity
			msg.data[3] = baseVelocity
	        elif c2=='B':
			msg.data[2] = -baseVelocity
			msg.data[3] = -baseVelocity
		elif c2=='C': #Flecha derecha
			msg.data[0] = -baseVelocity
	    		msg.data[1] = baseVelocity
		elif c2=='D': #Flecha izquierda
			msg.data[0] = baseVelocity
	    		msg.data[1] = -baseVelocity
	    else:
		print 'wrong key pressed'
	    while c!='':
	        c = sys.stdin.read(1)
        except IOError: pass

        ##publish the message
        pub.publish(msg)
	rospy.sleep(0.1)

##Other input stuff
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

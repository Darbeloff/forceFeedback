#!/usr/bin/env python
import rospy
import time
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import WrenchStamped

# define some global variables
def defineGlobalVars():
	global humForce
	humForce = 0
	global compForce
	compForce = 0
	global optoforceReading
	optoforceReading = 0


def humCallback(data):
	# max force
	maxHumForce = 150 # newtons

	# make deadband because joy is annoying
	joyForce = data.axes[1]
	if abs(data.axes[1]) < .1:
		joyForce = 0

	global humForce
	humForce = float(maxHumForce*joyForce)


def compCallback(data):
	# get and assign computer velocity to global
	global compForce
	compForce = float(data.data)


def optoforceCallback(data):
	# get and assign computer velocity to global
	global optoforceReading
	optoforceReading = -1*float(data.wrench.force.z)


def start():
	# initialize node
	rospy.init_node('combinedForceToArduino', anonymous = True)

	# init arduino commands pub
	global cmd2ArduinoPub
	cmd2ArduinoPub = rospy.Publisher('command2Arduino', Float32, queue_size = 10)
	global measurement2ArduinoPub
	measurement2ArduinoPub = rospy.Publisher('measurement2Arduino', Float32, queue_size = 10)

	# command subscribers
	humSubscriber = rospy.Subscriber("joy", Joy, humCallback)
	compSubscriber = rospy.Subscriber('compForceTraj', Float32, compCallback)

	# measurement subscribers
	optoforceSubscriber = rospy.Subscriber('/optoforce_0', WrenchStamped, optoforceCallback)

	# go to send command loop
	sendCommand()

	# rospy.spin()
	rospy.spin()


def sendCommand():
	# send send send
	readyCount = 0
	pubRate = 50 # hertz
	constantForceOffset = 10
	sendTraj = rospy.set_param("sendTraj",0)
	rate = rospy.Rate(pubRate) # hertz

	while (not rospy.is_shutdown()):
		# check whether we should be sending trajectory
		sendTraj = rospy.get_param("sendTraj",0)

		# get the most up to date readings
		measurement = optoforceReading

		# decide if we are sending the traj
		if sendTraj == 1:
			# publish that signal
			command = (compForce + humForce)/2.0
			print('sending')
			cmd2ArduinoPub.publish(command)
			measurement2ArduinoPub.publish(measurement)

		else:
			# send desired force
			cmd2ArduinoPub.publish(humForce)
			measurement2ArduinoPub.publish(measurement)

			if (optoforceReading > constantForceOffset*0.8) and (optoforceReading < constantForceOffset*1.2):
				readyCount = readyCount + 1
				print('counting up to send')
			else:
				readyCount = 0
				print('not in range')
			if readyCount > 100: # been at the right force for 2 seconds...start
				sendTraj = rospy.set_param("sendTraj",1)
				readyCount = 0

		# sleep to maintain loop rate
		rate.sleep()


if __name__ == '__main__':
    try:
	defineGlobalVars()
	start()
    except rospy.ROSInterruptException:
        pass

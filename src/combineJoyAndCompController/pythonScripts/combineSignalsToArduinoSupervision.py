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
	maxHumForce = 20 # newtons

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

	if compForce == -1.0: # send a buzz
		compForce = 5.0 # send a default instead

		# tell arduino to buzz
		buzz2ArduinoPub.publish(1.0)



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
	global buzz2ArduinoPub
	buzz2ArduinoPub = rospy.Publisher('buzz2Arduino', Float32, queue_size = 10)

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
	constantForceOffset = 0
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
			print('sending')
			cmd2ArduinoPub.publish(compForce)
			measurement2ArduinoPub.publish(measurement)

		else:
			# send desired force
			cmd2ArduinoPub.publish(0)
			measurement2ArduinoPub.publish(measurement)

			if (optoforceReading > constantForceOffset - 1) and (optoforceReading < constantForceOffset + 1):
				readyCount = readyCount + 1
				print('counting up to send')
			else:
				readyCount = 0
				print('not in range')
			if readyCount > 200: # been at the right force for 4 seconds...start
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

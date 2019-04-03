#!/usr/bin/env python
import rospy
import numpy as np
from scipy import interpolate
import math
import random
from std_msgs.msg import Float32

def start():
	# initialize node
	rospy.init_node('forceTraj', anonymous = True)

	# initialize computer force trajectory publisher
	global compForceTrajPub
	compForceTrajPub = rospy.Publisher('compForceTraj', Float32, queue_size = 10)

	# generate trajectory
	trajPubRate = 100.0; # Hz
	trajVec = genTraj(trajPubRate)

	# begin sending traj
	sendTraj(trajPubRate, trajVec)

	# rospy.spin()
	rospy.spin()

def genTraj(trajPubRate):
	maxForceD = 15 # newtwons per second
	minForce = -75
	maxForce = 75

	trialLength = 20 # seconds

	# first get time indicies
	# not sure i need this...timeVec = np.arange(0.0,trialLength,1/trajPubRate)

	# random seed
	random.seed()

	# now generate reference points for trajectory
	trajVecSparse = np.zeros(int(trialLength))
	trajVecSparse[0] = 0
	for i in range(1,trialLength): # go from 1 because at 0 its set at 90
		trajVecSparse[i] = random.random()*(maxForce - minForce) + minForce

		# ensure it doesn't exceed max velocity
		if (abs(trajVecSparse[i] - trajVecSparse[i-1]) > maxForceD):
			if trajVecSparse[i] > trajVecSparse[i-1]:
				trajVecSparse[i] = trajVecSparse[i-1] + maxForceD
			if trajVecSparse[i] < trajVecSparse[i-1]:
				trajVecSparse[i] = trajVecSparse[i-1] - maxForceD

	# now interpolate at trajPubRate
	interpFun = interpolate.interp1d(np.arange(0,len(trajVecSparse),1),trajVecSparse,kind="cubic")
	timeVec = np.arange(0.0,trialLength-1,1/trajPubRate)
	trajVec = interpFun(timeVec)
	return trajVec

def sendTraj(trajPubRate, trajVec):
	# ask lead arm to follow randomized trajectory
	loopNum = 0
	rate = rospy.Rate(trajPubRate) # hertz
	while (not rospy.is_shutdown()):
		# check whether we should be sending trajectory
		sendTraj = rospy.get_param("sendTraj",0)
		if (sendTraj == 1) and (loopNum < len(trajVec)):
			# do stuff
			# print(trajVec[loopNum])
			compForceTrajPub.publish(trajVec[loopNum])
			loopNum = loopNum + 1
		else:
			# send 0
			compForceTrajPub.publish(0)

		# sleep to maintain loop rate
		rate.sleep()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass

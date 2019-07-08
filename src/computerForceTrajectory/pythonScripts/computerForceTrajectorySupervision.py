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

	trialLength = int(rospy.get_param("trialTime", 60.0)) # seconds # seconds
	trial_version = rospy.get_param("trial_version",0) # 0 is only start/end, 1 is start/end/up, 2 is start/end/up/buzz

	# random seed
	random.seed()

	# pick start time (feature 1)...within first 5->15 seconds
	start_time = int(random.random()*(15-5) + 5)
	end_time = int(np.array(random.sample(range(-5,5),1))) + 50 # 30 +/- 5 second trial but takes two seconds to get back down to 0

	# pick desired force
	desired_force = 8

	# now generate reference points for just start and stop of trajectory
	trajVecSparse = np.zeros(int(trialLength))
	trajVecSparse[0] = 0
	for i in range(1,trialLength): # go from 1 because at 0 its set at 0
		if (i == start_time):
			trajVecSparse[i] = 0
		elif (i == start_time + 1):
			trajVecSparse[i] = 2.5
		elif (i == start_time + 2):
			trajVecSparse[i] = 5
		elif (i == end_time):
			trajVecSparse[i] = 5
		elif (i == end_time + 1):
			trajVecSparse[i] = 2.5
		elif (i == end_time + 2):
			trajVecSparse[i] = 0
		elif (i > start_time and i < end_time):
			trajVecSparse[i] = 5
		else:
			trajVecSparse[i] = 0

	# now add in "failures"...characterized by a sustained (3 seconds) offset
	max_fails = 4
	min_fails = 2
	num_failures = int(random.random()*(max_fails - min_fails) + min_fails)

	if trial_version == 0:
		num_failures = 0 # this ensures only start/end

	# choose failure start time, end times, and direction...then insert into trajVecSparse
	buzz_start_time = []
	fail_direction_prev = -999
	fail_direction_prev_prev = -999
	srl_action_range = range(start_time, end_time)
	srl_action_range_failures = srl_action_range[0::6]
	srl_action_range_failures = srl_action_range_failures[1:-1]
	fail_start_times = random.sample(srl_action_range_failures, num_failures)
	for i in range(0,num_failures):
		# get end time and direction
		fail_end_time = 3 + fail_start_times[i]

		# if only start/end/up
		if trial_version == 1:
			fail_direction = 1 # always up

		# if only start/end/up/buzz
		if trial_version == 2:
			fail_direction = random.sample([0, 1], 1)
			fail_direction = fail_direction[0]

			# logic to ensure we get at least one up and at least one buzz
			if i == 2 and fail_direction_prev == fail_direction_prev_prev and fail_direction_prev == 1:
				fail_direction = 0
			if i == 2 and fail_direction_prev == fail_direction_prev_prev and fail_direction_prev == 0:
				fail_direction = 1
			if i == 1 and num_failures == 2 and fail_direction_prev == 1:
				fail_direction = 0
			if i == 1 and num_failures == 2 and fail_direction_prev == 0:
				fail_direction = 1

			# logic to catch the buzzes and log them
			if fail_direction == 0:
				buzz_start_time.append(fail_start_times[i])

		# assign previous direction logic
		fail_direction_prev_prev = fail_direction_prev
		fail_direction_prev = fail_direction

		# insert into sparse trajectory
		for j in range(fail_start_times[i],fail_end_time):
			trajVecSparse[j] = trajVecSparse[j] + 4*np.array(fail_direction)

	print(trajVecSparse)

	# now interpolate at trajPubRate
	interpFun = interpolate.interp1d(np.arange(0,len(trajVecSparse),1),trajVecSparse,kind="cubic")
	timeVec = np.arange(0.0,trialLength-1,1/trajPubRate)
	trajVec = interpFun(timeVec)

	# now add in buzz signal...janky!!!
	if trial_version == 2 and len(buzz_start_time) > 0:
		for i in range(0,len(buzz_start_time)):
			print(buzz_start_time[i])
			trajVec[int(buzz_start_time[i]/60.0*5900)] = -1.0 # converting to the interpolated traj

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

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
import random
import time
import Tkinter as tk
import string

def defineGlobalVars():
	global color
	color = 0
	global prevButtonState # tracks previous state of button
	prevButtonState = 0 # starts off
	global buttonChanged # tracks if button has changed state
	buttonChanged = 0


def joyCallback(data):
	global color
	global prevButtonState
	global buttonChanged

	buttonState = data.buttons[0]

	# this tells us that the button state has changed, allow the program to send a new 1 or 2
	if prevButtonState != buttonState:
		buttonChanged = 1

	# if the button is a 1 at the same time as a white letter is up, say the subject got it
	if buttonState == 1 and color == 'white' and buttonChanged == 1:
		visDistPub.publish(1)
		buttonChanged = 0

	# 2 means pressed button but no white
	if buttonState == 1 and color != 'white' and buttonChanged == 1:
		visDistPub.publish(2)
		buttonChanged = 0

	prevButtonState = buttonState


def main():
	# initialize node
	rospy.init_node('tkinterGUI', anonymous = True, disable_signals=True)

	# subscribe to joy, our human confirmation source
	humSubscriber = rospy.Subscriber("joy", Joy, joyCallback)

	# initialize visual distractor success publisher
	global visDistPub
	visDistPub = rospy.Publisher('visDistPub', Float32, queue_size = 10)

	# init global
	defineGlobalVars()

	# get the gui going
	makeGUI()

	# rospy spin
	rospy.spin()

def makeGUI():
	# tkinter setup
	root = tk.Tk()
	global label
	label = tk.Label(root, text="lets go!", font=("Helvetica", 150))
	label.pack()

	# how long is the letter up?
	#letterTime = .400 # 100 ms
	letterTime = rospy.get_param("letterTime", 0.400)
	trialTime = rospy.get_param("trialTime", 20.0) # seconds
	#whitePercentage = 30.0 # 30 percent of letters are white
	whitePercentage = rospy.get_param("letterPercentage", 30.0)

	# rospy rate given letterTime
	rate = rospy.Rate(float(1/letterTime)) # 1/seconds = freq in hertz

	alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'

	random.seed()

	global color
	global buttonPressed
	numWhite = 0
	first_time = True
	prev_letter_choice = '1'
	letterChoice = '1'

	printString("Starting soon", "red")

	while (not rospy.is_shutdown()):
		# check whether we should be sending trajectory
		sendTraj = rospy.get_param("sendTraj",0)

		# decide whether to start experiment
		if sendTraj == 1:
			if first_time:
				currTime = time.time()
				startTime = currTime
				endTime = startTime+trialTime
				first_time = False
			else:
				# get letter
				while letterChoice == prev_letter_choice:
					letterChoice = random.choice(alphabet)

				# get color (1 = white, 0 = yellow)
				colorChoice = random.random()
				if colorChoice >= whitePercentage/100.0:
					color = "yellow"
				else:
					color = "white"
					numWhite = numWhite + 1
					visDistPub.publish(0)

				# print the letter and color
				printString(letterChoice,color)

				# update currTime
				currTime = time.time()

				# update prev letter
				prev_letter_choice = letterChoice

				if currTime > endTime:
					break

		# sleep the appropriate amount
		rate.sleep()

	# trial is over
	printString("We done!", "red")
	visDistPub.publish(numWhite)
	visDistPub.publish(-999)

	# update root
	root.mainloop()


def printString(letter, color):
	# insert the new letter
	label.config(text=letter,fg=color,height=50,width=50)
	label.update()

	# gap between letters
	time.sleep(.02) # 20 ms


main()

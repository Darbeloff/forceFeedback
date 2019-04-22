#!/usr/bin/env python
import random
import time
import tkinter as tk
import string


def main():
	# get the gui going
	makeGUI()

def makeGUI():
	# tkinter setup
	root = tk.Tk()
	global label
	label = tk.Label(root, text="lets go!", font=("Helvetica", 150))
	label.pack()

	# how long is the letter up? 
	letterTime = .1 # 100 ms
	trialTime = 30 # seconds

	# get the timers setup
	currTime = time.time()
	startTime = currTime
	endTime = startTime+trialTime 
	
	while currTime < endTime:
		# get letter
		alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ'
		letterChoice = random.choice(alphabet)

		# get color (1 = white, 0 = yellow)
		whitePercentage = 10 # 10 percent of letters are white

		colorChoice = random.random()
		if colorChoice >= whitePercentage/100:
			color = "yellow"
		else:
			color = "white"

		# print the letter and color
		printString(letterChoice,color)

		# sleep the appropriate amount
		time.sleep(letterTime)

	# update root
	root.mainloop()
	

def printString(letter, color):	
	# insert the new letter
	label.config(text=letter,fg=color,height=50,width=50)
	label.update()

	# gap between letters
	time.sleep(.02) # 20 ms


main()

colorChoice = random.random()
if colorChoice >= whitePercentage/100:
	color = "white"
else:
	color = "yellow"
color




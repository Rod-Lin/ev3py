#! /usr/bin/python3

import motor
import sensor
import math
import time
import os

motors = motor.initList(motor.getList())
sensors = sensor.initList(sensor.getList())

print (motors)
print (sensors)

"""
motor.runSingleRelat(motors[1], 50, 100)
motor.runDoubleRelat(motors[1], motors[2], 50, 200, 50, 200)

print (sensor.val(sensors[0][0]))
print (sensor.val(sensors[0][1]))
print (sensor.val(sensors[0][2]))
"""

print (sensor.val(sensors[0][1]))
print (sensor.val(sensors[0][2]))

# m1/2: motor 1/2
# sen1/2: sensor 1/2
# speed: ideal speed
# tc1/2: target color 1/2

# black1/2: reflection for black(of sensor 1/2), used in cross line process
# white1/2: reflection for white(of sensor 1/2), used in break line process

# kp, ki, kd: coefficients for PID

config = {
	"black1": 14,
	"black2": 14,
	"white1": 30,
	"white2": 30,
	"white_count_thres": 70,
	"more_output1_thres": 260,
	"state_changing_thres": 25
}

pos1_begin = 0
pos1_end = 0

pos2_begin = 0
pos2_end = 0

white_count = 0

def check_break_line(m1, m2, sen1, sen2, val1, val2, speed, state):
	global white_count
	global pos1_begin
	global pos1_end
	global pos2_begin
	global pos2_end

	if (val1 >= config["white1"] and val2 >= config["white2"]):
		white_count += 1
		if (white_count > config["white_count_thres"]):
			print("!!! too much white")
			pos1_end = motor.getPos(m1)
			pos2_end = motor.getPos(m2)

			# print("1 begin: %s; end: %s; 2 begin: %s; end: %s " % (pos1_begin, pos1_end, pos2_begin, pos2_end))

			motor.runDoubleRelat(m1, m2,
								 80, pos1_begin - pos1_end + 50,
								 80, pos2_begin - pos2_end + 50,
								 "hold")
			motor.waitForDoubleHold(m1, m2)

			"""
			# motor.runDoubleRelat(m1, m2, 35, 1000, 35, 1000, "hold")
			motor.runDoubleDirect(m1, m2, speed, speed)
			while 1:
				val1 = sensor.val(sen1)
				val2 = sensor.val(sen2)
				if (val1 <= white1 or val2 <= white2):
					break
			"""
			while 1:
				try_times = 0
				while try_times < 4:
					try_times += 1
					motor.runDoubleRelat(m1, m2, 80, 140, 80, 140)
					motor.waitForStop(m1)
					pos1_begin = motor.getPos(m1)
					pos2_begin = motor.getPos(m2)
					motor.runDoubleRelat(m1, m2, 80, 500, 80, -500)
					motor.waitForStop(m1)
					motor.runDoubleRelat(m1, m2, 70, -800, 70, 800)

					while not (motor.hasStopped(m1) or motor.hasStopped(m2)):
						val1 = sensor.val(sen1)
						if (val1 <= config["black1"]
							or val2 <= config["black2"]):
							motor.stop(m1);
							motor.stop(m2);
							print("restore")
							break
					
					if (val1 <= config["black1"]
						or val2 <= config["black2"]): break
					
					pos1_end = motor.getPos(m1)
					pos2_end = motor.getPos(m2)
					motor.runDoubleRelat(m1, m2,
										 80, 330,
										 80, -330)
					motor.waitForDoubleStop(m1, m2)

				if (try_times > 4):
					print("HOLY SH--!!")
					motor.runDoubleRelat(m1, m2, 60, -(try_times + 3) * 140, 60, -(try_times + 3) * 140)
					motor.waitForStop(m1)
					print("let's change a direction")
					motor.runDoubleRelat(m1, m2, 60, -100, 60, 100)
					motor.waitForStop(m1)
				else:
					break
			# end here

			motor.runDoubleDirect(m1, m2, speed, speed)
			white_count = 0
			return 1
	else:
		pos1_begin = motor.getPos(m1)
		pos2_begin = motor.getPos(m2)
		white_count = 0
	
	return 0

def check_double_black(m1, m2, sen1, sen2, val1, val2, speed, state):
	if (val1 <= config["black1"]
		and val2 <= config["black2"]):

		motor.runDoubleRelat(m1, m2, 80, 200, 80, -100)
		motor.waitForDoubleStop(m1, m2)
		if (sensor.val(sen2) > config["black2"]): # no line on the right
			motor.runDoubleRelat(m1, m2, 80, -220, 80, 80)
			motor.waitForDoubleStop(m1, m2)
		else:
			motor.runDoubleRelat(m1, m2, 80, -200, 80, 100)
			print("double black")
			motor.runDoubleRelat(m1, m2, 80, 100, 80, 100)
			motor.waitForDoubleStop(m1, m2)
			motor.runDoubleRelat(m1, m2, 80, 450, 80, -450)
			motor.waitForDoubleStop(m1, m2)
			motor.runDoubleDirect(m1, m2, -70, 70)
			while 1:
				if (sensor.val(sen1) <= config["black1"]
					or sensor.val(sen2) <= config["black2"]):
					break
			motor.runDoubleDirect(m1, m2, speed, speed)
			return 1
		motor.runDoubleDirect(m1, m2, speed, speed)

	return 0

def check_touch(m1, m2, sen1, sen2, val1, val2, speed, state):
	touch_val = sensor.val(sensors[1])

	if (touch_val):
		motor.stop(m1)
		motor.stop(m2)
		exit()
		motor.runDoubleRelat(m1, m2, 80, -200, 80, -200)
		motor.waitForDoubleStop(m1, m2)
		motor.runDoubleRelat(m1, m2, 80, -300, 80, 300)
		motor.waitForDoubleStop(m1, m2)
		motor.runDoubleDirect(m1, m2, 60, 20)
		# motor.waitForDoubleStop(m1, m2)
		while 1:
			val1 = sensor.val(sen1)
			if (val1 <= config["black1"]):
				motor.runDoubleRelat(m1, m2, 80, 100, 80, 100)
				motor.waitForDoubleStop(m1, m2)
				break
		motor.runDoubleDirect(m1, m2, speed, speed)
		return 1

	return 0

def inter(m1, m2, sen1, sen2, val1, val2, speed, state):
	if (check_touch(m1, m2, sen1, sen2, val1, val2, speed, state)): return

	if (check_break_line(m1, m2, sen1, sen2, val1, val2, speed, state)): return

	if (check_double_black(m1, m2, sen1, sen2, val1, val2, speed, state)): return

	return

def pid(m1, m2, sen1, sen2, speed,
		tc1_1, tc1_2,
		tc2_1, tc2_2,
		kp, ki, kd, inter_act):
	intg1 = 0
	derv1 = 0
	last_err1 = 0

	intg2 = 0
	derv2 = 0
	last_err2 = 0

	tc1 = tc1_1
	tc2 = tc2_1

	state = 1

	kp_o = kp
	ki_o = ki
	kd_o = kd

	motor.runDoubleDirect(m1, m2, speed, speed)

	for i in range(0, 5000):
		val1 = sensor.val(sen1)
		val2 = sensor.val(sen2)

		inter_act(m1, m2, sen1, sen2, val1, val2, speed, state)

		if (val1 <= config["state_changing_thres"]):
			# output1 = -70
			# output2 = 70
			# if (output1 > 0):
			#	print(output1)
			# print("more output1: " + str(output1))
			intg1 = 0
			intg2 = 0
			tc1 = tc1_2
			tc2 = tc2_2
			state = 2
			# motor.stop(m1)
			# motor.stop(m2)
			print(val1)
			# exit()
		else:
			tc1 = tc1_1
			tc2 = tc2_1
			state = 1

		error1 = val1 - tc1
		intg1 += error1
		intg1 *= 1 - ki
		derv1 = error1 - last_err1
		output1 = kp * error1 + ki * intg1 + kd * derv1

		error2 = val2 - tc2
		intg2 += error2
		intg2 *= 1 - ki
		derv2 = error2 - last_err2
		output2 = kp * error2 + ki * intg2 + kd * derv2

		s1 = speed + output1
		s1 = speed - output2

		s2 = speed - output1
		s2 = speed + output2

		if (s1 > 100):
			s1 = 100
		elif (s1 < -100):
			s1 = -100

		if (s2 > 100):
			s2 = 100
		elif (s2 < -100):
			s2 = -100

		motor.setDoubleSpeed(m1, m2, int(s1), int(s2))
		last_err1 = error1
		last_err2 = error2
	motor.stop(m1)
	motor.stop(m2)

# motor.runDoubleRelat(motors[1], motors[2], 60, 10000, 60, 10000)
# motor.waitForDoubleStop(motors[1], motors[2])
# exit()

motor.stop(motors[1])
motor.stop(motors[2])

motor.reset(motors[1])
motor.reset(motors[2])

# motor.setPolarity(motors[1], "inversed")
# motor.setPolarity(motors[2], "inversed")

# pid(motors[1], motors[2], sensors[0][1], 30, 13, 5, 0.001, 40)

def speak(text):
	os.system("espeak \"%s\" --stdout | aplay" % text)

speak("state one")
time.sleep(2)
speak("bee")

val1_1 = sensor.val(sensors[0][1])
val2_1 = sensor.val(sensors[0][2])

print("val1_1: %s, val2_1: %s" % (val1_1, val2_1))

speak("state two")
time.sleep(2)
speak("bee")

val1_2 = sensor.val(sensors[0][1])
val2_2 = sensor.val(sensors[0][2])

print("val1_2: %s, val2_2: %s" % (val1_2, val2_2))

# motor.runDoubleDirect(motors[1], motors[2], -60, -60)
# motor.waitForDoubleStop(motors[1], motors[2])

# motor.setMK(0.6)
# motor.setM2K(1.1)

# MK: 0.9
# > 8v: s 40, p 6, i 0.004, d 30

# MK: 0.8
# > 7.7v: s 40, p 8, i 0.003, d 30

# MK: 1
# 7.28v: s 40, p 6.5, i 0.004, d 50

# MK: 1
# 7.4v - 7.2v: s 40, p 7, i 0.003, d 30

time.sleep(2)
speak("start")

pid(motors[1], motors[2], sensors[0][1], sensors[0][2], 50, val1_1, val1_2, val2_1, val2_2, 7, 0.006, 40, inter)
# pid2(motors[1], motors[2], sensors[0][1], sensors[0][2], sensors[1], 45, val1, val2, 20, 20, 22, 32, 1.5, 0.04, 70)

"""
motor.runDoubleDirect(motors[1], motors[2], 40, 40)
while 1:
	if (sensor.val(sensors[0][1]) == 3):
		break

motor.stop(motors[1])
motor.stop(motors[2])
"""

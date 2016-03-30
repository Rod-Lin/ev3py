#! /usr/bin/python3

import motor
import sensor
import math
import time

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

def pid(m1, m2, sen1, speed, black, kp, ki, kd):
	intg = 0
	derv = 0
	last_err = 0
	motor.runDoubleDirect(m1, m2, speed, speed)
	for i in range(0, 500):
		val = sensor.val(sen1)
		error = val - black
		intg += error
		derv = error - last_err
		output = kp * error + ki * intg + kd * derv

		s1 = speed + output
		s2 = speed - output

		if (s1 > 100):
			s1 = 100
		elif (s1 < -100):
			s1 = -100

		if (s2 > 100):
			s2 = 100
		elif (s2 < -100):
			s2 = -100

		print(str(int(s1)) + ", " + str(int(s2)))
		motor.setDoubleSpeed(m1, m2, int(s1), int(s2))
		last_err = error
	motor.stop(motors[1])
	motor.stop(motors[2])

def pid2(m1, m2, sen1, sen2, speed, black1, black2, white1, white2, kp, ki, kd):
	intg1 = 0
	derv1 = 0
	last_err1 = 0

	intg2 = 0
	derv2 = 0
	last_err2 = 0

	pos1_begin = 0
	pos1_end = 0

	pos2_begin = 0
	pos2_end = 0

	white_count = 0

	motor.runDoubleDirect(m1, m2, speed, speed)
	for i in range(0, 1000):
		val1 = sensor.val(sen1)
		val2 = sensor.val(sen2)
		
		if (val1 >= white1 and val2 >= white2):
			white_count += 1
			if (white_count > 60):
				print("!!! too much white")
				pos1_end = motor.getPos(m1)
				pos2_end = motor.getPos(m2)

				# print("1 begin: %s; end: %s; 2 begin: %s; end: %s " % (pos1_begin, pos1_end, pos2_begin, pos2_end))

				motor.runDoubleRelat(m1, m2,
									 35, pos1_begin - pos1_end + 50,
									 35, pos2_begin - pos2_end + 50,
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
					motor.runDoubleRelat(m1, m2, 60, 100, 60, 100)
					motor.waitForStop(m1)
					pos1_begin = motor.getPos(m1)
					pos2_begin = motor.getPos(m2)
					motor.runDoubleRelat(m1, m2, 60, 500, 60, -500)
					motor.waitForStop(m1)
					count = 0
					while count < 12:
						val1 = sensor.val(sen1)
						print(str(val2))
						if (val1 <= white1):
							print("restore")
							break
						motor.runDoubleRelat(m1, m2, 60, -80, 60, 80)
						motor.waitForStop(m2)
						count += 1
					if (val1 <= white1):
						break
					else:
						pos1_end = motor.getPos(m1)
						pos2_end = motor.getPos(m2)
						motor.runDoubleRelat(m1, m2,
											 50, pos1_begin - pos1_end + 50,
											 50, pos2_begin - pos2_end + 50,
											 "hold")
						motor.waitForDoubleHold(m1, m2)

				motor.runDoubleDirect(m1, m2, speed, speed)
				white_count = 0
				continue
		else:
			pos1_begin = motor.getPos(m1)
			pos2_begin = motor.getPos(m2)
			white_count = 0

		# print("val1: %s; val2: %s; white count: %s" % (val1, val2, white_count))
		

		error1 = val1 - black1
		intg1 += error1
		intg1 *= 1 - ki
		derv1 = error1 - last_err1
		output1 = kp * error1 + ki * intg1 + kd * derv1

		error2 = val2 - black2
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

		# print(str(int(s1)) + ", " + str(int(s2)))
		motor.setDoubleSpeed(m1, m2, int(s1), int(s2))
		last_err1 = error1
		last_err2 = error2
	motor.stop(motors[1])
	motor.stop(motors[2])

# motor.stop(motors[1])
# motor.stop(motors[2])

motor.reset(motors[1])
motor.reset(motors[2])

motor.setPolarity(motors[1], "inversed")
motor.setPolarity(motors[2], "inversed")

# pid(motors[1], motors[2], sensors[0][1], 30, 13, 5, 0.001, 40)

val1 = sensor.val(sensors[0][1])
val2 = sensor.val(sensors[0][2]) - 8

print("val1: %s, val2: %s" % (val1, val2))

pid2(motors[1], motors[2], sensors[0][1], sensors[0][2], 40, val1, val2, 20, 30, 5, 0.001, 40)

# motor.runDoubleRelat(motors[1], motors[2], 60, 50000, 60, 50000)

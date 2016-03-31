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

# m1/2: motor 1/2
# sen1/2: sensor 1/2
# speed: ideal speed
# tc1/2: target color 1/2

# black1/2: reflection for black(of sensor 1/2), used in cross line process
# white1/2: reflection for white(of sensor 1/2), used in break line process

# kp, ki, kd: coefficients for PID

def pid2(m1, m2, sen1, sen2, speed, tc1, tc2, black1, black2, white1, white2, kp, ki, kd):
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
	origin_speed = speed
	origin_kp = kp
	origin_ki = ki
	origin_kd = kd

	turn_count = 0
	keep_turn = 0
	is_in_dec = 0

	dec_threshold = 55
	origin_dec_threshold = dec_threshold

	motor.runDoubleDirect(m1, m2, speed, speed)
	for i in range(0, 2000):
		val1 = sensor.val(sen1)
		val2 = sensor.val(sen2)

		"""
		if (val1 <= black1 and val2 <= black2):
			motor.runDoubleRelat(m1, m2, 50, 150, 50, 150)
			motor.waitForStop(m1)
			motor.runDoubleRelat(m1, m2, 50, 360, 50, -360)
			motor.waitForStop(m1)
			motor.runDoubleDirect(m1, m2, speed, speed)
		"""
		
		if (val1 >= white1 and val2 >= white2):
			white_count += 1
			if ((not is_in_dec and white_count > 70) or (is_in_dec and white_count > 100)):
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
					try_times = 0
					while try_times < 3:
						try_times += 1
						motor.runDoubleRelat(m1, m2, 60, 100, 60, 100)
						motor.waitForStop(m1)
						pos1_begin = motor.getPos(m1)
						pos2_begin = motor.getPos(m2)
						motor.runDoubleRelat(m1, m2, 60, 500, 60, -500)
						motor.waitForStop(m1)
						count = 0
						while count < 20:
							val1 = sensor.val(sen1)
							# print(str(val2))
							if (val1 <= white1):
								print("restore")
								break
							motor.runDoubleRelat(m1, m2, 60, -40, 60, 40)
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

					if (try_times >= 3):
						print("HOLY SH--!!")
						motor.runDoubleRelat(m1, m2, 60, (-try_times - 2) * 100, 60, (-try_times - 2) * 100)
						motor.waitForStop(m1)
						print("let's change a direction")
						motor.runDoubleRelat(m1, m2, 60, -100, 60, 100)
						motor.waitForStop(m1)
					else:
						break
				# end here

				motor.runDoubleDirect(m1, m2, speed, speed)
				white_count = 0
				continue
		else:
			pos1_begin = motor.getPos(m1)
			pos2_begin = motor.getPos(m2)
			white_count = 0

		# print("val1: %s; val2: %s; white count: %s" % (val1, val2, white_count))
		

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

		# print(abs(s1 - s2))
		
		if (abs(s1 - s2) > dec_threshold):
			turn_count += 1
			# print("turn count: " + str(turn_count))
			if (turn_count >= 4 or (is_in_dec and turn_count >= 3)):
				print("deceleration start")
				if (not is_in_dec):
					is_in_dec = 1
					speed = 20
					kp = 4.2
					ki = 0.001
					kd = 40
					dec_threshold = 185
					keep_turn = 80
				else:
					# kp = 3.8
					keep_turn += 20
				
				# val2 -= 5
				turn_count = 0
				
		else:
			turn_count = 0
			if (is_in_dec):
				if (keep_turn == 1):
					keep_turn = 0
					print("restore speed")
					is_in_dec = 0
					speed = origin_speed
					kp = origin_kp
					ki = origin_ki
					kd = origin_kd
					intg1 = 0
					intg2 = 0

					dec_threshold = origin_dec_threshold
					# val2 += 5
				else:
					keep_turn -= 1

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
val2 = sensor.val(sensors[0][2]) - 10

print("val1: %s, val2: %s" % (val1, val2))

pid2(motors[1], motors[2], sensors[0][1], sensors[0][2], 40, val1, val2, 20, 20, 22, 32, 1.5, 0.04, 70)

# motor.runDoubleRelat(motors[1], motors[2], 60, 50000, 60, 50000)

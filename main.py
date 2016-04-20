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
	"state_changing_thres": 25,
	"stucked_thres": 150
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

			motor.runDoubleRelat(m1, m2, 70, -200, 70, 200)
			found_back = 0
			while not (motor.hasStopped(m1) or motor.hasStopped(m2)):
				if (val1 <= config["black1"]
					or val2 <= config["black2"]):
					motor.stop(m1);
					motor.stop(m2);
					found_back = 1
					print("restore")
					break
			if not found_back:
				motor.runDoubleRelat(m1, m2, 70, 400, 70, -400)
				while not (motor.hasStopped(m1) or motor.hasStopped(m2)):
					if (val1 <= config["black1"]
						or val2 <= config["black2"]):
						motor.stop(m1);
						motor.stop(m2);
						found_back = 1
						print("restore")
						break
				if not found_back:
					motor.runDoubleRelat(m1, m2, 80, -200, 80, 200)
					motor.waitForDoubleStop(m1, m2)
					motor.runDoubleRelat(m1, m2,
										 80, pos1_begin - pos1_end,
										 80, pos2_begin - pos2_end,
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
					if_out = 0
					while 1:
						try_times = 0
						while try_times < 5:
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
								or val2 <= config["black2"]):
								if_out = 1
								break
							
							pos1_end = motor.getPos(m1)
							pos2_end = motor.getPos(m2)
							motor.runDoubleRelat(m1, m2,
												 80, 330,
												 80, -330)
							motor.waitForDoubleStop(m1, m2)

						if (try_times >= 5 and not if_out):
							print("HOLY SH--!!")
							motor.runDoubleRelat(m1, m2, 60, -(try_times + 1) * 140, 60, -(try_times + 1) * 140)
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
			# motor.runDoubleDirect(m1, m2, speed, speed)
		motor.runDoubleDirect(m1, m2, speed, speed)
		return 1

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
	if (check_touch(m1, m2, sen1, sen2, val1, val2, speed, state)): return 1

	if (check_break_line(m1, m2, sen1, sen2, val1, val2, speed, state)): return 1

	if (check_double_black(m1, m2, sen1, sen2, val1, val2, speed, state)): return 1

	return 0

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

	time_begin = time.time()
	time_end = 0
	pos1_begin = motor.getPos(m1)
	pos2_begin = motor.getPos(m2)
	pos1_end = 0
	pos2_end = 0

	speed_o = speed

	stuck_c = 0
	unstuck_c = 0

	motor.runDoubleDirect(m1, m2, speed, speed)

	for i in range(0, 5000):
		val1 = sensor.val(sen1)
		val2 = sensor.val(sen2)

		inter_act(m1, m2, sen1, sen2, val1, val2, speed, state)
		"""
		if not inter_act(m1, m2, sen1, sen2, val1, val2, speed, state):
			# compute real speed
			time_end = time.time()
			if (time_end - time_begin >= 0.5):
				pos1_end = motor.getPos(m1)
				pos2_end = motor.getPos(m2)
				rs1 = int((pos1_end - pos1_begin) / (time_end - time_begin))
				rs2 = int((pos2_end - pos2_begin) / (time_end - time_begin))
				print("%s %s" % (rs1, rs2))
				if (abs(rs1) <= config["stucked_thres"]
					and abs(rs2) <= config["stucked_thres"]):
					print("stucked!!")
					stuck_c += 1
					unstuck_c = 0
					if (stuck_c >= 3):
						motor.runDoubleRelat(m1, m2, 80, 700, 80, 700)
						motor.waitForDoubleStop(m1, m2)
						motor.runDoubleDirect(m1, m2, speed, speed)
				else:
					unstuck_c += 1
					if (unstuck_c >= 4):
						stuck_c = 0
				# else:
					# speed = speed_o
				# time_end = time.time()
				# print("%s, %s" % (int((pos1_end - pos1_begin) / (time_end - time_begin)),
				#				  int((pos2_end - pos2_begin) / (time_end - time_begin))));
				pos1_begin = pos1_end
				pos2_begin = pos2_end
				time_begin = time_end
		else:
			time_begin = time.time()
			pos1_begin = motor.getPos(m1)
			pos2_begin = motor.getPos(m2)
		"""

		if (val1 <= config["state_changing_thres"]):
			# output1 = -70
			# output2 = 70
			# if (output1 > 0):
			#	print(output1)
			# print("more output1: " + str(output1))
			# intg1 = 0
			# intg2 = 0
			tc1 = tc1_2
			tc2 = tc2_2
			state = 2
			# motor.stop(m1)
			# motor.stop(m2)
			# print(val1)
			# exit()
		else:
			tc1 = tc1_1
			tc2 = tc2_1
			speed = speed_o
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
# pid2(motors[1], motors[2], sensors[0][1], sensors[0][2], sensors[1], 45, val1, val2, 20, 20, 22, 32, 1.5, 0.04, 70)

############################
"""
motor.runDoubleDirect(motors[1], motors[2], 40, 40)
while 1:
	if (sensor.val(sensors[0][1]) == 3):
		break

motor.stop(motors[1])
motor.stop(motors[2])
"""

# Mr. H

def isBlack(val):
	#if (val <= config["black" + str(i)]):
	#	return 1
	#return 0
	return val == 1 or val == 7

def isWhite(val):
	#if (val >= config["white" + str(i)]):
	#	return 1
	#return 0
	return val == 6

def isStucked(m1, m2, pos1, pos2, time_begin):
	time_end = time.time()
	if time_end - time_begin > 0.2:
		end1 = motor.getPos(m1)
		end2 = motor.getPos(m2)

		rs1 = abs(int((end1 - pos1) / (time_end - time_begin)))
		rs2 = abs(int((end2 - pos2) / (time_end - time_begin)))

		print(rs1, " ", rs2)
		if rs1 < 50 and rs2 < 50:
			print("stucked! ", rs1, " ", rs2)
			motor.runDoubleRelat(m1, m2, 80, 700, 80, 700)
			motor.waitForDoubleStop(m1, m2)

		return 1
	return 0
	# if (rs1 >= 120 and rs2 >= )

def line(m1, m2, sen1, sen2, speed):

	pos1_begin = motor.getPos(m1)
	pos2_begin = motor.getPos(m2)
	pos1_end = 0
	pos2_end = 0

	is_b1 = 0
	is_b2 = 0

	scan_s = 100

	last_dir = 0

	rs_begin1 = motor.getPos(m1)
	rs_begin2 = motor.getPos(m2)
	time_begin = time.time()

	motor.runDoubleDirect(m1, m2, speed, speed)

	for i in range(0, 5000):
		val1 = sensor.val(sen1)
		val2 = sensor.val(sen2)
		touch1 = sensor.val(sensors[1])

		is_b1 = isBlack(val1)
		is_b2 = isBlack(val2)

		pos1_end = motor.getPos(m1)
		pos2_end = motor.getPos(m2)

		if (touch1):
			break
		"""
		if (isStucked(m1, m2, rs_begin1, rs_begin2, time_begin)):
			rs_begin1 = motor.getPos(m1)
			rs_begin2 = motor.getPos(m2)
			time_begin = time.time()
		"""
		# print(pos1_end - pos1_begin)

		if (is_b1 and is_b2):
			motor.runDoubleRelat(m1, m2, 80, 100, 80, 100)
			motor.waitForDoubleStop(m1, m2)
			motor.runDoubleRelat(m1, m2, 80, 450, 80, -450)
			motor.waitForDoubleStop(m1, m2)
			motor.runDoubleDirect(m1, m2, -scan_s, scan_s)
			while 1:
				if (isBlack(sensor.val(sen1))
					or isBlack(sensor.val(sen2))):
					break
			motor.runDoubleDirect(m1, m2, speed, speed)
			pos1_begin = motor.getPos(m1)
			pos2_begin = motor.getPos(m2)
		elif is_b1:
			# motor.runDoubleDirect(m1, m2, -scan_s, scan_s)
			motor.setDoubleSpeed(m1, m2, -scan_s, scan_s)
			#while not (isWhite(sensor.val(sen1))
			#		   or isBlack(sensor.val(sen2))):
			#	pass
			pos1_begin = motor.getPos(m1)
			pos2_begin = motor.getPos(m2)
			last_dir = 1
		elif is_b2:
			# motor.runDoubleDirect(m1, m2, scan_s, -scan_s)
			motor.setDoubleSpeed(m1, m2, scan_s, -scan_s)
			#while not (isBlack(sensor.val(sen1))
			#		   or isWhite(sensor.val(sen2))):
			#	pass
			pos1_begin = motor.getPos(m1)
			pos2_begin = motor.getPos(m2)
			last_dir = -1
		elif (pos1_end - pos1_begin) > 400:
			motor.stop(m1);
			motor.stop(m2);
			while 1:
				motor.runDoubleRelat(m1, m2, 80, 300, 80, -300)
				while not motor.hasStopped(m1):
					if isBlack(sensor.val(sen1)) or isBlack(sensor.val(sen2)):
						motor.stop(m1)
						motor.stop(m2)
						break
				else:
					motor.runDoubleRelat(m1, m2, 80, -600 - last_dir * 20,
												 80, 600 + last_dir * 20)
					while not motor.hasStopped(m2):
						if isBlack(sensor.val(sen1)) or isBlack(sensor.val(sen2)):
							motor.stop(m1)
							motor.stop(m2)
							break
					else:
						motor.runDoubleRelat(m1, m2, 80, 300, 80, -300)
						motor.waitForDoubleStop(m1, m2)
						motor.runDoubleRelat(m1, m2, 80, 300, 80, 300)
						motor.waitForStop(m1)
						continue
				break
			pos1_begin = motor.getPos(m1)
			pos2_begin = motor.getPos(m2)
			motor.runDoubleDirect(m1, m2, speed, speed)
			last_dir = 0
		else:
			motor.setDoubleSpeed(m1, m2, speed, speed)
			# last_dir -= 0.1
			# motor.runDoubleDirect(m1, m2, speed, speed)
			# pos1_begin = motor.getPos(m1)
			# pos2_begin = motor.getPos(m2)

	motor.stop(m1)
	motor.stop(m2)

def collector_up(m):
	motor.runSingleRelat(m, 20, -100, "hold")
	# motor.waitForHold(m)
	time.sleep(2)

def collector_down(m):
	motor.runSingleRelat(m, 20, 100, "hold")
	# motor.waitForHold(m)
	time.sleep(2)

def claw_out(m):
	motor.runSingleRelat(m, 50, 300, "hold")
	# motor.waitForHold(m)
	time.sleep(2.5)

def claw_in(m):
	motor.runSingleRelat(m, 20, -300, "hold")
	# motor.waitForHold(m)
	time.sleep(2.5)

def catch_ball(claw, collector):
	claw_in(claw)
	claw_out(claw)

def release_ball(claw, collector):
	collector_down(collector)

def goto_ball(m1, m2, us1, dist):
	rot = math.atan(100 / (abs(dist - 50))) / 360 * 110 * 110 * 3.1415
	motor.runDoubleRelat(m1, m2, 60, -rot, 60, rot, "hold")
	motor.waitForDoubleHold(m1, m2)
	motor.runDoubleRelat(m1, m2, 70, 5 * dist, 70, 5 * dist)
	motor.waitForDoubleStop(m1, m2)

	"""
	val = sensor.val(us1)
	if val > 20 and val != 2550:
		motor.runDoubleDirect(m1, m2, 60, 60)
		pos1 = motor.getPos(m1)
		# pos2 = motor.getPos(m2)
		val = sensor.val(us1)
		while 1:
			if val <= 20 or val == 2550:
				motor.stop(m1)
				motor.stop(m2)
				break
			val = sensor.val(us1)
	# motor.runDoubleRelat(m1, m2, 60, -130 + dist / 20, 60, 130 - dist / 20, "hold")
	motor.runDoubleRelat(m1, m2, 60, -90, 60, 90, "hold")
	motor.waitForDoubleHold(m1, m2)
	# motor.runDoubleRelat(m1, m2, 70, 5 * dist, 70, 5 * dist)
	# motor.waitForDoubleStop(m1, m2)
	motor.runDoubleRelat(m1, m2, 70, 100, 70, 100)
	motor.waitForDoubleStop(m1, m2)
	return motor.getPos(m1) - pos1
	"""
	return 5 * dist

def catch_balls(m1, m2, us1, claw, collector):
	motor.setPolarity(m1, "inversed")
	motor.setPolarity(m2, "inversed")
	sensor.setMode(us1, "US-DIST-CM")
	# motor.runDoubleRelat(m1, m2, 60, 3000, 60, -3000)
	touch1 = sensor.val(sensors[1])
	collector_up(collector)
	claw_out(claw)
	while not touch1:
		touch1 = sensor.val(sensors[1])
		us1_val = sensor.val(us1)
		# print(us1_val)
		if (us1_val < 250):
			motor.stop(m1)
			motor.stop(m2)
			speak("find a ball")
			dist = goto_ball(m1, m2, us1, us1_val)
			catch_ball(claw, collector)
			motor.runDoubleRelat(m1, m2, 70, -dist, 70, -dist)
			motor.waitForDoubleStop(m1, m2)
			speak("ha")
			# motor.runDoubleRelat(m1, m2, 60, 3000, 60, -3000)
			# print("stop!!")
			# break
		motor.runDoubleRelat(m1, m2, 80, 40, 80, -40, "hold")
		motor.waitForDoubleHold(m1, m2)
		time.sleep(0.1)
	motor.stop(m1)
	motor.stop(m2)
	collector_down(collector)

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

#collector_up(motors[3])
#collector_down(motors[3])
#collector_up(motors[3])
#collector_down(motors[3])

# catch_ball(motors[0], motors[3])
# release_ball(motors[0], motors[3])

catch_balls(motors[2], motors[1], sensors[2], motors[0], motors[3])
exit()

if 0:
	sensor.setMode(sensors[0][1], "COL-REFLECT")
	sensor.setMode(sensors[0][2], "COL-REFLECT")


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

	time.sleep(2)
	speak("start")

	pid(motors[1], motors[2], sensors[0][1], sensors[0][2], 45, val1_1, val1_2, val2_1, val2_2, 7, 0.005, 40, inter)
else:
	sensor.setMode(sensors[0][1], "COL-COLOR")
	sensor.setMode(sensors[0][2], "COL-COLOR")
	line(motors[1], motors[2], sensors[0][1], sensors[0][2], 45)

#! /usr/bin/python3

import sensor
import motor
import sys
import time
import math

motors = motor.initList(motor.getList())
sensors = sensor.initList(sensor.getList())

config = {
	"check_period": 400,
	"rs_sample_time": 1, # sec
	"search_offset": 40
}

"""
def isBlack(val):
	return val <= 22

def isWhite(val):
	return val > 22

def isBlack_s(val):
	return val <= 17

def isWhite_s(val):
	return val > 17
"""

def isBlack(val):
	return val == 1

def isWhite(val):
	return val == 6

def isBlack_s(val):
	return val == 1

def isWhite_s(val):
	return val == 6

def line(m1 = motors[1],
		 m2 = motors[2],
		 sen1 = sensors[1],
		 sen2 = sensors[2],
		 touch1 = sensors[0],
		 touch2 = sensors[3],
		 speed = 40):

	sensor.setMode(sen1, "COL-COLOR")
	sensor.setMode(sen2, "COL-COLOR")

	pos1_st = motor.getPos(m1)
	pos2_st = motor.getPos(m2)
	pos1_ed = 0
	pos2_ed = 0

	rs1_st = motor.getPos(m1)
	rs2_st = motor.getPos(m2)
	time_st = time.time()
	time_ed = 0
	rs1_ed = 0
	rs2_ed = 0
	mount_st = 0

	fs1_o = find_speed1 = 70
	fs2_o = find_speed2 = 70
	sp_o = speed

	val1 = 0
	val2 = 0

	last_dir = 0
	mount_mode = 0

	on_slope = 0
	slow_time_st = 0

	def obstacle(dir = 1):
		motor.runDoubleRelat(m1, m2, 70, -300, 70, -300)
		motor.waitForDoubleStop(m1, m2)

		motor.runDoubleRelat(m1, m2, 70, -dir * 400, 70, dir * 400)
		motor.waitForDoubleStop(m1, m2)

		motor.runDoubleDirect(m1, m2, 50 + dir * 30, 50 - dir * 30)
		while not (isBlack(sensor.val(sen1)) or isBlack(sensor.val(sen2))):
			pass

	def openMountMode():
		nonlocal find_speed1, find_speed2
		nonlocal fs1_o, fs2_o
		nonlocal sp_o
		nonlocal mount_st
		nonlocal mount_mode

		print("mount mode")
		find_speed1 = 50
		find_speed2 = 50
		speed = 45
		mount_st = time.time()
		config["check_period"] = 800
		mount_mode = 1

	def closeMountMode():
		nonlocal find_speed1, find_speed2
		nonlocal fs1_o, fs2_o
		nonlocal sp_o
		nonlocal mount_st
		nonlocal mount_mode

		print("*****************")
		find_speed1 = fs1_o
		find_speed2 = fs2_o
		speed = sp_o
		mount_st = time.time()
		config["check_period"] = 400
		mount_mode = 0

	def resetRealSpeedDetector():
		nonlocal rs1_st
		nonlocal rs2_st
		nonlocal time_st
		rs1_st = motor.getPos(m1)
		rs2_st = motor.getPos(m2)
		time_st = time.time()

	def resetBreakLine():
		nonlocal pos1_st
		nonlocal pos2_st
		pos1_st = motor.getPos(m1)
		pos2_st = motor.getPos(m2)

	def resetAllDetector():
		nonlocal pos1_st
		nonlocal pos2_st
		nonlocal rs1_st
		nonlocal rs2_st
		nonlocal time_st
		rs1_st = pos1_st = motor.getPos(m1)
		rs2_st = pos2_st = motor.getPos(m2)
		time_st = time.time()

	def checkBreakLine():
		found_line = 0
		while not found_line:
			motor.runDoubleRelat(m1, m2, find_speed1, -300, find_speed1, 300, "hold")
			while not (motor.isHolding(m1) or motor.isHolding(m2)):
				if isBlack_s(sensor.val(sen1)) or isBlack_s(sensor.val(sen2)):
					found_line = 1
					break
			else:
				motor.runDoubleRelat(m1, m2, find_speed1, 600 + config["search_offset"] * last_dir,
											 find_speed1, -600 - config["search_offset"] * last_dir, "hold")
				while not (motor.isHolding(m1) or motor.isHolding(m2)):
					if isBlack_s(sensor.val(sen1)) or isBlack_s(sensor.val(sen2)):
						found_line = 1
						break
				else:
					motor.runDoubleRelat(m1, m2, find_speed1, -300, find_speed1, 300, "hold")
					motor.waitForDoubleHold(m1, m2)
					motor.runDoubleRelat(m1, m2, find_speed1, 400, find_speed1, 400, "hold")
					motor.waitForDoubleHold(m1, m2)

	motor.runDoubleDirect(m1, m2, speed, speed)

	for i in range(0, 5000):
		val1 = sensor.val(sen1)
		val2 = sensor.val(sen2)
		touch1_val = sensor.val(touch1)
		touch2_val = sensor.val(touch2)

		pos1_ed = motor.getPos(m1)
		pos2_ed = motor.getPos(m2)

		is_b1 = isBlack(val1)
		is_b2 = isBlack(val2)

		if touch1_val:
			obstacle()
			motor.runDoubleDirect(m1, m2, speed, speed)
			resetAllDetector()
			continue

		if touch2_val:
			motor.runDoubleRelat(m1, m2, 90, 400, 90, 400, "hold")
			motor.waitForDoubleHold(m1, m2)
			motor.runDoubleDirect(m1, m2, speed, speed)
			resetAllDetector()
			continue

		"""
		if touch2_val:
			motor.runDoubleRelat(m1, m2, 90, 400, 90, 400, "hold")
			motor.waitForDoubleHold(m1, m2)
			openMountMode()
			motor.runDoubleDirect(m1, m2, speed, speed)

			resetAllDetector()
		"""
		"""
		time_ed = time.time()
		if on_slope == 2 and time_ed - slow_time_st >= 3:
			speed = sp_o
			find_speed1 = fs1_o
			find_speed2 = fs2_o
			on_slope = 0
		elif time_ed - time_st >= config["rs_sample_time"]:

			real_sp1 = abs(motor.getPos(m1) - rs1_st) / (time_ed - time_st)
			real_sp2 = abs(motor.getPos(m2) - rs2_st) / (time_ed - time_st)

			print(int(real_sp1), " ", int(real_sp2))
			if 0 and (((real_sp1 < 120 and real_sp2 < 120)
					  or (real_sp1 < 50 and real_sp2 < 170)
					  or (real_sp2 < 50 and real_sp1 < 170)) or touch2_val):
				openMountMode()
			elif time_ed - mount_st > 20:
				  # or (real_sp1 > 260 and real_sp2 > 260 and time_ed - mount_st > 5)):
				closeMountMode()
		"""

		if isBlack_s(val1) and isBlack_s(val2):
			if on_slope:
				motor.runDoubleRelat(m1, m2, 70, -100, 70, -100)
				motor.waitForDoubleStop(m1, m2)

				motor.runDoubleRelat(m1, m2, 70, -850, 70, 850)
				motor.waitForDoubleStop(m1, m2)

				while 1:
					motor.runDoubleRelat(m1, m2, 40, -300, 40, -300, "hold")
					motor.waitForDoubleHold(m1, m2)
					motor.runDoubleRelat(m1, m2, 50, 100, 50, 100, "hold")
					motor.waitForDoubleHold(m1, m2)
					if (sensor.val(touch2)):
						break

				motor.runDoubleRelat(m1, m2, 60, -600, 60, -600, "hold")
				motor.waitForDoubleHold(m1, m2)

				motor.runDoubleRelat(m1, m2, 60, 400, 60, -400, "hold")
				motor.waitForDoubleHold(m1, m2)

				motor.runDoubleDirect(m1, m2, 60, -60)
				while not (isBlack(sensor.val(sen1)) or isBlack(sensor.val(sen2))):
					pass

				motor.runDoubleDirect(m1, m2, speed, speed)
				on_slope = 0
			else:
				# double black
				#motor.runDoubleRelat(m1, m2, 70, 100,
				#							 70, 100, "hold")
				#motor.waitForDoubleHold(m1, m2)

				# if mount_mode:
				tmp_pos1 = motor.getPos(m1)
				# tmp_pos2 = motor.getPos(m2)
				motor.runDoubleRelat(m1, m2, 60, 800,
											 60, 800, "hold")
				while (not (motor.hasStopped(m1) or motor.hasStopped(m2))):
					if isWhite(sensor.val(sen1)) or isWhite(sensor.val(sen2)):
						break
					if (motor.getPos(m1) - tmp_pos1 > 70):
						# on_slope = 1
						motor.runDoubleRelat(m1, m2, 70, 400,
													 70, 400, "hold")
						motor.waitForDoubleHold(m1, m2)

						# closeMountMode()
						break

				motor.runDoubleRelat(m1, m2, 70, 120,
											 70, 120, "hold")
				motor.waitForDoubleHold(m1, m2)

				motor.runDoubleRelat(m1, m2, find_speed2, 500,
											 find_speed2, -500, "hold")
				if motor.waitForDoubleHold(m1, m2, 7):
					st = time.time()
					motor.runDoubleDirect(m1, m2, -find_speed2, find_speed2)
					while time.time() - st <= 16:
						if isBlack_s(sensor.val(sen1)):
							last_dir = -1
							break
						if isBlack_s(sensor.val(sen2)):
							last_dir = 1
							break
					else:
						closeMountMode()
				else:
					closeMountMode()
				motor.runDoubleDirect(m1, m2, speed, speed)
			resetAllDetector()
		elif is_b1:
			motor.setDoubleSpeed(m1, m2, -find_speed1, find_speed1)
			last_dir = -1
			resetBreakLine()
		elif is_b2:
			motor.setDoubleSpeed(m1, m2, find_speed1, -find_speed1)
			last_dir = 1
			resetBreakLine()
		elif pos1_ed - pos1_st >= config["check_period"] \
			 or pos2_ed - pos2_st >= config["check_period"]:
			print(pos1_ed, " ", pos1_st)
			print("check break line")
			checkBreakLine()
			motor.runDoubleDirect(m1, m2, speed, speed)
			# resetAllDetector()
			resetBreakLine()
		else:
			motor.setDoubleSpeed(m1, m2, speed, speed)
	motor.stop(m1)
	motor.stop(m2)

line(speed = 40)

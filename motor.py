#! /usr/bin/python3

import os
from const import *

def getList():
	ret = []
	for mot in os.listdir(ev3py_TMotorPath):
		if (len(mot) > 5 and mot[:5] == "motor"):
			fp = open("%s/%s/%s" % (ev3py_TMotorPath, mot, ev3py_TMotorPortName), "r")
			ret.append([int(mot[5:]), fp.readline()[:-1]])
			fp.close()
	return ret

def initList(list):
	ret = [None, None, None, None]
	for mot in list:
		if mot[1][:3] == "out":
			ret[ord(mot[1][3]) - ord('A')] = mot[0]
	return ret;

# file interfaces

def getPath(port, name):
	return "%s/motor%s/%s" % (ev3py_TMotorPath, port, name)

def setCommand(port, cmd):
	fp = open(getPath(port, "command"), "w")
	fp.write(cmd)
	fp.close()

def setPosSP(port, pos):
	fp = open(getPath(port, "position_sp"), "w")
	fp.write(str(int(pos)))
	fp.close()

def setDutyCycleSP(port, val):
	fp = open(getPath(port, "duty_cycle_sp"), "w")
	fp.write(str(int(val)))
	fp.close()

def setStopCommand(port, cmd):
	fp = open(getPath(port, "stop_command"), "w")
	fp.write(cmd)
	fp.close()

def setPolarity(port, pole):
	fp = open(getPath(port, "polarity"), "w")
	fp.write(pole)
	fp.close()

def getPos(port):
	fp = open(getPath(port, "position"), "r")
	ret = fp.readline()[:-1]
	fp.close()
	return int(ret)

def getState(port):
	fp = open(getPath(port, "state"), "r")
	ret = fp.readline()[:-1]
	fp.close()
	return ret

# basic motor interfaces

def runSingleRelat(m1, speed, pos, stop_cmd = "coast"):
	setDutyCycleSP(m1, speed)
	setPosSP(m1, pos)
	setStopCommand(m1, stop_cmd)
	setCommand(m1, "run-to-rel-pos")

def runDoubleRelat(m1, m2, s1, pos1, s2, pos2, stop_cmd = "coast"):
	setDutyCycleSP(m1, s1)
	setPosSP(m1, pos1)
	setStopCommand(m1, stop_cmd)

	setDutyCycleSP(m2, s2)
	setPosSP(m2, pos2)
	setStopCommand(m2, stop_cmd)

	setCommand(m1, "run-to-rel-pos")
	setCommand(m2, "run-to-rel-pos")

def runSingleDirect(m1, speed):
	setDutyCycleSP(m1, speed)
	setCommand(m1, "run-direct")

def runDoubleDirect(m1, m2, s1, s2):
	setDutyCycleSP(m1, s1)
	setDutyCycleSP(m2, s2)
	setCommand(m1, "run-direct")
	setCommand(m2, "run-direct")

def setDoubleSpeed(m1, m2, s1, s2):
	setDutyCycleSP(m1, s1)
	setDutyCycleSP(m2, s2)

def stop(m1, stop_cmd = "coast"):
	setStopCommand(m1, stop_cmd)
	setCommand(m1, "stop")

def reset(m1):
	setCommand(m1, "reset")

def waitForStop(m1):
	while 1:
		if (not "running" in getState(m1).split()):
			return

def waitForDoubleHold(m1, m2):
	while 1:
		if ("holding" in getState(m1).split()
			and "holding" in getState(m2).split()):
			return

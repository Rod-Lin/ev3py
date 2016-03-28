#! /usr/bin/python3

import os
from const import *

def getList():
	ret = []
	for sen in os.listdir(ev3py_SensorPath):
		if (len(sen) > 6 and sen[:6] == "sensor"):
			fp = open("%s/%s/%s" % (ev3py_SensorPath, sen, ev3py_SensorPortName), "r")
			ret.append([int(sen[6:]), fp.readline()[:-1]])
			fp.close()
	return ret

def initList(lst):
	ret = [None, None, None, None]

	for sen in lst:
		port = -1
		for arg in sen[1].split(":"):
			if (arg[:2] == "in"):
				port = int(arg[2:]) - 1
			elif (arg[:3] == "mux"):
				if (not isinstance(ret[port], list)):
					ret[port] = [None, None, None] # 1 to 3 mux
				ret[port][int(arg[3:]) - 1] = sen[0]
		if (ret[port] == None and port >= 0):
			ret[port] = sen[0]

	return ret

def getPath(port, name):
	return "%s/sensor%s/%s" % (ev3py_SensorPath, port, name)

def val(port):
	fp = open(getPath(port, "value0"), "r")
	ret = fp.readline()[:-1]
	fp.close()
	return int(ret)
	
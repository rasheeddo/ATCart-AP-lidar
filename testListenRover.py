# import zmq
import numpy as np
from numpy import pi
import socket
import struct
# import pickle
import time
import os
os.environ["MAVLINK20"] = "2"
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil


# FCU connection variables
vehicle = None
is_vehicle_connected = False

global cur_lat, cur_lon, cur_yaw

cur_lat = 0.0
cur_lon = 0.0
cur_yaw = 0.0

def vehicle_connect():
	global vehicle, is_vehicle_connected

	if vehicle == None:
		try:
			vehicle = connect('/dev/ttyUSB1', wait_ready=True, baud=921600)
		except:
			print('Connection error! Retrying...')
			vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=921600)
			time.sleep(1)

	if vehicle == None:
		is_vehicle_connected = False
		return False
	else:
		is_vehicle_connected = True
		return True

# Callback to print the location in global frame
def location_callback(self, attr_name, value):
	global cur_lat, cur_lon
	cur_lat = value.global_frame.lat
	cur_lon = value.global_frame.lon
	print("cur_lat: %.7f  cur_lon: %.7f" %(cur_lat, cur_lon))
	# print(value.global_frame)

def attitude_callback(self, attr_name, value):
	global cur_yaw
	cur_yaw = value.yaw
	print("cur_yaw: %.6f" %cur_yaw)
	## range is -pi to pi, 0 is north


######################################################################

print("INFO: Connecting to vehicle.")
while (not vehicle_connect()):
	pass
print("INFO: Vehicle connected.")

#Add observer for the vehicle's current location
vehicle.add_attribute_listener('location', location_callback)
vehicle.add_attribute_listener('attitude', attitude_callback)

while True:
	# print("cur_lat: %.7f  cur_lon: %.7f" %(cur_lat, cur_lon))
	time.sleep(0.01)


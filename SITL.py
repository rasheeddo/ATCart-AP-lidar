# import zmq
import numpy as np
import socket
import struct
# import pickle
import time
# import os
# os.environ["MAVLINK20"] = "2"
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# FCU connection variables
vehicle = None
is_vehicle_connected = False

def send_msg_to_gcs(text_to_be_sent):
	# MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
	# Defined here: https://mavlink.io/en/messages/common.html#MAV_SEVERITY
	# MAV_SEVERITY = 3 will let the message be displayed on Mission Planner HUD, but 6 is ok for QGroundControl
	if is_vehicle_connected == True:
		text_msg = 'LIDAR: ' + text_to_be_sent
		status_msg = vehicle.message_factory.statustext_encode(
			3,                      # MAV_SEVERITY
			text_msg.encode()	    # max size is char[50]       
		)
		vehicle.send_mavlink(status_msg)
		vehicle.flush()
		print("INFO: " + text_to_be_sent)
	else:
		print("INFO: Vehicle not connected. Cannot send text message to Ground Control Station (GCS)")


def vehicle_connect():

	vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)




######################################################################

print("INFO: Connecting to vehicle.")
# while (not vehicle_connect()):
# 	pass
vehicle = connect('udp:127.0.0.1:14551', wait_ready=True)
print("INFO: Vehicle connected.")

# print('Autopilot Firmware version: %s' % vehicle.version)
# print('Autopilot capabilities (supports ftp): %s' % vehicle.capabilities.ftp)
# print('Global Location: %s' % vehicle.location.global_frame)
# print('Global Location (relative altitude): %s' % vehicle.location.global_relative_frame)
# print('Local Location: %s' % vehicle.location.local_frame)    #NED
# print('Attitude: %s' % vehicle.attitude)
# print('Velocity: %s' % vehicle.velocity)
# print('GPS: %s' % vehicle.gps_0)
# print('Groundspeed: %s' % vehicle.groundspeed)
# print('Airspeed: %s' % vehicle.airspeed)
# print('Gimbal status: %s' % vehicle.gimbal)
# print('Battery: %s' % vehicle.battery)
# print('EKF OK?: %s' % vehicle.ekf_ok)
# print('Last Heartbeat: %s' % vehicle.last_heartbeat)
# print('Rangefinder: %s' % vehicle.rangefinder)
# print('Rangefinder distance: %s' % vehicle.rangefinder.distance)
# print('Rangefinder voltage: %s' % vehicle.rangefinder.voltage)
# print('Heading: %s' % vehicle.heading)
# print('Is Armable?: %s' % vehicle.is_armable)
# print('System status: %s' % vehicle.system_status.state)
# print('Mode: %s' % vehicle.mode.name)    # settable
# print('Armed: %s' % vehicle.armed)    # settable

msg = vehicle.message_factory.command_long_encode(
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
		0, #confirmation
		30,    # param 1, yaw in degrees
		0,          # param 2, yaw speed deg/s
		1,          # param 3, direction -1 ccw, 1 cw
		1, # param 4, relative offset 1, absolute angle 0
		0, 0, 0)    # param 5 ~ 7 not used
	# send command to vehicle

msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        100, 0, 0, # x, y, z positions (not used)
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000101111111111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        1.57, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111110110, # type_mask (only speeds enabled)
        10, 0, 0, # x, y, z positions (not used)
        5, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

vehicle.send_mavlink(msg)


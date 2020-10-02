import zmq
import numpy as np
import socket
import struct
import pickle
import time
import os
os.environ["MAVLINK20"] = "2"
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# FCU connection variables
vehicle = None
is_vehicle_connected = False

##### Obstacle distance params #####
# Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
# See here: https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
min_distance = 120  # In cm, this is the distance from lidar to cart's nose plus a litte bit for safety
max_distance = 1500  # In cm, I found the lidar never give the value over than 15meters
distances_array_length = 72
angle_offset = -90.0	# deg, the first index of distances is -90.0deg
increment_f  = 2.5		# deg, increment angle of our array
distances = np.ones((distances_array_length,), dtype=np.uint16) * (max_distance + 1)

######################################################
##  Functions - MAVLink                             ##
######################################################

# https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
# def send_obstacle_distance_message():
# 	global current_time_us, distances, min_distance, max_distance, angle_offset, increment_f
# 	if angle_offset is None or increment_f is None:
# 		print("Please call set_obstacle_distance_params before continue")
# 	else:
# 		print("Send msg")
# 		msg = vehicle.message_factory.obstacle_distance_encode(
# 			current_time_us,    # us Timestamp (UNIX time or time since system boot)
# 			0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
# 			distances,          # distances,    uint16_t[72],   cm
# 			0,                  # increment,    uint8_t,        deg
# 			min_distance,	    # min_distance, uint16_t,       cm
# 			max_distance,       # max_distance, uint16_t,       cm
# 			increment_f,	    # increment_f,  float,          deg
# 			angle_offset,       # angle_offset, float,          deg
# 			12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
# 		)

# 		vehicle.send_mavlink(msg)
# 		vehicle.flush()

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
	global vehicle, is_vehicle_connected

	if vehicle == None:
		try:
			print("Connecting to Ardupilot....")
			vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=921600)
		except:
			print('Connection error! Retrying...')
			vehicle = connect('/dev/ttyUSB1', wait_ready=True, baud=921600)
			time.sleep(1)

	if vehicle == None:
		is_vehicle_connected = False
		return False
	else:
		is_vehicle_connected = True
		return True


# Connect to the Vehicle
#vehicle = connect('/dev/ttyTHS0', wait_ready=True, baud=921600)
# print("Connecting to Ardupilot...")
# vehicle = connect('/dev/ttyUSB1', wait_ready=True, baud=115200)
# print("Connection completed")

################################## ZMQ UDP ###########################
context = zmq.Context()
## we create a socket as we usually do
# DATA_PORT = 3101
# data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# data_sock.bind(("0.0.0.0", DATA_PORT))
# data_sock.setblocking(0)

WEBRTC_PORT = 22334
webrtc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
webrtc_sock.bind(("0.0.0.0", WEBRTC_PORT))
webrtc_sock.setblocking(0)

## us ZMQ to poll data from c++ collision-detector
poller = zmq.Poller()
# poller.register(data_sock, zmq.POLLIN)
poller.register(webrtc_sock, zmq.POLLIN)
######################################################################

print("INFO: Connecting to vehicle.")
while (not vehicle_connect()):
	pass
print("INFO: Vehicle connected.")


obstacle_distance_msg_hz = 15.0

# sched = BackgroundScheduler()
# sched.add_job(send_obstacle_distance_message, 'interval', seconds = 1/obstacle_distance_msg_hz)
# send_msg_to_gcs('Sending obstacle distance messages to FCU')

last_time = time.time()

while True:

	# Store the timestamp for MAVLink messages
	current_time_us = int(round(time.time() * 1000000))
	
	try:
		socks = dict(poller.poll())
	except KeyboardInterrupt:
		break

	if webrtc_sock.fileno() in socks:
		data, addr = webrtc_sock.recvfrom(1500)
		parse_data = pickle.loads(data)
		if parse_data.startswith('MANUAL'):
			print(parse_data)
			vehicle.mode = 'MANUAL'
		elif parse_data.startswith('HOLD'):
			print(parse_data)
			vehicle.mode = 'HOLD'
		elif parse_data.startswith('TURNLEFT'):
			print(parse_data)
			vehicle.channels.overrides['1'] = 1300
			vehicle.channels.overrides['2'] = 1500
		elif parse_data.startswith('TURNRIGHT'):
			print(parse_data)
			vehicle.channels.overrides['1'] = 1700
			vehicle.channels.overrides['2'] = 1500
		elif parse_data.startswith('STR'):
			print(parse_data)
			try:
				pieces = parse_data.split(':')
				STR_val = (-1)*float(pieces[1])
				THR_val = float(pieces[3])
				print(STR_val, THR_val)
				#val = float(pieces[1])
				#sbus_throttle = int(round(670 * val + 1024))
				steering_pwm = int(round(STR_val*200 + 1500))
				throttle_pwm = int(round(THR_val*200 + 1500))
				#print("throttle_pwm", throttle_pwm)
			except Exception as ee:
				print('failed to parse  :', ee)
			else:
				vehicle.channels.overrides['1'] = steering_pwm
				vehicle.channels.overrides['2'] = throttle_pwm

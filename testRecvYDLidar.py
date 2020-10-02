import zmq
import numpy as np
import socket
import struct
import pickle
import time

distances_array_length = 72
min_distance = 120  # In cm, this is the distance from lidar to cart's nose plus a litte bit for safety
max_distance = 1500  # In cm, I found the lidar never give the value over than 15meters
distances = np.ones((distances_array_length,), dtype=np.uint16) * (max_distance + 1)

################################## ZMQ UDP ###########################
## we create a socket as we usually do
DATA_PORT = 3101
lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
lidar_sock.bind(("0.0.0.0", DATA_PORT))
lidar_sock.setblocking(0)

######################################################################

last_time = time.time()

while True:

	# Store the timestamp for MAVLink messages
	current_time_us = int(round(time.time() * 1000000))
	try:
		data, addr = lidar_sock.recvfrom(1500)

	except socket.error:
		pass
	else:
		#print(len(data))
		array_size = len(data)/4
		
		## unpack the whole packet into buff (tuple-type)
		buff = struct.unpack('%df' %(array_size), data)

		## first half of buf is angle and second half is dist
		angle_array = np.asarray(buff[0:int(array_size/2)])
		dist_array = np.asarray(buff[int(array_size/2):int(array_size)])

		## reverse array because 72 distances in OBSTACLE_DISTANCE
		## it starts from left side to right side (clockwise), but lidar scan (counter-clockwise)
		angle_array = angle_array[::-1]
		dist_array = dist_array[::-1]*100.0  # change to cm unit
		
		dist_array = dist_array.astype(np.uint16)	# convert to uint16 for AP

		for i in range(len(dist_array)):
			if dist_array[i] == 0:
				dist_array[i] = 65535
			else:
				pass

		distances = np.copy(dist_array)
		# if ((time.time() - last_time) >= 1/obstacle_distance_msg_hz):
		# 	msg = vehicle.message_factory.obstacle_distance_encode(
		# 		current_time_us,    # us Timestamp (UNIX time or time since system boot)
		# 		0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
		# 		distances,          # distances,    uint16_t[72],   cm
		# 		0,                  # increment,    uint8_t,        deg
		# 		min_distance,	    # min_distance, uint16_t,       cm
		# 		max_distance,       # max_distance, uint16_t,       cm
		# 		increment_f,	    # increment_f,  float,          deg
		# 		angle_offset,       # angle_offset, float,          deg
		# 		12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
		# 	)

		# 	vehicle.send_mavlink(msg)
		# 	vehicle.flush()

		# 	last_time = time.time()

		print(distances)

		# print(dist_array)
		print(" ")
		

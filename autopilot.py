from dronekit import connect
import zmq
import numpy as np
import socket
import struct
import pickle
import time

# Connect to the Vehicle
#vehicle = connect('/dev/ttyTHS0', wait_ready=True, baud=921600)
print("Connecting to Ardupilot...")
vehicle = connect('/dev/ttyUSB1', wait_ready=True, baud=115200)
print("Connection completed")
context = zmq.Context()

AUTOPILOT_PORT = '22334'
ap_socket = context.socket(zmq.SUB)
ap_socket.connect("tcp://127.0.0.1:"+AUTOPILOT_PORT)
ap_socket.setsockopt_string(zmq.SUBSCRIBE, np.unicode(''))

poller = zmq.Poller()
poller.register(ap_socket, zmq.POLLIN)

current_mode = 'HOLD'
timeout = 100
prev_parse_data = 'NONE'
unlock = True
last_change_time = time.time()
while True:
	
	try:
		socks = dict(poller.poll(timeout))
	except KeyboardInterrupt:
		break

	if ap_socket in socks:
		data = ap_socket.recv(zmq.NOBLOCK)

		parse_data = pickle.loads(data)

		if (parse_data != prev_parse_data):
			if (parse_data != current_mode) and ((time.time() - last_change_time) > 1.0):
				current_mode = parse_data
				print("Current Mode: %s" %(current_mode))
				vehicle.mode = current_mode
				
			else:
				pass

			last_change_time = time.time()



		prev_parse_data = parse_data
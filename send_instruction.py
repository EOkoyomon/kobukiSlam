import binascii
import select
import socket
import struct
import subprocess
import sys
import time
from duck_detect import duck_direction
from plan_route import plan_route

FOLDER = "/home/aaron/.ros/"
POINT_CLOUD_FOLDER = "/home/aaron/catkin_ws/src/pc_data/data/"
ROS_WRITE_COMMAND = "rosrun pc_data collector.py"
CLOUD_FILE = POINT_CLOUD_FOLDER + "final_cloud.pcd"
POSITIONS_FILE = POINT_CLOUD_FOLDER + "final_position.txt"
SERVER_ADDR = "10.42.0.1"
SERVER_PORT = 8080
SLEEP_INTERVAL_IN_S = 0.01
i = 0
DEBUG = True

class Client():
	def __init__(self, address, port):
		# Destination IP Address
		self.address = address

		# Destination Port
		self.port = int(port)

		# Opens Socket (default arguments: AF_INET means we use IPv4, SOCK_STREAM means use TCP)
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
                self.start_read = False

	def connect(self):
		# Connect to Destination Machine
		destination_IP = self.address
		destination_Port = self.port
		self.socket.connect((destination_IP, destination_Port))

	def sendInfo(self, info):
		# Send (transmit) message to destination socket
		arr = [struct.pack("i", int(num)) for num in info]
		data = b"".join(arr)
		self.socket.send(data)
		# If center
		if info[1]:
			self.start_read = True
                        print("starting to read")
		# if info[0] or info[1] or info[2]:
		#	print("Network writes:", i)

	def sendInstructions(self, list_of_instructions):
		data = [struct.pack("i", 249), struct.pack("i", len(list_of_instructions))]
		for angle, distance in list_of_instructions:
			data.append(struct.pack("f", angle))
			data.append(struct.pack("f", distance))
		self.socket.send(b"".join(data))

	def recvSignal(self):
		ready_to_read, _, _ = select.select([self.socket], [], [], 0)
                print("Trying to read")
		for sock in ready_to_read:
			if sock == self.socket:
                                print("Reading")
				signal = self.socket.recv(4)
                                if not signal:
                                        print("Read failed - connection closed")
                                        exit(1)

                                # int_signal = int(binascii.hexlify(signal)[::-1], 16)
                                int_signal = struct.unpack("i", signal)[0]
				if int_signal == 249:
                                        print("Got the start instruction signal!")
					return True
                                else:
                                        print("Wrong int my guy", int_signal)

		return False

if __name__ == "__main__":
	#print("Enter IP Address or Hostname")
	# sys.stdout.flush()
	# address = input()
	address = SERVER_ADDR
	
	# print("Enter Port Number")
	# sys.stdout.flush()
	# port = input()
	port = SERVER_PORT

	client = Client(address, port)
	client.connect()
	print("Connected to " + SERVER_ADDR)

	start_mapping_back = False
	#msg = "0,0,0"
	while True:
		i += 1

		if DEBUG:
			print("Enter Message (comma separated detect left, center, right)")
			sys.stdout.flush()
			# Gets message from stdin (user input)
			msg = input()
			client.sendInfo(msg)
		else:
			# Sends input (msg) to specified socket
			detect_left, detect_center, detect_right = duck_direction(FOLDER)
			client.sendInfo([detect_left, detect_center, detect_right])
	
		if client.start_read:
			start_mapping_back = client.recvSignal()

		if start_mapping_back:
			break
	
		time.sleep(SLEEP_INTERVAL_IN_S)

	# Start slam stuff - creates the point cloud information needed to plan route
	""" Call slam
	
	# before python send_isntruction. and it will start the kinect, start saving rgb images to ~/.ros, and start slam building.
	roslaunch pc_data pc_data.launch
	# when we get to the duck
	rosrun pc_data collector.py _path:="/home/aaron/catkin_ws/src/pc_data/data/"
	> saves 2 files. final_cloud.pcd, and positions.txt.
prashanth should parse positions.txt for the x y and z and orientation, and 
	"""
	while True:
		if DEBUG:
			# Turn 180 degrees, drive .28m, rotate -37.6 degrees, drive.28m, rotate 0, drive .28
			list_of_instructions = [(180.0, 0.28), (-37.6, 0.28), (0, 0.28)]
			client.sendInstructions(list_of_instructions)
		else:
			process = subprocess.Popen(ROS_WRITE_COMMAND, shell=True, stdout=subprocess.PIPE)
			process.wait()
			
			# Get instructions - Returns a list of tuples of (angle, distance)
			list_of_instructions = plan_route(CLOUD_FILE, POSITIONS_FILE)

			# Send instrucitons
			client.sendInstructions(list_of_instructions)

		time.sleep(SLEEP_INTERVAL_IN_S)

	print("I'm done")
	

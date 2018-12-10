import socket
import struct
import sys
import time
from duck_detect import duck_direction

FOLDER="duck_vid/"
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
		if info[0] or info[1] or info[2]:
			print("Network writes:", i)


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

	#msg = "0,0,0"
	while True:
		i += 1

		if DEBUG:
			print("Enter Message (comma separated detect left, center, right)")
			sys.stdout.flush()
			# Gets message from stdin (user input)
			msg = input()
			client.sendInfo(msg.split(","))
		else:
			# Sends input (msg) to specified socket
			detect_left, detect_center, detect_right = duck_direction(FOLDER)
			client.sendInfo([detect_left, detect_center, detect_right])
		
		time.sleep(SLEEP_INTERVAL_IN_S)



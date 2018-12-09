import socket
import struct
import sys

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

	def sendInfo(self, message):
		# Send (transmit) message to destination socket
		# left, center, right, dist = message.split(",")
		# print(type(message))
		msg = message.split(",")
		arr = [struct.pack("f", float(num)) for num in msg]
		self.socket.send(b"".join(arr))


if __name__ == "__main__":
	print("Enter IP Address or Hostname")
	sys.stdout.flush()
	address = input()
	
	print("Enter Port Number")
	sys.stdout.flush()
	port = input()

	client = Client(address, port)
	client.connect()

	msg = "1,1,1,1"
	while True:
		# print("Enter Message (comma separated detect left, center, right, dist)")
		# sys.stdout.flush()

		# Gets message from stdin (user input)
		# msg = input()

		# Sends input (msg) to specified socket
		client.sendInfo(msg)



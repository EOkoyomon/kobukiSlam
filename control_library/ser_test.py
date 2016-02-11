import serial
import time

ser = serial.Serial("/dev/serial0", 115200, timeout = 1.0, rtscts = 0)

i = 0
while True:
	ser.write(bytes("hello" + str(i), "utf-8"))
	read = ser.read(6)
	print(read)
	time.sleep(1)
	i +=1 

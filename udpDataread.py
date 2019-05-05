import socket
import sys
from ctypes import *

UDP_IP = "192.168.1.100"
UDP_PORT = 4242

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind(('', UDP_PORT))

class Data(Structure):
	_fields_ = [("odo", c_double*3),("imu", c_double*6),("heading", c_double)]

while True:
	buff = sock.recv(sizeof(Data))
	myData = Data.from_buffer_copy(buff)
	print "X=%d, Y=%d, phi=%d" % (myData.odo[0], myData.odo[1], myData.odo[2])
	print "ax=%f, ay=%f, az=%f, mx=%d, my=%d, mz=%d" % (myData.imu[0], myData.imu[1], myData.imu[2], myData.imu[3], myData.imu[4], myData.imu[5])
	print "Heading=%d" % (myData.heading)

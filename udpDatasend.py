import socket
import sys
from ctypes import *
import time

UDP_IP = '192.168.1.1'
UDP_PORT = 4242

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.connect((UDP_IP, UDP_PORT))
					 
class Robot(Structure):
	_fields_ = [("velocity", c_double),("theta", c_double),("mode", c_int)] #setting up c type struct

while True:
	sendRobot = Robot(0,0,2) #parse data
	sock.sendto(sendRobot, (UDP_IP, UDP_PORT))
	time.sleep(0.1)

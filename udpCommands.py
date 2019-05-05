import socket
import sys
from ctypes import *
import time

v = 0
t = 0

UDP_IP = '192.168.1.1'
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET,
                     socket.SOCK_DGRAM)
sock.connect((UDP_IP, UDP_PORT))

class Robot(Structure):
    _fields_ = [("velocity", c_double),("theta", c_double),("mode",c_int)]


print("Welcome, the controls for the robot are:")
print("q is to exit")
print("r is to restart the robot")
print("Space bar is to stop")
print("w is to servo 0 deg")
print("d is to servo 90 deg")
print("s is to servo 180 deg")
print("a is to servo 270 deg")
print("Up arrow is to increase speed")
print("Down arrow is to decrease speed")
print("Left arrow is to increase angle")
print("Right arrow is to decrease angle")
while True:
	if (getch.getch() == '\033'): #get velocity term
		b = getch.getch()
		c = getch.getch()
		print("b is equal to: ") #for testing
		print(b) #for testing
		if (c == 'A'): #UP ARROW
			v += 10
			if (v > 255):
				v = 255
			sendRobot = Robot(v,t,0) #parse data
			sock.send(sendRobot) #send parse data
		elif (c == 'B'): #DOWN ARROW
			v -= 10 
			if (v < 0):
				v = 0
			sendRobot = Robot(v,t,0) #parse data
			sock.send(sendRobot) #send parse data
		elif (c == 'C'): #RIGHT ARROW
			t -= 15 
			if (t < 0):
				t += 360
			sendRobot = Robot(v,t,0) #parse data
			sock.send(sendRobot) #send parse data
		elif (c == 'D'): #LEFT ARROW
			t += 15 
			if (t > 360):
				t -= 360
			sendRobot = Robot(v,t,0) #parse data
			sock.send(sendRobot) #send parse data
	a = getch.getch()
	if (a == 'w'):
		t = 0
		sendRobot = Robot(v,t,0) #parse data
		sock.send(sendRobot) #send parse data
	elif (a == 'd'):
		t = 90
		sendRobot = Robot(v,t,0) #parse data
		sock.send(sendRobot) #send parse data
	elif (a == 's'):
		t = 180
		sendRobot = Robot(v,t,0) #parse data
		sock.send(sendRobot) #send parse data
	elif (a	== 'a'):
		t = 270
		sendRobot = Robot(v,t,0) #parse data
		sock.send(sendRobot) #send parse data
	elif (a == 'r'): #restart check
		sendRobot = Robot(0,0,1) #parse data
		sock.send(sendRobot) #send parse data
		print "X=%d, Y=%d, phi=%f" % (myData.X, myData.Y, myData.phi)
	elif (a == ' '):
		sendRobot = Robot(0,0,3) #parse data
		sock.send(sendRobot) #send parse data
	elif (a == 'q'):
		print("Exiting")
		sys.exit()

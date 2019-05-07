import socket
import sys
from ctypes import *
import time
import threading
import getch

v = 0
t = 0

UDP_IP = '192.168.1.1'
UDP_PORT = 5005
UDP_PORT2 = 4242

sock = socket.socket(socket.AF_INET,
                     socket.SOCK_DGRAM)
sock2 = socket.socket(socket.AF_INET,
                      socket.SOCK_DGRAM)
sock.connect((UDP_IP, UDP_PORT))
sock2.connect((UDP_IP, UDP_PORT2))

#sock.bind(('', UDP_PORT2))

class Robot(Structure):
    _fields_ = [("velocity", c_double),("theta", c_double),("mode",c_int)]

class Data(Structure):
        _fields_ = [("odo", c_double*3),("imu", c_double*6),("heading", c_do$

def udpRead():
        while True:
                buff = sock2.recv(sizeof(Data))
                myData = Data.from_buffer_copy(buff)
                print "X=%d, Y=%d, phi=%d" % (myData.odo[0], myData.odo[1], $
                print "ax=%f, ay=%f, az=%f, mx=%d, my=%d, mz=%d" % (myData.i$
                print "Heading=%d" % (myData.heading)

def udpSend():
        while True:
                sendRobot = Robot(0,0,2) #parse data
                sock2.sendto(sendRobot, (UDP_IP, UDP_PORT2))
                time.sleep(0.1)

T = threading.Thread(target=udpRead)
T.daemon=True
S = threading.Thread(target=udpSend)
S.daemon=True
T.start()
S.start()

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
while True:
        a = getch.getch()
        if (a == '\033'): #get velocity term
                b = getch.getch()
                c = getch.getch()
                #print("c is equal to: ") #for testing
                #print(c) #for testing
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
        #print(a)
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
        elif (a == 'a'):
                t = -90
                sendRobot = Robot(v,t,0) #parse data
                sock.send(sendRobot) #send parse data
        elif (a == 'r'): #restart check
                sendRobot = Robot(0,0,1) #parse data
                sock.send(sendRobot) #send parse data
        elif (a == ' '):
                sendRobot = Robot(0,0,3) #parse data
                sock.send(sendRobot) #send parse data
        elif (a == 'q'):
                print("Exiting")
                sys.exit()

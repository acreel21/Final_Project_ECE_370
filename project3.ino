#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <math.h>
#include <LSM303.h>
#include <Wire.h>
#include "arduino_secrets.h"

#define encoderR 5
#define encoderL 6
#define motorRfow 9
#define motorRbac 10
#define motorLfow 12
#define motorLbac 11
#define BUFSZ 1024

// UDP Struct set-up
typedef struct Robot {
  double Velocity;
  double Theta;
  int Mode;
} Robot;

typedef struct Data {
  double odo[3];
  double imu[6];
  double heading;
} Data;

Robot myRobot;
Robot myRobot2;
Data myData;
// UDP Struct set-up

// AP Mode Set-up
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key Index number (needed only for WEP)
IPAddress ip;
int status = WL_IDLE_STATUS;
// AP Mode Set-up

//UDP Set-up
IPAddress server(192, 168, 1, 1); //udp ip address
unsigned int localPort = 5005; //udp port
IPAddress udpIP(192, 168, 1, 100); //udp ip address
unsigned int udpPort = 4242; //udp port
WiFiUDP UdpOne;
WiFiUDP UdpTwo;
char recvBuffer[BUFSZ];
//UDP Set-up

// IMU Set up
LSM303 compass;
float azold = 0.94;
float ax;
float ay;
float az;
float mx;
float my;
float mz;
float head;
// IMU Set up

// Odometry Set-up
int L = 90; //base length
float r = 15; //radius of wheels
float cl = 2*3.14*(L); //circumference
float cr = 2*3.14*(r); //circumference
float d = cr/(76*4);
float phi = (360*(d/cl))*((2*3.14)/360); //angle
float deltaX = (L/2)*sin(phi);
float deltaY = (L/2)-(L/2)*cos(phi);
float xGlobal = 0;
float yGlobal = 0;
float zGlobal = 0.0;
float phiGlobal = 0;
float deltaXp = 0; 
float deltaYp = 0;
float phiDir = 0;
// Odometry Set-up

// Closed-loop Control Variables
int cntR = 0; //tick for right wheel
int cntL = 0; //tick for left wheel
float kg = 0.0133333; //gear ratio
int vell = 0; //right wheel velocity
int velr = 0; //left wheel velocity
int w = 0; //angular velocity
int errorDir = 0; //angle error
int errorSper = 0; //Right wheel error
int errorSpel = 0; //Left wheel error
int Kpspe = 25; //Wheel gain
int Kpdir = 2; //Angle gain
int pickup = 0;
int Ml = 0; //left motor
int Mr = 0; //right motor
float thetar = 0;
float thetal = 0;
// Closed-loop Control Variables


void setup() {
  // put your setup code here, to run once:
  wifiSetup(); //setup wifi
  openPort(); //UDP set-up
  pinSetup(); //configure pins and interrupts
  imuSetup();  //set-up IMU 
}

void loop() {
  // put your main code here, to run repeatedly:
  checkImu(); //Updates IMU data 
  checkUDP(); //Updates UDP data
  setMotor(); //turns the motor at the right speed
  setMl(myRobot.Velocity); //Feedback for speed
  setMr(myRobot.Velocity); //Feedback for speed
  setDir(myRobot.Theta);
}

void wifiSetup(){
  //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8, 7, 4, 2);

  //Initialize serial and wait for port to open:
  Serial.begin(9600);

  status = WiFi.beginAP(ssid, pass, 1);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }
}

void openPort(){
  //UDP setup
  UdpOne.begin(localPort);
  UdpTwo.begin(udpPort);
}


void pinSetup(){
  pinMode(motorRfow, OUTPUT);
  pinMode(motorRbac, OUTPUT);
  pinMode(motorLfow, OUTPUT);
  pinMode(motorLbac, OUTPUT);
  pinMode(encoderR, INPUT_PULLUP);
  pinMode(encoderL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderR), encoderR_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderL), encoderL_ISR, CHANGE);
}

void imuSetup(){
  //IMU setup
  Wire.begin();
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-5754, -4079, -4267};
  compass.m_max = (LSM303::vector<int16_t>){+357, +1948, +1990};
  head = compass.heading((LSM303::vector<int>){1, 0, 0});
  phiGlobal = head*(3.14/180);
}

void encoderR_ISR(){
  //Serial.println("Im right");
  cntR++;
  phiGlobal += phi;
  deltaXp = ((deltaX*cos(phiGlobal))+deltaY*sin(phiGlobal));
  deltaYp = ((deltaX*sin(phiGlobal))+deltaY*cos(phiGlobal));
  xGlobal += deltaXp;
  yGlobal += deltaYp;
  myData.odo[0]=xGlobal;
  myData.odo[1]=yGlobal;
  myData.odo[2]=zGlobal;
}

void encoderL_ISR(){
  //Serial.println("Im left");
  cntL++;
  phiGlobal -= phi;
  deltaXp = ((deltaX*cos(phiGlobal))+deltaY*sin(phiGlobal));
  deltaYp = ((deltaX*sin(phiGlobal))+deltaY*cos(phiGlobal));
  xGlobal += deltaXp;
  yGlobal += deltaYp;
  myData.odo[0]=xGlobal;
  myData.odo[1]=yGlobal;
  myData.odo[2]=zGlobal;
}

void checkImu(){
  compass.read();
  ax = (((double)(compass.a.x)*0.061)/1000.0)*9.80665;
  ay = (((double)(compass.a.y)*0.061)/1000.0)*9.80665;
  az = (((double)(compass.a.z)*0.061)/1000.0)*9.80665;
  mx = ((double)(compass.m.x)*0.16)/1000.0;
  my = ((double)(compass.m.y)*0.16)/1000.0;
  mz = ((double)(compass.m.z)*0.16)/1000.0;
  head = compass.heading((LSM303::vector<int>){1, 0, 0});
  myData.imu[0]=ax;
  myData.imu[1]=ay;
  myData.imu[2]=az;
  myData.imu[3]=mx;
  myData.imu[4]=my;
  myData.imu[5]=mz;
  myData.heading=head;
  if(head > 180){
    head -= 360;
  }
  if (head >= -180 && head <= -169){
    head = abs(head);
    //Serial.println(head);
  }
  float ap = (((double)(compass.a.z)*0.061)/1000.0);
  float Jz = (ap-azold);
  if (Jz > 0.5){
    pickup = 1;
  //Serial.println("Pickup");
  }
}

void checkUDP(){
int packetSize = UdpOne.parsePacket(); //parse packet for UDP
  if (packetSize > 0) {
    // read the packet into recvBuffer
    int len = UdpOne.read(recvBuffer, BUFSZ); //read different packet from UDP packet
    if (len > 0) {
      //Serial.println("Contents:");
      //Serial.println(recvBuffer);
      memcpy(&myRobot, recvBuffer, sizeof(myRobot));
      if (myRobot.Mode == 1){
        Serial.println("Restart");
        pickup = 0;
        phiGlobal = head*(3.14/180);
        xGlobal = 0;
        yGlobal = 0;
        errorDir = 0;
        errorSpel = 0;
        errorSper = 0;
        myData.odo[0]=xGlobal;
        myData.odo[1]=yGlobal;
        myData.odo[2]=zGlobal;
      }
      //Serial.println(myRobot.Velocity);
      //Serial.println(myRobot.Theta);
      }
    else {
      Serial.println("Read 0 bytes.");
    }
  }
  int packetSize1 = UdpTwo.parsePacket(); //parse packet for UDP
  if (packetSize1 > 0) {
    // read the packet into recvBuffer
    int len = UdpTwo.read(recvBuffer, BUFSZ); //read different packet from UDP packet
    if (len > 0) {
      //Serial.println("Contents:");
      //Serial.println(recvBuffer);
      memcpy(&myRobot2, recvBuffer, sizeof(myRobot2));
      if (myRobot2.Mode == 2) {
        //Serial.println("Mode 2");
        UdpTwo.beginPacket(UdpTwo.remoteIP(), UdpTwo.remotePort());
        char txBuffer[1024] = { 0 };
        memcpy(txBuffer, &myData, sizeof(Data));
        UdpTwo.write(txBuffer, sizeof(Data));
        UdpTwo.endPacket();
      }
    }
    else {
      Serial.println("Read 0 bytes.");
    }
  }
}

int setMl(double v1){
  float(thetal) = (float(cntL)*(0.5))*kg; //gets speed
  errorSpel = v1 - thetal; //finds error
  vell = errorSpel*Kpspe; //finds velocity for the motors
  //Serial.print("Desired Velocity is: ");
  //Serial.println(v1);
  //Serial.print("Theta is: ");
  //Serial.println(thetal);
  //Serial.print("errorSpeL is: ");
  //Serial.println(errorSpel);
  //Serial.print("Velocity in set M1 is: ");
  //Serial.println(vell);
  cntL=0;
}

int setMr(double v2){
  float(thetar) = (float(cntR)*(0.5))*kg; //gets speed
  errorSper = v2 - thetar; //finds error
  velr = errorSper*Kpspe; //finds velocity for the motors
  cntR=0;
  //Serial.print("Desired Velocity is: ");
  //Serial.println(v2);
  //Serial.print("Theta right is: ");
  //Serial.println(thetar);
  //Serial.print("errorSpe is: ");
  //Serial.println(errorSper);
  //Serial.print("Velocity right is: ");
  //Serial.println(velr);
}

int setDir(double t){
  phiDir =  (int)head;//0.1*(float)phiGlobal*(360/(2*3.14))+0.9*(int)head;
  errorDir = t - phiDir; //finds error
  w = errorDir*Kpdir; //finds angluar speed
  //Serial.print("The desired angluar speed:  ");
  //Serial.println(t);
  Serial.print("The heading:  ");
  Serial.println(head);
  //Serial.print("The phi:  ");
  //Serial.println(phiDir);
  //Serial.print("The phi Global:  ");
  //Serial.println(phiGlobal);
  //Serial.print("The error is:  ");
  //Serial.println(errorDir);
  //Serial.print("The angluar speed:  ");
  //Serial.println(w);
}

void setMotor(){
  Mr = (((2*velr) + (w*L))/(2*r));  //motor speed right wheel
  Ml = (((2*vell) - (w*L))/(2*r)); //motor speed left wheel
//  Serial.print("Velocity in set M1 is: ");
//  Serial.println(vell);
//  Serial.print("Velocity right is: ");
//  Serial.println(velr);
//  Serial.print("The angluar speed:  ");
//  Serial.println(w);
//  Serial.print("Right wheel :  ");
//  Serial.println(Mr);
//  Serial.print("Left wheel :  ");
//  Serial.println(Ml);
  if (Ml > 255){
    Ml = 255;
  }
  if (Mr > 255){
    Mr = 255;
  }
  if (Ml < 0){
    Ml = 0;
  }
  if (Mr < 0){
    Mr = 0;
  }
  if (pickup == 1 || myRobot.Mode == 3 || myRobot.Mode == 1){ //check if robot was pickup or not
    Ml = 0;
    Mr = 0;
  }
  analogWrite(motorRfow,Mr);
  analogWrite(motorRbac,0);
  analogWrite(motorLfow,Ml);
  analogWrite(motorLbac,0);
}

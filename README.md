![Car](https://user-images.githubusercontent.com/32926144/204031427-6d897c76-a962-4154-ac50-8f55f074ad9d.jpg)

In this article, we are going to Control the Robot Car through the G sensor of our mobile phone and you will be able to move the Robot just by tilting the Phone. We will also use Arduino and RemoteXY app for this G-Sensor Controlled Robot. RemoteXY app is used to create the interface in the Smart Phone for controlling the Robot. We will add the joystick in the interface so that Robot can also be controlled by Joystick as well as by tilting the phone.

**Circuit Diagram**

![Circuit Diagram](https://user-images.githubusercontent.com/32926144/204031560-42b42916-6c35-4b6d-8d1e-b46d772b7c70.png)

G-Sensor or Gravity sensor is basically Accelerometer in Smart phone which is used to control the screen orientation of the phone. Accelerometer senses the X,Y, Z directions of the Gravitational force and rotate the Screen according to alignment of the Phone. Now days, more sensitive and accurate Gyroscope sensor is used in mobiles for deciding the orientation of the Screen. In our Project, Robot car will move, according to the direction in which phone is being tilted, like when we tilt the phone forward, then car will move forward and we tilt it down then car will move backward. This is same like when we play some car games in Mobile, they also use G sensor to move the car accordingly.

**Required Components:**
Two wheel robot car chassis
Arduino UNO *L298N Motor Controller
HC-06 Bluetooth module (HC-05 will work too)
Power supply or Cells
Connecting wires

**Creating Interface for Robot using RemoteXY app:**
For creating the interface to control the Robot Car using RemoteXY app, you will have to go to following link http://remotexy.com/en/editor

The webpage will look like this:


![Interface](https://user-images.githubusercontent.com/32926144/204031615-55e772ef-b40f-4ba7-a536-8cd074144c1a.jpg)

**CODE**

#define REMOTEXY_MODE__SOFTWARESERIAL  
#include <SoftwareSerial.h>             //Including the software serial library
#include <RemoteXY.h>                   //Including the remotexy library

/* RemoteXY connection settings */ 
#define REMOTEXY_SERIAL_RX 2            //defining the pin 2 as RX pin
#define REMOTEXY_SERIAL_TX 3            //defining the pin 3 as TX pin
#define REMOTEXY_SERIAL_SPEED 9600      //setting baudrate at 9600

unsigned char RemoteXY_CONF[] =       //remotexy configuration
  { 3,0,23,0,1,5,5,15,41,11 
  ,43,43,1,2,0,6,5,27,11,5 
  ,79,78,0,79,70,70,0 };  
    
struct {                            //Function for declaring the variables
  signed char joystick_1_x;         //joystick x-axis 
  signed char joystick_1_y;         //joystick y-axis
  unsigned char switch_1;           //variables for switch
  unsigned char connect_flag; 

} RemoteXY;  

//defining the pins for first motor
#define IN1 10 
#define IN2 9 
#define ENA 12 

//defining the pins for second motor
#define IN3 8 
#define IN4 7 
#define ENB 11 

//defining the LED pin  
#define ledpin 13 

unsigned char first_motor[3] =  
  {IN1, IN2, ENA}; 
unsigned char second_motor[3] =  
  {IN3, IN4, ENB}; 

void Speed (unsigned char * pointer, int motor_speed) 
{ 
  if (motor_speed>100) motor_speed=100; 
  if (motor_speed<-100) motor_speed=-100; 
  if (motor_speed>0) { 
    digitalWrite(pointer[0], HIGH); 
    digitalWrite(pointer[1], LOW); 
    analogWrite(pointer[2], motor_speed*2.55); 
  } 
  else if (motor_speed<0) { 
    digitalWrite(pointer[0], LOW); 
    digitalWrite(pointer[1], HIGH); 
    analogWrite(pointer[2], (-motor_speed)*2.55); 
  } 
  else { 
    digitalWrite(pointer[0], LOW); 
    digitalWrite(pointer[1], LOW); 
    analogWrite(pointer[2], 0); 
  } 
} 

void setup() 
{ 
  //defining the motor pins as the output pins
  pinMode (IN1, OUTPUT); 
  pinMode (IN2, OUTPUT); 
  pinMode (IN3, OUTPUT); 
  pinMode (IN4, OUTPUT); 
  pinMode (ledpin, OUTPUT); 
  RemoteXY_Init (); 
} 

void loop() 
{ 
  RemoteXY_Handler (); 
  digitalWrite (ledpin, (RemoteXY.switch_1==0)?LOW:HIGH);  
  Speed (first_motor, RemoteXY.joystick_1_y - RemoteXY.joystick_1_x); 
  Speed (second_motor, RemoteXY.joystick_1_y + RemoteXY.joystick_1_x); 
} 

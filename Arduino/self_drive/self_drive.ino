#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/String.h>
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

//copy these two lines to terminal:
//rosrun rosserial_python serial_node.py /dev/ttyACM0
//rosrun teleop_twist_keyboard teleop_twist_keyboard.py



SoftwareSerial SWSerial(NOT_A_PIN, 11); // RX on no pin (unused), TX on pin 11 (to S1 of motor driver).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

SoftwareSerial portROS(0, 1);


int pulPin = 3; // Pulse Pin for steering motor controller
int dirPin = 5; // Direction Pin for steering motor controller
int enblPin = 6; // Enbl Pin for steering motor controller
int ledPin = 13; // LED


const long CONTROL_TIMEOUT = 1000; //ms to wait  before killing motors

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdVelCallback);

//geometry_msgs::Twist msg;
//ros::Publisher chatter_pub("cmd_vel", &msg);


unsigned long lastData = 0;

int cnt=0;

void cmdVelCallback(const geometry_msgs::Twist & msg)  //Read the message. Act accordingly.
{
  if (cnt%2==0)  
    digitalWrite(2, HIGH); 
  else                        
    digitalWrite(2, LOW); 

  cnt=cnt+1;
    
  // this function is called as soon as there is any cmd_vel command in the ros network
  
  lastData = millis();
  //we only care abot the linear x, and the rotational z.
  // this variable is given forward/back velocity in range of [-0.5 +0.5]
  const float linear = msg.linear.x;
  // this variable is given left/right velocity in range of [-1 +1]
  const float spin = msg.angular.z;

  // give movement to forward/back direction
  if (linear > 0)
  {
    forward(int(66*linear));
  }
  else if (linear < 0)
  {
    backward(int(66*linear));
  }
  

  // if given command is right means spin is negative
  if (spin<0)
  {    
    turnRight(int(-50*spin));       
  }
  else if (spin>0)
  {
    turnLeft(int(50*spin)); 
  }
  
}


void setup() {

  pinMode(2, OUTPUT);  

  SWSerial.begin(9600);
  portROS.begin(57600);
  
  
  
  pinMode(pulPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enblPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(pulPin, LOW);
  digitalWrite(ledPin, LOW);
  digitalWrite(enblPin, LOW);
  digitalWrite(dirPin, LOW);   
    
 
  digitalWrite(enblPin, HIGH);
  delay(100);
  digitalWrite(enblPin, LOW);  
  
  nh.initNode();  
  nh.subscribe(sub);
  //nh.advertise(chatter_pub);

}


void loop() 
{  
  nh.spinOnce();

  if(millis() - lastData >= CONTROL_TIMEOUT)
  {
    lastData=millis();
    msg.linear.x = 0;
    msg.angular.z = 0;    
    //chatter_pub.publish(&msg);
  }  

}


//Motor control functions: forward, backward, turn left, turn right-----------------------------------
int turnLeft(int speed) {
  ST.motor(1,-speed);
  ST.motor(2,speed);
  //for(int i=0; i <= duration; i++){
    //leftSpeed(speed);
  //}
}
int turnRight(int speed) {
  ST.motor(1,speed);
  ST.motor(2,-speed);
   //for(int i=0; i <= duration; i++){
    //rightSpeed(speed);
  //} 
}

int forward(int fwd) {
  ST.motor(1,fwd);
  ST.motor(2,fwd);
}

int reverse(int back) {
  ST.motor(1,-back);
  ST.motor(2,-back);
}

void idle() {
  ST.motor(1,0);
}

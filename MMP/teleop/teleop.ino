#include "Motor.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include <std_msgs/Int32.h>
#include <ros/time.h>


ros::NodeHandle  nh;
int updatenh = 0;

#define LOOPTIME 10

Motor right(10,11,18,19);
Motor left(8,9,20,21);

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

double left_kp = 1.3 , left_ki = 0 , left_kd = 0;             // modify for optimal performance
double right_kp = 1.1 , right_ki = 0 , right_kd = 0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

float demandx=0;
float demandz=0;

double demand_speed_left;
double demand_speed_right;

unsigned long currentMillis;
unsigned long prevMillis;

float encoder0Diff;
float encoder1Diff;

float encoder0Prev;
float encoder1Prev; 

float encoder0Error;
float encoder1Error;


void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );
std_msgs::Int32 left_wheel_msg;                                         //create a "speed_msg" ROS message
ros::Publisher left_wheel_pub("lwheel", &left_wheel_msg);
std_msgs::Int32 right_wheel_msg;
ros::Publisher right_wheel_pub("rwheel", &right_wheel_msg);

double pos_act_left = 0;                    //Actual speed for left wheel in m/s
double pos_act_right = 0;                    //Command speed for left wheel in m/s 

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(right_wheel_pub);
  nh.advertise(left_wheel_pub);                  //prepare to publish speed in ROS topic
  //Serial.begin(115200);
  
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);
  
  //Serial.println("Basic Encoder Test:");
  attachInterrupt(digitalPinToInterrupt(left.en_a), change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left.en_b), change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_a), change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_b), change_right_b, CHANGE);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= LOOPTIME){
    prevMillis = currentMillis;

    demand_speed_left = demandx - (demandz*0.070);
    demand_speed_right = demandx + (demandz*0.070);
 
    encoder0Diff = encoder0Pos - encoder0Prev; 
    encoder1Diff = encoder1Pos - encoder1Prev;
    
    pos_act_left = encoder0Pos;                    
    pos_act_right = encoder1Pos; 
  
    encoder0Error = (demand_speed_left*357.02)-encoder0Diff; 
    encoder1Error = (demand_speed_right*357.02)-encoder1Diff;
  
    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
  
    left_setpoint = demand_speed_left*357.02;  
    right_setpoint = demand_speed_right*357.02;
  
    left_input = encoder0Diff; 
    right_input = encoder1Diff;
    
    leftPID.Compute();
    left.rotate(left_output);
    rightPID.Compute();
    right.rotate(right_output);
  //  Serial.print(encoder0Pos);
  //  Serial.print(",");
  //  Serial.println(encoder1Pos);
  }
  publishSpeed(LOOPTIME);
  if(updatenh>10){
    nh.spinOnce();
    updatenh = 0;
  } 
  else{
    updatenh++; 
    }
}


void publishSpeed(double time) {
  left_wheel_msg.data = pos_act_left;
  right_wheel_msg.data = pos_act_right;
  left_wheel_pub.publish(&left_wheel_msg);
  right_wheel_pub.publish(&right_wheel_msg);
  
  
}


void change_left_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(left.en_a) == HIGH) { 
    if (digitalRead(left.en_b) == LOW) {  
      encoder0Pos = encoder0Pos + 1;       
    } 
    else {
      encoder0Pos = encoder0Pos - 1;
    }
  }
  else                                       
  { 
      
    if (digitalRead(left.en_b) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          
    } 
    else {
      encoder0Pos = encoder0Pos - 1;       
    }
  }
 
}

void change_left_b(){  

  if (digitalRead(left.en_b) == HIGH) {   
    if (digitalRead(left.en_a) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         
    } 
    else {
      encoder0Pos = encoder0Pos - 1;        
    }
  }
  // Look for a high-to-low on channel B
  else { 
    if (digitalRead(left.en_a) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          
    } 
    else {
      encoder0Pos = encoder0Pos - 1;        
    }
  }
  

}



void change_right_a(){  

  if (digitalRead(right.en_a) == HIGH) { 
    if (digitalRead(right.en_b) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         
    }
  }
  else                                        
  { 
    if (digitalRead(right.en_b) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;         
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          
    }
  }
 
}

void change_right_b(){  

  if (digitalRead(right.en_b) == HIGH) {   
    if (digitalRead(right.en_a) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         
    }
  }
  else { 
    if (digitalRead(right.en_a) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         
    }
  }
  

}

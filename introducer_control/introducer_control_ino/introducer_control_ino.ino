/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <AFMotor.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

// change this to the number of steps on your motor
#define STEPS 100

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
AF_Stepper introducer(STEPS, 2);

// the previous reading from the analog input
int previous = 0;

void introducer_cb( const std_msgs::UInt16& cmd_msg){
  if (cmd_msg.data == 0)
    introducer.step(20, BACKWARD, MICROSTEP); //set servo angle, should be from 0-180  
  else
    introducer.step(20, FORWARD, MICROSTEP); //set servo angle, should be from 0-180  

    
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::UInt16> sub("introducer", introducer_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
}

void loop(){
  nh.spinOnce();
  delay(1);
}

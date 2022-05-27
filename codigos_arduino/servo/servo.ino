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
 
  #if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include <WProgram.h>
  #endif
  
  #include <Servo.h> 
  #include <ros.h>
  #include <std_msgs/Int16.h>
   
  ros::NodeHandle  nh;
  
  Servo servo_ho;
  Servo servo_ve;
  
  void servo_cb_h( const std_msgs::Int16& cmd_msg_h){
    servo_ho.write(cmd_msg_h.data); //set servo angle, should be from 0-180  
    //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  }
  void servo_cb_v( const std_msgs::Int16& cmd_msg_v){
    servo_ve.write(cmd_msg_v.data); //set servo angle, should be from 0-180  
    //digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
  }
  
  ros::Subscriber<std_msgs::Int16> Mov_ho("Mov_ho", servo_cb_h);
  ros::Subscriber<std_msgs::Int16> Mov_ve("Mov_ve", servo_cb_v);
  
  void setup(){
    pinMode(13, OUTPUT);
  
    nh.initNode();
    nh.subscribe(Mov_ho);
    nh.subscribe(Mov_ve);
    
    servo_ho.attach(A0); //attach it to pin 9
    servo_ve.attach(A1);
  }
  
  void loop(){
    nh.spinOnce();
    delay(1);
  }

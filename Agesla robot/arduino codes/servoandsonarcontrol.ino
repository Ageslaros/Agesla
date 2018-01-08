#include <ros.h> // Robot Operating System (ROS) official library
#include <ArduinoHardware.h> // Arduino library for non Arduino IDE compatibility
#include <std_msgs/Int32.h> // Integer messages for ROS
#include <NewPing.h> // Library for HC-SR04 https://playground.arduino.cc/Code/NewPing
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <std_msgs/UInt16.h>

// HC-SR04 values
#define TRIGGER_PIN 9 
#define ECHO_PIN 10
#define MAX_DISTANCE 400 // Max distance in centimeters.

// Declare a sonar Object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Servo servo;


void servo_cb( const std_msgs::UInt16& cmd_msg){
  int pos=0;
  for (pos = 0; pos <= cmd_msg.data; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = cmd_msg.data; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


// Setup ros node and publisher
std_msgs::Int32 sonar_msg;
ros::Publisher pub_sonar("sonar", &sonar_msg);
ros::Subscriber<std_msgs::UInt16> sub("servo", servo_cb);
ros::NodeHandle nh;

int SetDistance = 0;
int cm_distance = 0;

void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(pub_sonar);
  nh.subscribe(sub);
  servo.attach(6); //attach it to pin 6
}

void loop() {
  delay(50);
  unsigned int micro_seconds = sonar.ping();
  cm_distance = micro_seconds / US_ROUNDTRIP_CM;

  sonar_msg.data = cm_distance;
  pub_sonar.publish(&sonar_msg); // Publishes data on 
  //Serial.print("Range= ");
  Serial.print(cm_distance);
  Serial.print("\n");

  nh.spinOnce();
  
}

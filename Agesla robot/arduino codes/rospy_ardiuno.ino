#include <ros.h> // Robot Operating System (ROS) official library
#include <ArduinoHardware.h> // Arduino library for non Arduino IDE compatibility
#include <std_msgs/Int32.h> // Integer messages for ROS
#include <NewPing.h> // Library for HC-SR04 https://playground.arduino.cc/Code/NewPing

// HC-SR04 values
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 400 // Max distance in centimeters.

// Declare a sonar Object
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
void messageCb(const std_msgs::Int32 & msg)
{
  int Sonar_value = msg.data;
  digitalWrite(12, LOW);

  if (Sonar_value > 30)
  {
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    
    delay(20);
  }
  else if (Sonar_value > 20)
  {
    digitalWrite(12, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(5, LOW);
    
    
  }
  else if (Sonar_value > 0)
  {
    digitalWrite(12, HIGH);
    digitalWrite(5, HIGH);
    digitalWrite(4, LOW);
    digitalWrite(3, LOW);
    
  }
  else
  {
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
    digitalWrite(5, LOW);
    digitalWrite(12, LOW);

  }

}
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int32> sub_sender("sender", &messageCb);
// Setup ros node and publisher
std_msgs::Int32 sonar_msg;
ros::Publisher pub_sonar("sonar", &sonar_msg);


int SetDistance = 0;
int cm_distance = 0;

void setup() {
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(12, OUTPUT);
  nh.initNode();
  nh.advertise(pub_sonar);
  nh.subscribe(sub_sender);
}

void loop() {
  delay(50);
  unsigned int micro_seconds = sonar.ping();
  cm_distance = micro_seconds / US_ROUNDTRIP_CM;

  sonar_msg.data = cm_distance;
  pub_sonar.publish(&sonar_msg); // Publishes data on

  nh.spinOnce();

}

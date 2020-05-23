/*
 * roscore
 * rosrun rosserial_python serial_node.py /dev/ttyACM0
 * rostopic echo range
 */

#include <ros.h>
#include <std_msgs/Float64.h>

std_msgs::Float64 range_msg;
ros::Publisher pub_range("range", &range_msg);
ros::NodeHandle nh;

long publisher_timer;

const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// unused max and min distance
const unsigned int MAX_DIST = 23200; // 400 cm
const unsigned int MIN_DIST = 116; // 2 cm

void setup() {
  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Set Echo pin as input to measure the duration of pulses coming back from the distance sensor
  pinMode(ECHO_PIN, INPUT);

  nh.initNode();
  nh.advertise(pub_range);
}

// publish once a second
void loop() {
  if(millis() > publisher_timer) {
    range_msg.data = getRange();
    pub_range.publish(&range_msg);
    publisher_timer = millis() + 4000;
  }
  nh.spinOnce();
}

float getRange() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;

  // Hold the trigger pin for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  while(digitalRead(ECHO_PIN) == 0);

  // Measure how long the echo pin was held high (pulse width); the micros() counter will overflow after ~70 min
  t1 = micros();
  while(digitalRead(ECHO_PIN) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters
  cm = (pulse_width / 58.0);
  return cm;
}

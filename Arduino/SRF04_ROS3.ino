/*
 * roscore
 * rosrun rosserial_python serial_node.py /dev/ttyACM0
 * rostopic echo rangeMsg_topic
 * 
 * Erster Versuch mit Range Message
 * 2 unterschiedliche Loop Varianten
 */

#include <ros.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;
sensor_msgs::Range rangeMsg;
ros::Publisher pub_range("rangeMsg_topic", &rangeMsg);

const int trigPin = 9;
const int echoPin = 10;

long publisher_timer;
long duration;
float distance;

char frameid[] = "/sr04_ranger";

void setup() {
  nh.initNode();
  nh.advertise(pub_range);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  rangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  rangeMsg.header.frame_id = frameid;
  rangeMsg.field_of_view = 0.523599f; // Größe des Bogens, für den die Entfernungsmessung gültig ist; 30 Grad im Winkel
  rangeMsg.min_range = 0.05; // Meters
  rangeMsg.max_range = 2; // Meters
}

/*void loop() {
  if(millis() > publisher_timer) {
    digitalWrite(trigPin, LOW);
      delayMicroseconds(2);

      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      duration = pulseIn(echoPin, HIGH);
      distance = duration * 0.034/2;

      rangeMsg.range = distance / 100;
      rangeMsg.header.stamp = nh.now();
      pub_range.publish(&rangeMsg);

      publisher_timer = millis() + 4000;
  }
  nh.spinOnce();
}*/ 

void loop() {
  if(millis() > publisher_timer) {
    // Hinweis: Werte < range_min oder > range_max sollten verworfen werden
    rangeMsg.range = getRange() / 100;
    pub_range.publish(&rangeMsg);
    publisher_timer = millis() + 4000;
  }
  nh.spinOnce();
}

float getRange() {
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;

  // Halte den Trigger Pin für 10 Mikrosekunden
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Wait for pulse on echo pin
  while(digitalRead(echoPin) == 0);

  // Messe wie lange Echo Pin den Status HIGH war
  t1 = micros();
  while(digitalRead(echoPin) == 1);
  t2 = micros();
  pulse_width = t2 - t1;

  // Berechne die Distanz in Zentimeter
  cm = (pulse_width / 58.0);
  return cm;
}

/*
 * 2. Versuch mit Range Message.
 * Diese Mal unter der Verwendung der NewPing Bibliothek
*/

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
// // http://playground.arduino.cc/Code/NewPing
#include <NewPing.h>

#define TRIG_PIN 9 // Trigger Output
#define ECHO_PIN 10 // Echo Input
#define MAX_DIST 400

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST);

long publisher_timer;

char frameid[] = "/sr04_ranger";

ros::NodeHandle nh;
sensor_msgs::Range rangeMsg;
ros::Publisher pub_range("rangeMsg_topic", &rangeMsg);

void setup() {
  nh.initNode();
  nh.advertise(pub_range);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  rangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  rangeMsg.header.frame_id = frameid;
  rangeMsg.field_of_view = 0.523599f; // Größe des Bogens, für den die Entfernungsmessung gültig ist; 30 Grad im Winkel
                                      // 30 Grad wurde aus einem Beispiel entnommen
  rangeMsg.min_range = 0.05; // Meters
  rangeMsg.max_range = 4; // Meters
}

void loop() {
  if(millis() > publisher_timer) {
    rangeMsg.range = (float) sonar.ping_cm() / 100; // ping_cm() return int
    rangeMsg.header.stamp = nh.now();
    pub_range.publish(&rangeMsg);
    publisher_timer = millis() + 4000;
  }
  nh.spinOnce();
}

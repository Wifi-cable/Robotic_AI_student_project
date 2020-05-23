#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;
std_msgs::Float64 distance;
ros::Publisher pub_range("range", &distance);

const int trigPin = 9;
const int echoPin = 10;

long publisher_timer;
long duration;
float range;


void setup() {
  nh.initNode();
  nh.advertise(pub_range);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  if(micros() > publisher_timer) {
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);

      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      duration = pulseIn(echoPin, HIGH);

      range = duration * 0.034/2;

      distance.data = range;
      pub_range.publish(&distance);
      publisher_timer = millis() + 4000;
  }
  nh.spinOnce();
}

#include <ros.h>

volatile byte state2 = LOW;
volatile byte state3 = LOW;
volatile byte state4 = LOW;
volatile bool flag = false;

ros::NodeHandle  nh;

void cl_pb2(){
  state2 = !state2;
  flag = true;
}

void cl_pb3(){
  state3 = !state3;
  flag = true;
}

void cl_pb4(){
  state4 = !state4;
  flag = true;
}

void setup() {
  Serial.begin(115200);
  pinMode(PB2, INPUT_PULLUP);
  pinMode(PB3, INPUT_PULLUP);
  pinMode(PB4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PB2), cl_pb2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PB3), cl_pb3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PB4), cl_pb4, CHANGE);
}

void loop() {
  if (flag){
    Serial.print("state pb2: ");
    Serial.println(state2);

    Serial.print("state pb3: ");
    Serial.println(state3);
    
    Serial.print("state pb4: ");
    Serial.println(state4);
  }
  else {
    flag = false;
    Serial.println("helo world");
  }
}

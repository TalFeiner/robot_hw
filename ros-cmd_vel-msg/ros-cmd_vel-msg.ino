#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

const int right_motor = 9, left_motor = 10;
const int direction_right = 4, direction_left = 3;
const int brackPin = 2;
const int buttonPin = 8;
const int PWM_linear = 5, PWM_angular = 6;
bool direc_left_old = true;
bool direc_right_old = true;
float pwm_linear, pwm_angular;

ros::NodeHandle  nh;

void messageCb_brack( const std_msgs::Bool& brack_msg){
  if (brack_msg.data){
    digitalWrite(brackPin, HIGH);
  }
  else{
    digitalWrite(brackPin, LOW);
  }
}

void messageCb_r( const std_msgs::Int8& vel){
  if (vel.data >= 0){
    digitalWrite(direction_right, LOW);
  }
  else{
    digitalWrite(direction_right, HIGH);
  }
  analogWrite(right_motor, vel.data);
}

void messageCb_l( const std_msgs::Int8& vel){
  if (vel.data >= 0){
    digitalWrite(direction_left, LOW);
  }
  else{
    digitalWrite(direction_left, HIGH);
  }
  analogWrite(right_motor, vel.data);
  analogWrite(left_motor, vel.data);
}

ros::Subscriber<std_msgs::Bool> sub_brack("arduino_brack", &messageCb_brack );
ros::Subscriber<std_msgs::Int8> sub_vel_right("vel_left", &messageCb_r );
ros::Subscriber<std_msgs::Int8> sub_vel_left("vel_right", &messageCb_l );

void setup() {
  //pinMode(brackPin, OUTPUT);
  //pinMode(right_motor, OUTPUT);
  //pinMode(left_motor, OUTPUT);
  pinMode(direction_right, OUTPUT);
  pinMode(direction_left, OUTPUT);
  //pinMode(buttonPin, INPUT);
  //pinMode(PWM_linear, INPUT);
  //pinMode(PWM_angular, INPUT);
  //analogWrite(right_motor, 0);
  //analogWrite(left_motor, 0);
  if (digitalRead(buttonPin) == LOW){
  nh.initNode();
  nh.subscribe(sub_vel_left);
  nh.subscribe(sub_vel_right);
  }
}


void drive(int left_pwm, int right_pwm)
{
  bool direc_left;
  bool direc_right;
  if (left_pwm > 0) {
    direc_left = true;
  }
  else {
    direc_left = false;
  }
  if (left_pwm > 0) {
    direc_right = true;
  }
  else {
    direc_right = false;
  }
  if (direc_left_old == direc_left) {
    if(left_pwm < 0)
    {
      left_pwm = -left_pwm;
      direc_left_old = false;
      digitalWrite(direction_left, HIGH);
    }
    else
    {
      direc_left_old = true;
      digitalWrite(direction_left, LOW);
    }
  }
  else {
    digitalWrite(brackPin, HIGH);
    if(left_pwm < 0)
    {
      left_pwm = -left_pwm;
      direc_left_old = false;
      digitalWrite(direction_left, HIGH);
    }
    else
    {
      direc_left_old = true;
      digitalWrite(direction_left, LOW);
    }
  }
  if (direc_right_old == direc_right) {
    if(right_pwm < 0)
    {
      direc_right_old = false;
      right_pwm = -right_pwm;
      digitalWrite(direction_right, LOW);
    }
    else
    {
      direc_right_old = true;
      digitalWrite(direction_right, HIGH);
    }
  }
  else {
    digitalWrite(brackPin, HIGH);
    if(right_pwm < 0)
    {
      direc_right_old = false;
      right_pwm = -right_pwm;
      digitalWrite(direction_right, LOW);
    }
    else
    {
      direc_right_old = true;
      digitalWrite(direction_right, HIGH);
    }
  }
    digitalWrite(brackPin, LOW);
    analogWrite(left_motor,(int)left_pwm);
    analogWrite(right_motor, (int)right_pwm);

  
}
void loop() {
  if (digitalRead(buttonPin) == HIGH){
    pwm_linear = (float)pulseIn(PWM_linear, HIGH);
    pwm_angular = (float)pulseIn(PWM_angular, HIGH);

    
    pwm_linear = ((pwm_linear - 1500) / 400) * 255;
    pwm_angular = ((pwm_angular - 1500) / 400) * 255;
    if (pwm_linear > 255) pwm_linear = 255;
    if (pwm_linear < -255) pwm_linear = -255;
    if (pwm_angular > 255) pwm_angular = 255;
    if (pwm_angular < -255) pwm_angular = -255;
    float pwm_right = (pwm_linear + pwm_angular) / 2;
    float pwm_left = (pwm_linear - pwm_angular) / 2;
    drive(pwm_left, pwm_right);    
    
  }
  else
  {
  nh.spinOnce();
  }
  
  //delay(1);
}

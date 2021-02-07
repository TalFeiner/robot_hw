#include <ros.h>
#include <std_msgs/Int8.h>
//#include <std_msgs/Bool.h>
//#include <geometry_msgs/Twist.h>

const int right_motor = 3, left_motor = 9;
const int direction_right = 17, direction_left = 4;
const int brackPin_right = 16, brackPin_left = 7;
const int buttonPin = 14;
const int PWM_linear = 10, PWM_angular = 11;
float pwm_linear, pwm_angular;
float cmd_left, cmd_right;
bool direc_left_old = true;
bool direc_right_old = true;
int minPWM = 15;
//int cmdVelLinear = 0.0, cmdVelAngular = 0.0;
//long tOld = millis(); 
//float linearErrorOld = 0.0, angularErrorOld = 0.0;
//float linearI = 0.0, angularI = 0.0;
//float Kp = 100, Ki = 0.7, Kd = 0.3;
//float linearVel = 0.1, angularVel = 0.1; 

ros::NodeHandle  nh;

//void messageCb_brack( const std_msgs::Bool& brack_msg){
//  if (brack_msg.data){
//    digitalWrite(brackPin_right, HIGH);
//    digitalWrite(brackPin_left, HIGH);
//  }
//  else{
//    digitalWrite(brackPin_right, LOW);
//    digitalWrite(brackPin_left, LOW);
//  }
//}

void velCmdLeft( const std_msgs::Int8& velL){
  if(velL.data==-128) {
    digitalWrite(brackPin_right, HIGH);
    digitalWrite(brackPin_left, HIGH);
  }
  cmd_left = velL.data * 2;
}

void velCmdRight( const std_msgs::Int8& velR){
  if(velR.data==-128) {
    digitalWrite(brackPin_right, LOW);
    digitalWrite(brackPin_left, LOW);
  }
  cmd_right = velR.data * 2;
}

//void velLinear( const std_msgs::Int8& vel){
//  linearVel = vel.data;
//}
//
//void velAngular( const std_msgs::Int8& vel){
//  angularVel = vel.data;
//}

//void velCB( const geometry_msgs::Twist& vel){
//  cmdVelLinear = vel.linear.x;
//  cmdVelAngular = vel.angular.z;
//}

//ros::Subscriber<std_msgs::Bool> sub_brack("/blattoidea/brake", &messageCb_brack );
//ros::Subscriber<geometry_msgs::Twist> subVel("/blattoidea/cmd_vel", &velCB );   // PWM velocity values
ros::Subscriber<std_msgs::Int8> subCmdVelLeft("/blattoidea/cmd_left", &velCmdLeft );
ros::Subscriber<std_msgs::Int8> subCmdVelRight("/blattoidea/cmd_right", &velCmdRight );
//ros::Subscriber<std_msgs::Int8> subVelLinear("/blattoidea/vel_linear", &velLinear );
//ros::Subscriber<std_msgs::Int8> subVelAngular("/blattoidea/vel_angular", &velAngular );

void setup() {
//  pinMode(brackPin_right, OUTPUT);
//  pinMode(brackPin_left, OUTPUT);
  //pinMode(right_motor, OUTPUT);
  //pinMode(left_motor, OUTPUT);
  pinMode(direction_right, OUTPUT);
  pinMode(direction_left, OUTPUT);
//  pinMode(buttonPin, INPUT);
  //pinMode(PWM_linear, INPUT);
  //pinMode(PWM_angular, INPUT);
  //analogWrite(right_motor, 0);
  //analogWrite(left_motor, 0);
  if (digitalRead(buttonPin) == LOW){
    nh.initNode();
    nh.subscribe(subCmdVelLeft);
    nh.subscribe(subCmdVelRight);
//    nh.subscribe(subVelLinear);
//    nh.subscribe(subVelAngular);
//    nh.subscribe(subVel);
//    nh.subscribe(sub_brack);
  }
  else{
    Serial.begin(57600);
    }
}


void drive  (int left_pwm, int right_pwm) {
  if ((abs(left_pwm) > minPWM) || (abs(right_pwm) > minPWM))  {
    bool direc_left;
    bool direc_right;
    
    if (left_pwm > 0) {
      direc_left = true;
    }
    else {
      direc_left = false;
    }
    if (right_pwm > 0) {
      direc_right = true;
    }
    else {
      direc_right = false;
    }
  
    if (direc_left != direc_left_old) {
      digitalWrite(brackPin_left, HIGH);
      direc_left_old = direc_left;
      delay(1);
//      Serial.println("left");
    }
    if (direc_right != direc_right_old) {
      digitalWrite(brackPin_right, HIGH);
      direc_right_old = direc_right;
//      Serial.println("right");
      delay(1);
    }
//    Serial.println(((String)pwm_linear)+";"+((String)pwm_angular)+";"+((String)direc_left)+";"+((String)direc_right));
    
    if  (left_pwm < 0)  {
      left_pwm = -left_pwm;
      digitalWrite(direction_left, HIGH);
    }
    else  {
      digitalWrite(direction_left, LOW);
    }
    
    if  (right_pwm < 0) {
      right_pwm = -right_pwm;
      digitalWrite(direction_right, LOW);
    }
    else  {
      digitalWrite(direction_right, HIGH);
    }
  
    digitalWrite(brackPin_right, LOW);
    digitalWrite(brackPin_left, LOW);
    analogWrite(left_motor,(int)left_pwm);
    analogWrite(right_motor, (int)right_pwm);  
  }
  else {
    digitalWrite(brackPin_right, LOW);
    digitalWrite(brackPin_left, LOW);
    analogWrite(left_motor,(int)0);
    analogWrite(right_motor, (int)0);
  }
}

void loop() {
  if (digitalRead(buttonPin) == HIGH) {
    pwm_linear = (float)pulseIn(PWM_linear, HIGH);
    pwm_angular = (float)pulseIn(PWM_angular, HIGH);
        
    if (pwm_linear == 0 || pwm_angular == 0){
      pwm_linear = 0;
      pwm_angular = 0;
    }
    else{
      pwm_linear = ((pwm_linear - 1500) / 200) * 255;
      pwm_angular = ((pwm_angular - 1500) / 200) * 255;
      if (pwm_linear > 255) pwm_linear = 255;
      if (pwm_linear < -255) pwm_linear = -255;
      if (pwm_angular > 255) pwm_angular = 255;
      if (pwm_angular < -255) pwm_angular = -255;
    }
    
    float pwm_right = (pwm_linear + pwm_angular) / 2;
    float pwm_left = (pwm_linear - pwm_angular) / 2;
    drive(pwm_left, pwm_right);    
    
  }
  else  {
//    float linearError = linearVel - cmdVelLinear; 
//    float angularError = angularVel - cmdVelAngular;
//    linearI += linearError; 
//    angularI += angularError;
//    float dt = (tOld - millis()); 
//    float linearD = (linearError - linearErrorOld) / dt;
//    float angularD = (angularError - angularErrorOld) / dt;
//    pwm_linear = Kp * linearError + Ki * linearI + Kd * linearD;
//    pwm_angular = Kp * angularError + Ki * angularI + Kd * angularD;
//    
    if (cmd_left > 255) cmd_left = 255;
    if (cmd_left < -255) cmd_left = -255;
    if (cmd_right > 255) cmd_right = 255;
    if (cmd_right < -255) cmd_right = -255;
    drive(cmd_left, cmd_right);
    
    nh.spinOnce();
  }
  delay(1);
}

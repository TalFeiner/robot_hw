#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
//#include <geometry_msgs/Twist.h>

const int right_motor = 9, left_motor = 10;
const int direction_right = 4, direction_left = 3;
const int brackPin = 2;
const int buttonPin = 8;
const int PWM_linear = 5, PWM_angular = 6;
float pwm_linear, pwm_angular;
bool direc_left_old = true;
bool direc_right_old = true;
unsigned int minVel = 10;
//int cmdVelLinear = 0.0, cmdVelAngular = 0.0;
//long tOld = millis();
//float linearErrorOld = 0.0, angularErrorOld = 0.0;
//float linearI = 0.0, angularI = 0.0;
//float Kp = 100, Ki = 0.7, Kd = 0.3;
//float linearVel = 0.1, angularVel = 0.1;

ros::NodeHandle  nh;

void messageCb_brack( const std_msgs::Bool& brack_msg) {
  if (brack_msg.data) {
    digitalWrite(brackPin, HIGH);
  }
  else {
    digitalWrite(brackPin, LOW);
  }
}

void velCmdAngular( const std_msgs::Int8& vel) {
  pwm_angular = vel.data;
}

void velCmdLinear( const std_msgs::Int8& vel) {
  pwm_linear = vel.data;
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

ros::Subscriber<std_msgs::Bool> sub_brack("/blattoidea/brake", &messageCb_brack );
//ros::Subscriber<geometry_msgs::Twist> subVel("/blattoidea/cmd_vel", &velCB );   // PWM velocity values
ros::Subscriber<std_msgs::Int8> subCmdVelLinear("/blattoidea/cmd_pwm_linear", &velCmdLinear );
ros::Subscriber<std_msgs::Int8> subCmdVelAngular("/blattoidea/cmd_pwm_angular", &velCmdAngular );
//ros::Subscriber<std_msgs::Int8> subVelLinear("/blattoidea/vel_linear", &velLinear );
//ros::Subscriber<std_msgs::Int8> subVelAngular("/blattoidea/vel_angular", &velAngular );

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
  if (digitalRead(buttonPin) == LOW) {
    nh.initNode();
    nh.subscribe(subCmdVelLinear);
    nh.subscribe(subCmdVelAngular);
    //    nh.subscribe(subVelLinear);
    //    nh.subscribe(subVelAngular);
    //    nh.subscribe(subVel);
    nh.subscribe(sub_brack);
  }
}


void drive  (int left_pwm, int right_pwm) {
  if ((abs(left_pwm) > minVel) || (abs(right_pwm) > minVel))  {
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

    if (direc_left != direc_left_old) {
      digitalWrite(brackPin, HIGH);
      direc_left_old = direc_left;
      delay(100);
    }
    if (direc_right != direc_right_old) {
      digitalWrite(brackPin, HIGH);
      direc_right_old = direc_right;
      delay(100);
    }

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

    digitalWrite(brackPin, LOW);
    analogWrite(left_motor, (int)left_pwm);
    analogWrite(right_motor, (int)right_pwm);
  }
  else {
    digitalWrite(brackPin, HIGH);
    analogWrite(left_motor, (int)0);
    analogWrite(right_motor, (int)0);
  }
}
void loop() {
  if (digitalRead(buttonPin) == HIGH) {
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
    if (pwm_linear > 255) pwm_linear = 255;
    if (pwm_linear < -255) pwm_linear = -255;
    if (pwm_angular > 255) pwm_angular = 255;
    if (pwm_angular < -255) pwm_angular = -255;
    float pwm_right = (pwm_linear + pwm_angular) / 2;
    float pwm_left = (pwm_linear - pwm_angular) / 2;
    drive(pwm_left, pwm_right);

    nh.spinOnce();
  }
  delay(1);
}

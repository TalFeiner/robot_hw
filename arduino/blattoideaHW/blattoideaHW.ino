#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#define CHANNELS 11
#include <Adafruit_MCP4728.h>
#include <Wire.h>
Adafruit_MCP4728 mcp;

const byte debug = 0;
const float durationEncoder = 0.1, maxCmdDuration = 0.5, pidDuration = 0.1;  //  [sec]
const int minVelCmd = 80;
const int velMaxVal = 4000;
const float D = 0.1651, wheelsSeparation = 0.42; //[m]
const int pulsesPerRev = 60;
const int max_rem_val = 1810 ,min_rem_val = 1166;
const int norm_factor = (max_rem_val - min_rem_val) / 2;
const int mid_rem_val = 1470;
const float kp =90, ki = 30;

// serial2 pins- RX: 17, TX: 16
// serial1 pins- RX: 0, TX: 1
// sdaPin 20, sclPin 21;
// Mega interrupt 2, 3, 18, 19, 20, 21
const byte buttonPin = 15;
const byte PWMLinearPin = 45, PWMAngularPin = 46;
const byte directionRightPin = 29, directionLeftPin = 28;
const byte rightBrackPin = 25, leftBrackPin = 24;
const byte encLeftPinA = 2, encLeftPinB = 3;
const byte encRghitPinA = 18, encRghitPinB = 19;

bool direcLeftOld = true;
bool direcRightOld = true;
bool stringComplete = false;  // whether the string is complete
float setPointTemp = 0;

long oldPositionL  = -0, oldPositionR  = -0;
double oldTimeEncoder = 0, last_cmd_time = 0, lastPidTime = 0;
float cmd_motor_left = 0, cmd_motor_right = 0;
float linearCmdVal, angularCmdVal;
String inputString = "";         // a String to hold incoming data
double integralLeft = 0, integralRight = 0;


struct kalman {
   double vel_kf;
   double p_kf;
};

double rleftR_kf = 1.1, leftQ_kf = 0.5;
double rightR_kf = 1.1, rightQ_kf = 0.5;
struct kalman angularVelLKF = {vel_kf: 0.0, p_kf: 1.0};
struct kalman angularVelRKF = {vel_kf: 0.0, p_kf: 1.0};

double dist = 0.0, theta = 0.0, distError = 0.0, thetaError = 0.0, oldDist = 0.0, oldTheta = 0.0, dDist = 0.001, dTheta = 0.01;
String resetError = "false";

Encoder encL(encLeftPinA, encLeftPinB);
Encoder encR(encRghitPinA, encRghitPinB);


struct kalman kalmanFunc(double vel, struct kalman KF, double r_kf, double q_kf){
  KF.p_kf = KF.p_kf + q_kf;
  double y = vel - KF.vel_kf;
  double s = KF.p_kf + r_kf;
  double k = KF.p_kf/s;
  KF.vel_kf = KF.vel_kf + k*y;
  KF.p_kf = (1-k)*KF.p_kf;
  return KF;
}


int pidCalc(float setPoint, double vel, double dt, double integral) {
  bool integralCalc = true;
  if(setPointTemp != 0){
    setPoint = setPointTemp;
    setPointTemp = 0;
    integralCalc = false;
  }
  float error = setPoint - vel;
  if(integralCalc)
    integral += error *  dt;
  int cmd = kp * error + ki * integral;
  if(abs(cmd) > velMaxVal) {
    if(cmd > 0){
      cmd = velMaxVal;
      setPointTemp = velMaxVal;
    }
    else {
      cmd = -velMaxVal;
      setPointTemp = -velMaxVal;
    }
    if(ki != 0) {
      integral = (1 / ki) * (setPointTemp - (kp * error));
    }
    else {
      integralLeft = 0;
    }
  }
  else if(abs(cmd) < minVelCmd) {
    if(cmd > 0){
      cmd = minVelCmd;
      setPointTemp = minVelCmd;
    }
    else {
      cmd = -minVelCmd;
      setPointTemp = -minVelCmd;
    }
    if(ki != 0) {
      integral = (1 / ki) * (setPointTemp - (kp * error));
    }
    else {
      integral = 0;
    }
  }
  return cmd;
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}


String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


double wheelAngularVel(long newPosition, long oldPosition, double dt){
  double dtheta = (double)(2 * PI) * (double)((double)((double)newPosition - (double)oldPosition) / (double)pulsesPerRev);  //  [rad]
  return dtheta/dt;  //  [rad/sec]
}


void drive  (int leftVelCmd, int rightVelCmd) {
  if ((abs(leftVelCmd) > minVelCmd) || (abs(rightVelCmd) > minVelCmd))  {
    bool direcLeft;
    bool direcRight;
    
    if (leftVelCmd > 0) {
      direcLeft = true;
    }
    else {
      direcLeft = false;
    }
    if (rightVelCmd > 0) {
      direcRight = true;
    }
    else {
      direcRight = false;
    }
  
    if (direcLeft != direcLeftOld) {
      digitalWrite(leftBrackPin, HIGH);
      direcLeftOld = direcLeft;
      delay(200);
    }
    if (direcRight != direcRightOld) {
      digitalWrite(rightBrackPin, HIGH);
      direcRightOld = direcRight;
      delay(200);
    }    
    if  (leftVelCmd < 0)  {
      leftVelCmd = -leftVelCmd;
      digitalWrite(directionLeftPin, HIGH);
    }
    else  {
      digitalWrite(directionLeftPin, LOW);
    }
    
    if  (rightVelCmd < 0) {
      rightVelCmd = -rightVelCmd;
      digitalWrite(directionRightPin, LOW);
    }
    else  {
      digitalWrite(directionRightPin, HIGH);
    }
  
    digitalWrite(rightBrackPin, LOW);
    digitalWrite(leftBrackPin, LOW);
    mcp.setChannelValue(MCP4728_CHANNEL_A, (int)leftVelCmd);  //  lfet
    mcp.setChannelValue(MCP4728_CHANNEL_B, (int)rightVelCmd);  //  right
  }
  else {
    digitalWrite(rightBrackPin, HIGH);
    digitalWrite(leftBrackPin, HIGH);
    mcp.setChannelValue(MCP4728_CHANNEL_A, (int)0);  //  lfet
    mcp.setChannelValue(MCP4728_CHANNEL_B, (int)0);  //  right
  }
}


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  // reserve 2 bytes for the inputString:
  inputString.reserve(2);

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial2.println("Adafruit MCP4728 test!");
  // Try to initialize!
  if (!mcp.begin()) {
    Serial2.println("Failed to find MCP4728 chip");
    while (1) {
      delay(10);
    }
  }
  Serial2.println("Found MCP4728 chip");
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 0);
  mcp.setChannelValue(MCP4728_CHANNEL_C, 0);
  mcp.setChannelValue(MCP4728_CHANNEL_D, 0);

  pinMode(directionRightPin, OUTPUT);
  pinMode(directionLeftPin, OUTPUT);

  oldTimeEncoder = millis();
  lastPidTime = millis();
}


void loop() {
  double dt = (millis() - oldTimeEncoder) / 1000;  //  [sec]
  if(dt >= durationEncoder) {
    oldTimeEncoder = millis();

    long newPositionL = encL.read();
    long newPositionR = encR.read();

    double angularVelL = -wheelAngularVel(newPositionL, oldPositionL, dt);  //  [rad/sec]
    double angularVelR = wheelAngularVel(newPositionR, oldPositionR, dt);  //  [rad/sec]
    oldPositionL = newPositionL;
    oldPositionR = newPositionR;

    angularVelLKF = kalmanFunc (angularVelL, angularVelLKF, rleftR_kf, leftQ_kf);
    angularVelRKF = kalmanFunc (angularVelR, angularVelRKF, rightR_kf, rightQ_kf);

    double linearVelL = (D / 2) * angularVelLKF.vel_kf; //[m/sec]
    double linearVelR = (D / 2) * angularVelRKF.vel_kf;  //[m/sec]

    double omega = (linearVelR - linearVelL) / wheelsSeparation; //[rad/sec]
    double linearV = (linearVelR + linearVelL) / 2;  //[m/sec]

    dist = linearV * dt + dist;  //[m]
    theta = omega * dt + theta;  //[rad]

    double x = dist * cos(theta);  //[m]
    double y = dist * sin(theta);  //[m]

    if (debug==1)
    {
      Serial2.println("wheelAngularVel;" + String(angularVelLKF.vel_kf,8) + ";" + String(angularVelRKF.vel_kf,8) + '\n');
    }
    Serial2.print((String)"Odom;" + "Twist;" + "angular;" + String(omega,8) + ";linear;" + String(linearV,8));
    Serial2.println((String)";Pose;" + "x;" + String(x,8) + ";y;" + String(y,8) + ";" + '\n');
  }

  if(digitalRead(buttonPin) == HIGH) {
    linearCmdVal = (float)pulseIn(PWMLinearPin, HIGH);
    angularCmdVal = (float)pulseIn(PWMAngularPin, HIGH);

    if (debug==1)
    {
      Serial2.println("Debug;controllerPWMPin: linearCmdVal - " + (String)(linearCmdVal) + " , angularCmdVal - " + (String)(angularCmdVal));
    }

    if (linearCmdVal == 0 || angularCmdVal == 0){
      linearCmdVal = 0;
      angularCmdVal = 0;
    }
    else {
      linearCmdVal = ((linearCmdVal - mid_rem_val) / norm_factor) * velMaxVal;
      angularCmdVal = ((angularCmdVal - mid_rem_val) / norm_factor) * velMaxVal;
      if (linearCmdVal > velMaxVal) linearCmdVal = velMaxVal;
      if (linearCmdVal < -velMaxVal) linearCmdVal = -velMaxVal;
      if (angularCmdVal > velMaxVal) angularCmdVal = velMaxVal;
      if (angularCmdVal < -velMaxVal) angularCmdVal = -velMaxVal;
    }

    float cmdRight = (linearCmdVal - angularCmdVal) / 2;
    float cmdLeft = (linearCmdVal + angularCmdVal) / 2;

    if (debug==1)
    {
      Serial2.println("Debug;cmd: cmdRight - " + (String)(cmdRight) + " , cmdLeft - " + (String)(cmdLeft));
    }

    drive(cmdLeft, cmdRight);
  }

  else {
    // print the string when a newline arrives:
    if (stringComplete) {
      if (debug==1)
      {
        Serial2.println("Debug;inputString: " + inputString);
      }
      cmd_motor_left = getValue((String)inputString, ';', 0).toFloat();
      cmd_motor_right = getValue((String)inputString, ';', 1).toFloat();
      // clear the string:
      inputString = "";
      stringComplete = false;
      last_cmd_time = millis();
    }
    
    dt = (millis() - lastPidTime) / 1000;  //  [sec]
    if (dt >= pidDuration) {
      lastPidTime = millis();
      cmd_motor_left = pidCalc(cmd_motor_left, angularVelLKF.vel_kf, dt, integralLeft);
      cmd_motor_right = pidCalc(cmd_motor_right, angularVelRKF.vel_kf, dt, integralRight);
    }

    dt = (millis() - last_cmd_time) / 1000;  //  [sec]
    if (dt < maxCmdDuration) {
      drive((int)cmd_motor_left, (int)cmd_motor_right);
    }
    else {
      drive(0, 0);
    }
  }
  delay(1);
}

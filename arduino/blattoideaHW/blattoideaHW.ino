#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include <Adafruit_MCP4728.h>
#include <Wire.h>
Adafruit_MCP4728 mcp;

// sdaPin 20, sclPin 21;
const byte buttonPin = 15;
const byte PWMLinearPin = 45, PWMAngularPin = 46;
const byte directionRightPin = 29, directionLeftPin = 28;
const byte rightBrackPin = 25, leftBrackPin = 24;
//  Mega interrupt 2, 3, 18, 19, 20, 21
const byte encLeftPinA = 2, encLeftPinB = 3;
const byte encRghitPinA = 18, encRghitPinB = 19;
long oldPositionL  = -0, oldPositionR  = -0;
float D = 0.1651, wheelsSeparation = 0.42; //[m]
int pulsesPerRev = 60;
float duration = 0.1;  //  [sec]
double oldTime;
int velMaxVal = 4000;

float linearCmdVal, angularCmdVal;
bool direcLeftOld = true;
bool direcRightOld = true;
int minVelCmd = 80;

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
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit MCP4728 test!");
  // Try to initialize!
  if (!mcp.begin()) {
    Serial.println("Failed to find MCP4728 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found MCP4728 chip");
  mcp.setChannelValue(MCP4728_CHANNEL_A, 0);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 0);
  mcp.setChannelValue(MCP4728_CHANNEL_C, 0);
  mcp.setChannelValue(MCP4728_CHANNEL_D, 0);

  pinMode(directionRightPin, OUTPUT);
  pinMode(directionLeftPin, OUTPUT);
  oldTime = millis();
}

void loop() {
  if(digitalRead(buttonPin) == HIGH) {
    linearCmdVal = (float)pulseIn(PWMLinearPin, HIGH);
    angularCmdVal = (float)pulseIn(PWMAngularPin, HIGH);
        
    if (linearCmdVal == 0 || angularCmdVal == 0){
      linearCmdVal = 0;
      angularCmdVal = 0;
    }
    else{
      linearCmdVal = ((linearCmdVal - 1500) / 200) * velMaxVal;
      angularCmdVal = ((angularCmdVal - 1500) / 200) * velMaxVal;
      if (linearCmdVal > velMaxVal) linearCmdVal = velMaxVal;
      if (linearCmdVal < -velMaxVal) linearCmdVal = -velMaxVal;
      if (angularCmdVal > velMaxVal) angularCmdVal = velMaxVal;
      if (angularCmdVal < -velMaxVal) angularCmdVal = -velMaxVal;
    }
    
    float cmdRight = (linearCmdVal - angularCmdVal) / 2;
    float cmdLeft = (linearCmdVal + angularCmdVal) / 2;
    drive(cmdLeft, cmdRight);
  }

  double dt = (millis() - oldTime) / 1000;  //  [sec]
  if (dt >= duration){
    oldTime = millis();
    
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

    Serial.println(String("null") + ";" + String(angularVelLKF.vel_kf,8) + ";" + String(angularVelRKF.vel_kf,8) + ";" + "null");
  }
//  delay(1);
}

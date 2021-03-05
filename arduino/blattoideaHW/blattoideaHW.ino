#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#define CHANNELS 11
#include <Adafruit_MCP4728.h>
#include <Wire.h>
Adafruit_MCP4728 mcp;

const byte debug = 0;
const float maxCmdDuration = 0.5, pidDuration = 0.02, durationEncoder = 0.1, durationKF = 0.1, durationOdom = 0.1;  //  [sec]
const int minVelCmd = 80;
const int velMaxVal = 4000;
const float D = 0.1651, wheelsSeparation = 0.42; //[m]
const int pulsesPerRev = 60;
const int max_rem_val = 1810 ,min_rem_val = 1166;
const int norm_factor = (max_rem_val - min_rem_val) / 2;
const int mid_rem_val = 1470;
const float kp = 200, ki = 400, Ti = 10;
const double rleftR_kf = 1.1, leftQ_kf = 0.5;
const double rightR_kf = 1.1, rightQ_kf = 0.5;

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
bool pidReset = false;
bool stringComplete = false;  // whether the string is complete
float setPointTemp = 0;

long oldPositionL  = -0, oldPositionR  = -0;
double oldTimeEncoder = 0, oldTimeKF = 0, last_cmd_time = 0, lastPidTime = 0, oldTimeOdom = 0;
double cmd_motor_left = 0, cmd_motor_right = 0;
double linearCmdVal, angularCmdVal;
String inputString = "";         // a String to hold incoming data
double angularVelL = 0, angularVelR = 0;


struct kalman {
   double vel_kf;
   double p_kf;
   double sum;
   double velCov;
   int N;
};

struct pidPoseStruct {
   int cmd;
   double setPoint;
   double integral;
};

struct pidVelocityStruct {
   int cmd;
   double errorOld; 
   double setPoint;
};

struct kalman angularVelLKF = {vel_kf: 0.0, p_kf: 1.0, sum: 0.0, velCov: 0.0, N: 1};
struct kalman angularVelRKF = {vel_kf: 0.0, p_kf: 1.0, sum: 0.0, velCov: 0.0, N: 1};

struct pidPoseStruct pidLeftP = {cmd: 0, setPoint: 0, integral: 0};
struct pidPoseStruct pidRghitP = {cmd: 0, setPoint: 0, integral: 0};

struct pidVelocityStruct pidLeftV = {cmd: 0, errorOld: 0, setPoint: 0};
struct pidVelocityStruct pidRghitV = {cmd: 0, errorOld: 0, setPoint: 0};

double dist = 0.0, theta = 0.0, distError = 0.0, thetaError = 0.0, oldDist = 0.0, oldTheta = 0.0, dDist = 0.001, dTheta = 0.01;
bool resetError = false;

Encoder encL(encLeftPinA, encLeftPinB);
Encoder encR(encRghitPinA, encRghitPinB);


struct kalman kalmanFunc(double vel, struct kalman KF, double r_kf, double q_kf){
  KF.p_kf = KF.p_kf + q_kf;
  double y = vel - KF.vel_kf;
  double s = KF.p_kf + r_kf;
  double k = KF.p_kf/s;
  KF.vel_kf = KF.vel_kf + k*y;
  KF.p_kf = (1-k)*KF.p_kf;

  y = vel - KF.vel_kf;
  KF.sum = KF.sum + pow((y - KF.vel_kf), 2);
  KF.velCov = (1 / KF.N) * KF.sum - KF.p_kf;
  KF.N ++;
  return KF;
}


struct pidPoseStruct pidPose(struct pidPoseStruct pidS, double vel, double dt) {
  double error = 0;
  bool integralCalc = true;
  bool errorCalc = true;
  if(pidReset){
    pidS.integral = 0;
    errorCalc = false;
  }
  if(setPointTemp != 0){
    pidS.setPoint = setPointTemp;
    setPointTemp = 0;
    integralCalc = false;
  }
  if(errorCalc) error = pidS.setPoint - vel;
  if(integralCalc) pidS.integral = pidS.integral + error *  dt;
  pidS.cmd = kp * error + ki * pidS.integral;
  if(abs(pidS.cmd) > velMaxVal) {
    if(pidS.cmd > 0){
      pidS.cmd = velMaxVal;
      setPointTemp = velMaxVal;
    }
    else {
      pidS.cmd = -velMaxVal;
      setPointTemp = -velMaxVal;
    }
    if(ki != 0) {
      pidS.integral = (1 / ki) * (setPointTemp - (kp * error));
    }
    else {
      pidS.integral = 0;
    }
  }
  return pidS;
}


struct pidVelocityStruct pidVelocity(struct pidVelocityStruct pidS, double vel, double dt) {
  double error = 0;
  bool errorCalc = true;
  if(pidReset){
    pidS.errorOld = 0;
    pidS.cmd = 0;
    errorCalc = false;
  }
  if(errorCalc) error = pidS.setPoint - vel;
  pidS.cmd += (kp * (1 + (dt / Ti)) * error) - (kp * pidS.errorOld);
  pidS.errorOld = error;
  return pidS;
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


void stope(bool emergencyStope = false){
  do {
    digitalWrite(rightBrackPin, HIGH);
    digitalWrite(leftBrackPin, HIGH);
    mcp.setChannelValue(MCP4728_CHANNEL_A, (int)0);  //  lfet
    mcp.setChannelValue(MCP4728_CHANNEL_B, (int)0);  //  right
    delay(200);
  } while((angularVelLKF.vel_kf > 0.001 || angularVelRKF.vel_kf > 0.001) && emergencyStope);
}


void drive  (int leftVelCmd, int rightVelCmd) {
  if (leftVelCmd == 0 && rightVelCmd == 0) stope();
  if ((abs(leftVelCmd) > minVelCmd) || (abs(rightVelCmd) > minVelCmd)) {
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
      mcp.setChannelValue(MCP4728_CHANNEL_A, (int)0);  //  lfet
      direcLeftOld = direcLeft;
      pidReset = true;
      delay(200);
    }
    if (direcRight != direcRightOld) {
      digitalWrite(rightBrackPin, HIGH);
      mcp.setChannelValue(MCP4728_CHANNEL_B, (int)0);  //  right
      direcRightOld = direcRight;
      pidReset = true;
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
    digitalWrite(rightBrackPin, LOW);
    digitalWrite(leftBrackPin, LOW);
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
  oldTimeKF = millis();
  oldTimeOdom = millis();
  last_cmd_time = millis();
}


void loop() {
  double dt = (millis() - oldTimeEncoder) / 1000;  //  [sec]
  if(dt >= durationEncoder) {
    oldTimeEncoder = millis();

    long newPositionL = encL.read();
    long newPositionR = encR.read();

    angularVelL = -wheelAngularVel(newPositionL, oldPositionL, dt);  //  [rad/sec]
    angularVelR = wheelAngularVel(newPositionR, oldPositionR, dt);  //  [rad/sec]
    oldPositionL = newPositionL;
    oldPositionR = newPositionR;
  }

  dt = (millis() - oldTimeKF) / 1000;  //  [sec]
  if(dt >= durationKF) {
    oldTimeKF = millis();
    angularVelLKF = kalmanFunc (angularVelL, angularVelLKF, rleftR_kf, leftQ_kf);
    angularVelRKF = kalmanFunc (angularVelR, angularVelRKF, rightR_kf, rightQ_kf);
  
    double linearVelL = (D / 2) * angularVelLKF.vel_kf; //[m/sec]
    double linearVelR = (D / 2) * angularVelRKF.vel_kf;  //[m/sec]
    double linearErrorL = sqrt((sq((1 / 2) * angularVelLKF.vel_kf) * sq(0.001)) + (sq(D / 2) * sq(angularVelLKF.velCov)));  //+-[m/sec]
    double linearErrorR = sqrt((sq((1 / 2) * angularVelRKF.vel_kf) * sq(0.001)) + (sq(D / 2) * sq(angularVelRKF.velCov)));  //+-[m/sec]
  
    double omega = (linearVelR - linearVelL) / wheelsSeparation; //[rad/sec]
    double linearV = (linearVelR + linearVelL) / 2;  //[m/sec]
    double omegaError = sqrt((sq(1 / wheelsSeparation) * sq(linearErrorR)) + (sq(1 / wheelsSeparation) * sq(linearErrorL)) + (sq((linearVelL - linearVelR) / sq(wheelsSeparation)) * sq(0.005))); //+-[rad/sec]
    double linearErrorV = sqrt((sq(1/2) * sq(linearErrorR)) + (sq(1/2) * sq(linearVelL)));  //+-[m/sec]

    dist = linearV * dt + dist;  //[m]
    theta = omega * dt + theta;  //[rad]
    if(!resetError) {
      if((dist - oldDist) > dDist) {
        distError = sqrt((sq(1 * dt) * sq(linearErrorV)) + (sq(linearV * 1) * sq(dt - durationKF)) + (sq(1) * sq(distError)));  //+-[m]
        oldDist = dist;
      }
      if((theta - oldTheta) > dTheta) {
        thetaError = sqrt((sq(1 * dt) * sq(omegaError)) + (sq(omega * 1) * sq(dt - durationKF)) + (sq(1) * sq(thetaError)));  //+-[rad]
        oldTheta = theta;
      }
    }
    else {
      distError = 0.0;
      thetaError = 0.0;
      resetError = false;
    }
  
    double x = dist * cos(theta);  //[m]
    double y = dist * sin(theta);  //[m]
    double xError = sqrt((sq(1 * cos(theta)) * sq(distError)) + (sq(dist * (-sin(theta))) * sq(thetaError)));  //[m]
    double yError = sqrt((sq(1 * sin(theta)) * sq(distError)) + (sq(dist * cos(theta)) * sq(thetaError)));  //[m]
    
    dt = (millis() - oldTimeOdom) / 1000;  //  [sec]
    if(dt >= durationOdom) {
      oldTimeOdom = millis();
      if (debug==1)
      {
        Serial2.println("wheelAngularVel;" + String(angularVelLKF.vel_kf,8) + ";" + String(angularVelRKF.vel_kf,8) + '\n');
      }
      Serial2.print((String)"Odom;" + "Twist;" + "angular;" + String(omega,8) + ";linear;" + String(linearV,8));
      Serial2.println((String)";Pose;" + "x;" + String(x,8) + ";y;" + String(y,8) + ";theta;" + String(theta,8) + ";" + '\n');
    }
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
      cmd_motor_left = getValue((String)inputString, ';', 0).toDouble();
      cmd_motor_right = getValue((String)inputString, ';', 1).toDouble();
      // clear the string:
      inputString = "";
      stringComplete = false;
      last_cmd_time = millis();
    }
    
    dt = (millis() - lastPidTime) / 1000;  //  [sec]
    if (dt >= pidDuration) {
      lastPidTime = millis();
      pidLeftV.setPoint = cmd_motor_left;
      pidRghitV.setPoint = cmd_motor_right;
      pidLeftV = pidVelocity(pidLeftV, angularVelLKF.vel_kf, dt);
      pidRghitV = pidVelocity(pidRghitV, angularVelRKF.vel_kf, dt);
      cmd_motor_left = pidLeftV.cmd;
      cmd_motor_right = pidRghitV.cmd;
    }

    dt = (millis() - last_cmd_time) / 1000;  //  [sec]
    if (dt < maxCmdDuration) {
      if (cmd_motor_left > velMaxVal) cmd_motor_left = velMaxVal;
      if (cmd_motor_left < -velMaxVal) cmd_motor_left = -velMaxVal;
      if (cmd_motor_right > velMaxVal) cmd_motor_right = velMaxVal;
      if (cmd_motor_right < -velMaxVal) cmd_motor_right = -velMaxVal;
      drive((int)cmd_motor_left, (int)cmd_motor_right);
    }
    else stope();
  }
  delay(1);
}

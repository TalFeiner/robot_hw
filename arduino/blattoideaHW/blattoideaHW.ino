#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Adafruit_MCP4728.h>
Adafruit_MCP4728 mcp;

const byte debug = 0;
const float maxCmdDuration = 0.5, pidDuration = 0.1, durationEncoder = 0.1;  //  [sec]
const float durationKF = durationEncoder, durationOdom = durationEncoder;  //  [sec]
const int minVelCmd = 240;
const int cmdMaxVal = 4000;
const float D = 0.1651, wheelsSeparation = 0.42; //[m]
const int pulsesPerRev = 60;
const int max_rem_val = 1900 ,min_rem_val = 1100;
const int norm_factor = (max_rem_val - min_rem_val) / 2;
const int mid_rem_val = 1500;
const float kp = 40, ki = 100, kd = 0, Ti = 1;
const float rleftR_kf = 1.1, leftQ_kf = 0.5;
const float rightR_kf = 1.1, rightQ_kf = 0.5;

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

long oldPositionL  = -0, oldPositionR  = -0;
double oldTimeEncoder = 0, oldTimeKF = 0, last_cmd_time = 0, lastPidTime = 0, oldTimeOdom = 0;
double cmd_motor_left = 0, cmd_motor_right = 0;
double linearCmdVal, angularCmdVal;
String inputString = "";         // a String to hold incoming data
double angularVelL = 0, angularVelR = 0;
bool resetPidFlag = true;
int count = 0;


struct kalman {
   double vel_kf;
   double p_kf;
   double sum;
   double velCov;
   long N;
   bool resetVar;
};

struct pidPoseStruct {
   int cmd;
   double setPoint;
   double integral;
   double errorOld;
   float _setPointTemp;
   bool pidReset;
};

struct pidVelocityStruct {
   int cmd;
   double errorOld; 
   double setPoint;
   bool pidReset;
};

struct kalman angularVelLKF = {vel_kf: 0.0, p_kf: 1.0, sum: 0.0, velCov: 0.0, N: 1, resetVar: false};
struct kalman angularVelRKF = {vel_kf: 0.0, p_kf: 1.0, sum: 0.0, velCov: 0.0, N: 1, resetVar: false};

struct pidPoseStruct pidLeftP = {cmd: 0, setPoint: 0, integral: 0, errorOld: 0, _setPointTemp: 0, pidReset: false};
struct pidPoseStruct pidRghitP = {cmd: 0, setPoint: 0, integral: 0, errorOld: 0, _setPointTemp: 0, pidReset: false};
struct pidVelocityStruct pidLeftV = {cmd: 0, errorOld: 0, setPoint: 0, pidReset: false};
struct pidVelocityStruct pidRghitV = {cmd: 0, errorOld: 0, setPoint: 0, pidReset: false};

double dist = 0.0, theta = 0.0, distError = 0.0, thetaError = 0.0, oldDist = 0.0, oldTheta = 0.0;
const float dDist = 0.001, dTheta = 0.01;
bool resetError = false;

Encoder encR(encLeftPinA, encLeftPinB);
Encoder encL(encRghitPinA, encRghitPinB);


struct kalman kalmanFunc(double vel, struct kalman KF, double r_kf, double q_kf){
  KF.p_kf = KF.p_kf + q_kf;
  double y = vel - KF.vel_kf;
  double s = KF.p_kf + r_kf;
  double k = KF.p_kf/s;
  KF.vel_kf = KF.vel_kf + k*y;
  KF.p_kf = (1-k)*KF.p_kf;

  if(KF.N >= (KF.N + 1)) KF.resetVar = true;
  if(!KF.resetVar){
    y = vel - KF.vel_kf;
    double tmpSq = y - KF.vel_kf;
    KF.sum += sq(tmpSq);
    KF.velCov = (1 / KF.N) * KF.sum - KF.p_kf;
    KF.N ++;
  }
  else {
    KF.sum = 0;
    KF.velCov = 0;
    KF.N = 1;
    KF.resetVar = false;

    y = vel - KF.vel_kf;
    double tmpSq = y - KF.vel_kf;
    KF.sum += sq(tmpSq);
    KF.velCov = (1 / KF.N) * KF.sum - KF.p_kf;
    KF.N ++;
  }
  return KF;
}


struct pidPoseStruct pidPose(struct pidPoseStruct pidS, double vel, double dt) {
  double error = 0;
  double derivative = 0;
  bool integralCalc = true;
  bool errorCalc = true;
  bool derivativeCalc = true;
  if(pidS.pidReset){
    pidS.integral = 0;
    pidS.errorOld = 0;
    errorCalc = false;
    derivativeCalc = false;
    integralCalc = false;
    pidS.pidReset = false;
  }
  if(pidS._setPointTemp != 0){
    pidS.setPoint = pidS._setPointTemp;
    pidS._setPointTemp = 0;
    integralCalc = false;
  }
  if(errorCalc) error = pidS.setPoint - vel;
  if(integralCalc) pidS.integral += error *  dt;
  if(derivativeCalc) derivative = (error - pidS.errorOld) / dt;
  pidS.cmd = kp * error + ki * pidS.integral + kd * derivative;
  if(fabs(pidS.cmd) > cmdMaxVal) {
    if(pidS.cmd > 0){
      pidS.cmd = cmdMaxVal;
      pidS._setPointTemp = cmdMaxVal;
    }
    else {
      pidS.cmd = -cmdMaxVal;
      pidS._setPointTemp = -cmdMaxVal;
    }
    if(ki != 0) {
      pidS.integral += (1 / ki) * (pidS._setPointTemp - (kp * error));
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
  if(pidS.pidReset){
    pidS.errorOld = 0;
    pidS.cmd = 0;
    errorCalc = false;
    pidS.pidReset = false;
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
  while((fabs(angularVelLKF.vel_kf) > 0.01 || fabs(angularVelRKF.vel_kf) > 0.01) && emergencyStope) {
    digitalWrite(rightBrackPin, HIGH);
    digitalWrite(leftBrackPin, HIGH);
    mcp.setChannelValue(MCP4728_CHANNEL_A, (int)0);  //  lfet
    mcp.setChannelValue(MCP4728_CHANNEL_B, (int)0);  //  right
    delay(200);
    double dt = (millis() - oldTimeEncoder) / 1000;  //  [sec]
    if(dt >= durationEncoder) {
      oldTimeEncoder = millis();

      long newPositionL = encL.read();
      long newPositionR = -encR.read();

      angularVelL = wheelAngularVel(newPositionL, oldPositionL, dt);  //  [rad/sec]
      angularVelR = wheelAngularVel(newPositionR, oldPositionR, dt);  //  [rad/sec]
      oldPositionL = newPositionL;
      oldPositionR = newPositionR;

      angularVelLKF = kalmanFunc (angularVelL, angularVelLKF, rleftR_kf, leftQ_kf);
      angularVelRKF = kalmanFunc (angularVelR, angularVelRKF, rightR_kf, rightQ_kf);
    }
    digitalWrite(rightBrackPin, LOW);
    digitalWrite(leftBrackPin, LOW);
    delay(100);
  }
  if (emergencyStope) emergencyStope = false;
  digitalWrite(rightBrackPin, HIGH);
  digitalWrite(leftBrackPin, HIGH);
  mcp.setChannelValue(MCP4728_CHANNEL_A, (int)0);  //  lfet
  mcp.setChannelValue(MCP4728_CHANNEL_B, (int)0);  //  right
  if(resetPidFlag){
    pidRghitV.pidReset = true;
    pidRghitP.pidReset = true;
    angularVelRKF.resetVar = true;
    pidLeftV.pidReset = true;
    pidLeftP.pidReset = true;
    angularVelLKF.resetVar = true;
    resetPidFlag = false;
  }
  delay(200);
}


void drive  (int leftVelCmd, int rightVelCmd) {
    bool direcLeft;
    bool direcRight;
    
    if (leftVelCmd > 0) {
      direcLeft = true;
    }
    else if(leftVelCmd < 0){
      direcLeft = false;
    }
    else{
      direcLeft = direcLeftOld;
    }
    if (rightVelCmd > 0) {
      direcRight = true;
    }
    else if(rightVelCmd < 0){
      direcRight = false;
    }
     else{
      direcRight = direcRightOld;
    }
  
    if ((direcLeft != direcLeftOld) && (direcRight != direcRightOld)) {
      stope();
      direcLeftOld = direcLeft;
      direcRightOld = direcRight;
    }
    else if (direcLeft != direcLeftOld) {
      digitalWrite(leftBrackPin, HIGH);
      mcp.setChannelValue(MCP4728_CHANNEL_A, (int)0);  //  lfet
      direcLeftOld = direcLeft;
      pidLeftV.pidReset = true;
      pidLeftP.pidReset = true;
      angularVelLKF.resetVar = true;
      delay(200);
    }
    else if (direcRight != direcRightOld) {
      digitalWrite(rightBrackPin, HIGH);
      mcp.setChannelValue(MCP4728_CHANNEL_B, (int)0);  //  right
      direcRightOld = direcRight;
      pidRghitV.pidReset = true;
      pidRghitP.pidReset = true;
      angularVelRKF.resetVar = true;
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


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  // reserve 2 bytes for the inputString:
  inputString.reserve(2);
  
  while (!Serial2)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial2.println("Opene Serial2");
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial2.println("Opene Serial");
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
  delay(1);
  Serial.flush();
  Serial2.flush();
  delay(1);
}


void loop() {
  double dt = (millis() - oldTimeEncoder) / 1000;  //  [sec]
  if(dt >= durationEncoder) {
    oldTimeEncoder = millis();

    long newPositionL = encL.read();
    long newPositionR = -encR.read();

    angularVelL = wheelAngularVel(newPositionL, oldPositionL, dt);  //  [rad/sec]
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
    double tmpDSq = D / 2;
    double tmpAngularVelRKFLSq = (0.5) * angularVelLKF.vel_kf;
    double tmpLinearErrorLSqrt = (sq(tmpAngularVelRKFLSq) * sq(0.001)) + (sq(tmpDSq) * sq(angularVelLKF.velCov));
    double linearErrorL = sqrt(tmpLinearErrorLSqrt);  //+-[m/sec]
    double tmpAngularVelRKFRSq = (0.5) * angularVelRKF.vel_kf;
    double tmpLinearErrorRSqrt = (sq(tmpAngularVelRKFRSq) * sq(0.001)) + (sq(tmpDSq) * sq(angularVelRKF.velCov));
    double linearErrorR = sqrt(tmpLinearErrorRSqrt);  //+-[m/sec]
  
    double omega = (linearVelR - linearVelL) / wheelsSeparation; //[rad/sec]
    double linearV = (linearVelR + linearVelL) / 2;  //[m/sec]
    double tmpWheelsSeparationSq = 1 / wheelsSeparation;
    double tmpLinearVelSq = (linearVelL - linearVelR) / sq(wheelsSeparation);
    double tmpOmegaErrorSqrt = (sq(tmpWheelsSeparationSq) * sq(linearErrorR)) + (sq(tmpWheelsSeparationSq) * sq(linearErrorL)) + (sq(tmpLinearVelSq) * sq(0.005));
    double omegaError = sqrt(tmpOmegaErrorSqrt); //+-[rad/sec]
    double tmpLinearErrorVSqrt = (sq(0.5) * sq(linearErrorR)) + (sq(0.5) * sq(linearVelL));
    double linearErrorV = sqrt(tmpLinearErrorVSqrt);  //+-[m/sec]

    dist += linearV * dt;  //[m]
    theta += omega * dt;  //[rad]
    double x, y, xError, yError;
    if(!resetError) {
      if((dist - oldDist) > dDist) {
        double tmpDtSq = dt - durationKF;
        double tmpDistErrorSqrt = (sq(dt) * sq(linearErrorV)) + (sq(linearV) * sq(tmpDtSq)) + (sq(distError));
        distError = sqrt(tmpDistErrorSqrt);  //+-[m]
        oldDist = dist;
      }
      if((theta - oldTheta) > dTheta) {
        double tmpDtSq = dt - durationKF;
        double tmpThetaErrorSqrt = (sq(dt) * sq(omegaError)) + (sq(omega) * sq(tmpDtSq)) + (sq(thetaError));
        thetaError = sqrt(tmpThetaErrorSqrt);  //+-[rad]
        oldTheta = theta;
      }
      x = dist * cos(theta);  //[m]
      y = dist * sin(theta);  //[m]
      double tmpCosxErrorSq = cos(theta);
      double tmpSinxErrorSq = dist * (-sin(theta));
      double tmpxErrorSqrt = (sq(tmpCosxErrorSq) * sq(distError)) + (sq(tmpSinxErrorSq) * sq(thetaError));
      xError = sqrt(tmpxErrorSqrt);  //[m]
      double tmpSinyErrorSq = sin(theta);
      double tmpCosyErrorSq = dist * cos(theta);
      double tmpyErrorSqrt = (sq(tmpSinyErrorSq) * sq(distError)) + (sq(tmpCosyErrorSq) * sq(thetaError));
      yError = sqrt(tmpyErrorSqrt);  //[m]
    }
    else {
      distError = 0.0;
      thetaError = 0.0;
      x = dist * cos(theta);  //[m]
      y = dist * sin(theta);  //[m]
      double tmpCosxErrorSq = cos(theta);
      double tmpSinxErrorSq = dist * (-sin(theta));
      double tmpxErrorSqrt = (sq(tmpCosxErrorSq) * sq(distError)) + (sq(tmpSinxErrorSq) * sq(thetaError));
      xError = sqrt(tmpxErrorSqrt);  //[m]
      double tmpSinyErrorSq = sin(theta);
      double tmpCosyErrorSq = dist * cos(theta);
      double tmpyErrorSqrt = (sq(tmpSinyErrorSq) * sq(distError)) + (sq(tmpCosyErrorSq) * sq(thetaError));
      yError = sqrt(tmpyErrorSqrt);  //[m]
      Serial2.print((String)"Odom;" + "Twist;" + "angular;" + String(omega,8) + ";linear;" + String(linearV,8));
      Serial2.print((String)";Pose;" + "x;" + String(x,8) + ";y;" + String(y,8) + ";theta;" + String(theta,8));
      Serial2.println((String)";xVar;" + String(xError,8) + ";yVar;" + String(yError,8) + ";thetaVar;" + String(thetaError,8) + ";angularVar;" + String(omegaError,8) + ";linearVar;" + String(linearErrorV,8) + ";" + '\n');
      resetError = false;
    }

    dt = (millis() - oldTimeOdom) / 1000;  //  [sec]
    if(dt >= durationOdom) {
      oldTimeOdom = millis();
      if (debug==1)
      {
        Serial2.println("wheelAngularVel;" + String(angularVelLKF.vel_kf,8) + ";" + String(angularVelRKF.vel_kf,8) + '\n');
      }
      Serial2.print((String)"Odom;" + "Twist;" + "angular;" + String(omega,8) + ";linear;" + String(linearV,8));
      Serial2.print((String)";Pose;" + "x;" + String(x,8) + ";y;" + String(y,8) + ";theta;" + String(theta,8));
      Serial2.println((String)";xVar;" + String(xError,8) + ";yVar;" + String(yError,8) + ";thetaVar;" + String(thetaError,8) + ";angularVar;" + String(omegaError,8) + ";linearVar;" + String(linearErrorV,8) + ";" + '\n');
    }
  }

  if(digitalRead(buttonPin) == HIGH) {
    linearCmdVal = (float)pulseIn(PWMLinearPin, HIGH) + 50;
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
      linearCmdVal = ((linearCmdVal - mid_rem_val) / norm_factor) * cmdMaxVal / 2;
      angularCmdVal = ((angularCmdVal - mid_rem_val) / norm_factor) * cmdMaxVal / 2;
      if (linearCmdVal > cmdMaxVal) linearCmdVal = cmdMaxVal;
      if (linearCmdVal < -cmdMaxVal) linearCmdVal = -cmdMaxVal;
      if (angularCmdVal > cmdMaxVal) angularCmdVal = cmdMaxVal;
      if (angularCmdVal < -cmdMaxVal) angularCmdVal = -cmdMaxVal;
    }

    int cmdRight = (linearCmdVal - angularCmdVal) / 2;
    int cmdLeft = (linearCmdVal + angularCmdVal) / 2;

    if (debug==1)
    {
      Serial2.println("Debug;cmd: cmdRight - " + (String)(cmdRight) + " , cmdLeft - " + (String)(cmdLeft));
    }
    if ((abs(cmdLeft) > minVelCmd) && (abs(cmdRight) > minVelCmd)) {
      drive((int)cmdLeft, (int)cmdRight);
      }
    else if ((abs(cmdLeft) > minVelCmd) || (abs(cmdRight) > minVelCmd)){
      if (abs(cmdLeft) > minVelCmd){
        drive((int)cmdLeft, (int)0);
      }
      if (abs(cmdRight) > minVelCmd){
        drive((int)0, (int)cmdRight);
      }
    }
  else {
    digitalWrite(rightBrackPin, LOW);
    digitalWrite(leftBrackPin, LOW);
    mcp.setChannelValue(MCP4728_CHANNEL_A, (int)0);  //  lfet
    mcp.setChannelValue(MCP4728_CHANNEL_B, (int)0);  //  right
  }
  }
  else {
    // print the string when a newline arrives:
    if (stringComplete) {
      if (debug==1)
      {
        Serial2.println("Debug;inputString: " + inputString);
      }
      String stringName = getValue((String)inputString, ';', 0);
      if(stringName.equalsIgnoreCase("emergencyStope")) {
        stope(true);
      }
      else if(stringName.equalsIgnoreCase("cmdVel")) {
        cmd_motor_left = getValue((String)inputString, ';', 1).toDouble();
        cmd_motor_right = getValue((String)inputString, ';', 2).toDouble();
        last_cmd_time = millis();
      }
      else if(stringName.equalsIgnoreCase("resetError")) {
        resetError = true;
      }
      // clear the string:
      inputString = "";
      stringComplete = false;
    }

//    if (fabs(cmd_motor_left) == 0 && fabs(cmd_motor_right) == 0) stope();
//    else {
//      resetPidFlag = true;
//      dt = (millis() - lastPidTime) / 1000;  //  [sec]
//      if (dt >= pidDuration) {
//        lastPidTime = millis();
//        pidLeftV.setPoint = cmd_motor_left;
//        pidRghitV.setPoint = cmd_motor_right;
//        pidLeftV = pidVelocity(pidLeftV, angularVelLKF.vel_kf, dt);
//        pidRghitV = pidVelocity(pidRghitV, angularVelRKF.vel_kf, dt);
//        motor_left = (int)pidLeftV.cmd;
//        motor_right = (int)pidRghitV.cmd;
//        if (debug==1){
//          Serial2.println("Debug;cmd: cmdRight - " + (String)(motor_right) + " , " + (String)(angularVelRKF.vel_kf) + " , cmdLeft - " + (String)(motor_left) + " , " + (String)(angularVelLKF.vel_kf));
//        }
//      }
//    }

    int motor_left;
    int motor_right;
    if (fabs(cmd_motor_left) == 0 && fabs(cmd_motor_right) == 0) stope();
    else {
     resetPidFlag = true;
     dt = (millis() - lastPidTime) / 1000;  //  [sec]
     if (dt >= pidDuration) {
       lastPidTime = millis();
       pidLeftP.setPoint = cmd_motor_left;
       pidRghitP.setPoint = cmd_motor_right;
       pidLeftP = pidPose(pidLeftP, angularVelLKF.vel_kf, dt);
       pidRghitP = pidPose(pidRghitP, angularVelRKF.vel_kf, dt);
       motor_left = (int)pidLeftP.cmd;
       motor_right = (int)pidRghitP.cmd;
       if (debug==1){
         Serial2.println("Debug;cmd: cmdRight - " + (String)(motor_right) + " , " + (String)(angularVelRKF.vel_kf) + " , cmdLeft - " + (String)(motor_left) + " , " + (String)(angularVelLKF.vel_kf));
       }
     }
   }

    // int motor_left = cmd_motor_left;
    // int motor_right = cmd_motor_right;
    dt = (millis() - last_cmd_time) / 1000;  //  [sec]
    if (dt < maxCmdDuration) {
      if (motor_left > cmdMaxVal) motor_left = cmdMaxVal;
      if (motor_left < -cmdMaxVal) motor_left = -cmdMaxVal;
      if (motor_right > cmdMaxVal) motor_right = cmdMaxVal;
      if (motor_right < -cmdMaxVal) motor_right = -cmdMaxVal;
      drive((int)motor_left, (int)motor_right);
    }
    else stope();
  }
  delay(1);
  count++;
  if (count % 100 == 0){
    Serial.flush();
    Serial2.flush();
    count = 0;
  }
}

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

//Mega 2, 3, 18, 19, 20, 21
byte pinL1 = 2, pinL2 = 3; // 18
byte pinR1 = 20, pinR2 = 21;  // 21
long oldPositionL  = -0, oldPositionR  = -0;
float D = 0.1651, wheelsSeparation = 0.42; //[m]
int pulsesPerRev = 60, duration = 100;
double oldTime;

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

Encoder encL(pinL1, pinL2);
Encoder encR(pinR1, pinR2);

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

struct kalman kalmanFunc(double vel, struct kalman KF, double r_kf, double q_kf){
  KF.p_kf = KF.p_kf + q_kf;
  double y = vel - KF.vel_kf;
  double s = KF.p_kf + r_kf;
  double k = KF.p_kf/s;
  KF.vel_kf = KF.vel_kf + k*y;
  KF.p_kf = (1-k)*KF.p_kf;
  return KF;
}

double wheelAngularVel(long newPosition, long oldPosition, double dt){
  double dtheta = (double)(2 * PI) * (double)((double)((double)newPosition - (double)oldPosition) / (double)pulsesPerRev); //[rad]
  return dtheta/dt;  //[rad/sec]
}

void setup() {
  Serial.begin(115200);
  oldTime = millis();
}

void loop() {
  double dt = millis() - oldTime;
  if (dt >= duration){
    oldTime = millis();
    dt = dt / 1000.0;  //[sec]
    
    long newPositionL = encL.read();
    long newPositionR = encR.read();
    
    double angularVelL = wheelAngularVel(newPositionL, oldPositionL, dt);  //[rad/sec]
    double angularVelR = wheelAngularVel(newPositionR, oldPositionR, dt);  //[rad/sec]
    
    oldPositionL = newPositionL;
    oldPositionR = newPositionR;

    angularVelLKF = kalmanFunc (angularVelL, angularVelLKF, rleftR_kf, leftQ_kf);
    angularVelRKF = kalmanFunc (angularVelR, angularVelRKF, rightR_kf, rightQ_kf);
    
    double linearVelL = (D / 2) * angularVelLKF.vel_kf; //[m/sec]
    double linearVelR = (D / 2) * angularVelRKF.vel_kf;  //[m/sec]
    double linearErrorL = sqrt((sq((1 / 2) * angularVelLKF.vel_kf) * sq(0.001)) + (sq(D / 2) * sq(angularVelLKF.p_kf)));  //+-[m/sec]
    double linearErrorR = sqrt((sq((1 / 2) * angularVelRKF.vel_kf) * sq(0.001)) + (sq(D / 2) * sq(angularVelRKF.p_kf)));  //+-[m/sec]

    double omega = (linearVelR - linearVelL) / wheelsSeparation; //[rad/sec]
    double linearV = (linearVelR + linearVelL) / 2;  //[m/sec]
    double omegaError = sqrt((sq(1 / wheelsSeparation) * sq(linearErrorR)) + (sq(1 / wheelsSeparation) * sq(linearErrorL)) + (sq((linearVelL - linearVelR) / sq(wheelsSeparation)) * sq(0.005))); //+-[rad/sec]
    double linearErrorV = sqrt((sq(1/2) * sq(linearErrorR)) + (sq(1/2) * sq(linearVelL)));  //+-[m/sec]
    
    dist = linearV * dt + dist;  //[m]
    theta = omega * dt + theta;  //[rad]
    if(Serial.available()){
      String str = Serial.readString();
      String resetError = String(getValue(str, ';', 0));
    }
    if (resetError.equalsIgnoreCase("false")) {
      if  ((dist - oldDist) > dDist) {
        Serial.println("distError: " + String(distError));
        distError = sqrt((sq(1 * dt) * sq(linearErrorV)) + (sq(linearV * 1) * sq(dt - (((double)duration) / 1000.0))) + (sq(1) * sq(distError)));  //+-[m]
        oldDist = dist;
      }
      if  ((theta - oldTheta) > dTheta) {
        Serial.println("thetaError: " + String(thetaError));
        thetaError = sqrt((sq(1 * dt) * sq(omegaError)) + (sq(omega * 1) * sq(dt - (((double)duration) / 1000.0))) + (sq(1) * sq(thetaError)));  //+-[rad]
        oldTheta = theta;
      }
    }
    else if (resetError.equalsIgnoreCase("true"))  {
      distError = 0.0;
      thetaError = 0.0;
      resetError = String("false");
    }

    double x = dist * cos(theta);  //[m]
    double y = dist * sin(theta);  //[m]
    double xError = sqrt((sq(1 * cos(theta)) * sq(distError)) + (sq(dist * (-sin(theta))) * sq(thetaError)));  //[m]
    double yError = sqrt((sq(1 * sin(theta)) * sq(distError)) + (sq(dist * cos(theta)) * sq(thetaError)));  //[m]

    Serial.println(String("null") + ";" + String(linearV) + ";" + String(omega) + ";" + "null");
      
     //---- debug -----//
//    Serial.print("vel L: ");
//    Serial.println(angularVelLKF);
//    Serial.println("\t, vel R: ");
//    Serial.println(angularVelRKF);

//    Serial.println(String(distError) + ";" + String(dist - oldDist));
//    Serial.println(String(linearV) + ";" + String(omega) + ";" + "x: " + String(x) + " ; " + "y: " + String(y) + " ; " + "theta: " + String(theta));
//      Serial.println("x: " + String(x) + " ; " + "xError: " + String(xError) + " ; " + "y: " + String(y) + " ; " + "yError: " + String(yError) + " ; " + "theta: " + String(theta)+ " ; " + "thetaError: " + String(thetaError));
  }
  delay(1);
}

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <Coordinates.h>

//Mega 2, 3, 18, 19, 20, 21
byte pinL1 = 2, pinL2 = 3; // 18
byte pinR1 = 19, pinR2 = 21;  // 21
long oldPositionL  = -999, oldPositionR  = -999;
float D = 0.1651, wheelsSeparation = 0.36; //[m]
int pulsesPerRev = 60, duration = 2;
double oldTime;

struct kalman {
   double vel_kf;
   double p_kf;
};

double rleftR_kf = 1.1, leftQ_kf = 0.5;
double rightR_kf = 1.1, rightQ_kf = 0.5;
struct kalman angularVelLKF = {vel_kf: 0.0, p_kf: 1.0};
struct kalman angularVelRKF = {vel_kf: 0.0, p_kf: 1.0};

double s, theta;

Encoder encL(pinL1, pinL2);
Encoder encR(pinR1, pinR2);

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

    angularVelLKF = kalmanFunc (-angularVelL, angularVelLKF, rleftR_kf, leftQ_kf);
    angularVelRKF = kalmanFunc (-angularVelR, angularVelRKF, rightR_kf, rightQ_kf);
    
    double linearVelL = D / 2 * angularVelLKF.vel_kf; //[m/sec]
    double linearVelR = D / 2 * angularVelRKF.vel_kf;  //[m/sec]

    double omega = (linearVelR - linearVelL) / wheelsSeparation; //[rad/sec]
    double linearV = (linearVelR + linearVelL) / 2;  //[m/sec]

    s = linearV * dt + s;  //[m]
    theta = omega * dt + theta;  //[rad]

    Coordinates point = Coordinates();
    point.fromPolar(s,theta);
    float x = point.getX();  //[m]
    float y = point.getY();  //[m]
      
     //---- debug -----//
//    Serial.print("vel L: ");
//    Serial.println(angularVelLKF);
//    Serial.println("\t, vel R: ");
//    Serial.println(angularVelRKF);

    Serial.println(String(angularVelLKF.vel_kf) + ";" + String(angularVelRKF.vel_kf));
  }
  delay(1);
}

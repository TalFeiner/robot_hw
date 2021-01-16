#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

//Mega 2, 3, 18, 19, 20, 21
byte pinL1 = 2, pinL2 = 3; // 18
byte pinR1 = 19, pinR2 = 20;  // 21
long oldPositionL  = -999, oldPositionR  = -999;
float D = 0.1651, wheelsSeparation = 0.36; //[m]
int pulsesPerRev = 60;
double oldTime;

double rleftR_kf = 1.1, leftQ_kf = 0.5;
double rightR_kf = 1.1, rightQ_kf = 0.5;

double xR_kf = 1.1, xQ_kf = 0.5;
double thetaR_kf = 1.1, thetaQ_kf = 0.5;

double x, theta;

struct kalmanReturn {
   double val;
   double cov;
};

Encoder encL(pinL1, pinL2);
Encoder encR(pinR1, pinR2);

struct kalmanReturn kalmanF(double vel, double vel_kf, double p_kf, double r_kf, double q_kf){
  struct kalmanReturn out;
  p_kf = p_kf + q_kf;
  double y = vel - vel_kf;
  double s = p_kf + r_kf;
  double k = p_kf/s;
  //vel_kf = vel_kf + k*y;
  //p_kf = (1-k)*p_kf;
  out.val = vel_kf + k*y;
  out.cov = (1-k)*p_kf;
  return out;
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
  if (dt >= 10){
    oldTime = millis();
    dt = dt / 1000.0;  //[sec]
    
    long newPositionL = encL.read();
    long newPositionR = encR.read();
    
    double angularVelL = wheelAngularVel(newPositionL, oldPositionL, dt);  //[rad/sec]
    double angularVelR = wheelAngularVel(newPositionR, oldPositionR, dt);  //[rad/sec]
    
    oldPositionL = newPositionL;
    oldPositionR = newPositionR;

    struct kalmanReturn angularVelLK = kalmanF (angularVelL, angularVelLK.val, angularVelLK.cov, rleftR_kf, leftQ_kf);
    struct kalmanReturn angularVelRK = kalmanF (angularVelR, angularVelRK.val, angularVelRK.cov, rightR_kf, rightQ_kf);
    
    double linearVelL = D / 2 * angularVelLK.val; //[m/sec]
    double linearVelR = D / 2 * angularVelRK.val;  //[m/sec]

    double omega = (linearVelR - linearVelL) / wheelsSeparation; //[rad/sec]
    double linearV = (linearVelR + linearVelL) / 2;  //[m/sec]

    x = linearV * dt + x;
    theta = omega * dt + theta;
   
    struct kalmanReturn xK = kalmanF (x, xK.val, xK.cov, xR_kf, xQ_kf);
    struct kalmanReturn thetaK = kalmanF (theta, thetaK.val, thetaK.cov, thetaR_kf, thetaQ_kf);
      
     //---- debug -----//
//    Serial.print("vel L: ");
//    Serial.println(angularVelL);
//    Serial.println("\t, vel R: ");
//    Serial.println(angularVelR);

    Serial.println(String(angularVelLK.val) + ";" + String(angularVelRK.val));
  }
  delay(1);
}

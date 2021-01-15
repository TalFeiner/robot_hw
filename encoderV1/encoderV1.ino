#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

//Mega 2, 3, 18, 19, 20, 21
byte pinL1 = 2, pinL2 = 3; // 18
byte pinR1 = 19, pinR2 = 20;  // 21
long oldPositionL  = -999, oldPositionR  = -999;
double D = 0.1651;
int pulsesPerRev = 60;
double oldTime;

double leftVel_kf = 0, leftP_kf = 1.0, rleftR_kf = 1.1, leftQ_kf = 0.5;
double rightVel_kf = 0, rightP_kf = 1.0, rightR_kf = 1.1, rightQ_kf = 0.5;

Encoder encL(pinL1, pinL2);
Encoder encR(pinR1, pinR2);

double kalmanF (double vel, double vel_kf, double p_kf, double r_kf, double q_kf){
  p_kf = p_kf + q_kf;
  double y = vel - vel_kf;
  double s = p_kf + r_kf;
  double k = p_kf/s;
  vel_kf = vel_kf + k*y;
  p_kf = (1-k)*p_kf;
  return vel_kf;
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
  if (dt >= 100){
    oldTime = millis();
    dt = dt / 1000.0;  //[sec]
    
    long newPositionL = encL.read();
    long newPositionR = encR.read();
    
    double velL = wheelAngularVel(newPositionL, oldPositionL, dt);
    double velR = wheelAngularVel(newPositionR, oldPositionR, dt);
    
    oldPositionL = newPositionL;
    oldPositionR = newPositionR;

    velL = kalmanF (velL, leftVel_kf, leftP_kf, rleftR_kf, leftQ_kf);
    velR = kalmanF (velR, rightVel_kf, rightP_kf, rightR_kf, rightQ_kf);
      
     //---- debug -----//
//    Serial.print("vel L: ");
//    Serial.println(velL);
//    Serial.println("\t, vel R: ");
//    Serial.println(velR);

    Serial.println(String(velL) + ";" + String(velR));
  }
  delay(1);
}

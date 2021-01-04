double vt;
double vel_kf = 0;
double p_kf = 1.0;
double r_kf = 1.1;
double q_kf = 0.5;

double theta_old = 0.0, time_old; // theta_predict = 0.0, P_k = 0.0;
double D = 0.1651, vel = 0.0;
double t, dt, dtheta, theta_observation, theta; //, dtheta_predicted

void setup() {
  Serial.begin(115200);
  time_old = millis();
  //delay(10);
}

void loop() {
  //double P, y_k, S_k, K_k;
  int pulses_per_rev = 60.0;
  //float Q_k = 0.001, R_k = 0.1;

  int newPosition = 0;
  int oldPosition = 0;
  int outputVal = 0;
  
      theta_observation = (2 * PI) * (newPosition / pulses_per_rev); //[rad]
      dtheta = newPosition - oldPosition;
      oldPosition = newPosition;
      t = millis();
      dt = (t - time_old) / 1000.0; //[sec]
      time_old = t;
      vel = dtheta/dt;
      
      //Serial.println(vel);
    
   // ---- KF ---- //
   p_kf = p_kf + q_kf;
   double y = vel - vel_kf;
   double s = p_kf + r_kf;
   double k = p_kf/s;
   vel_kf = vel_kf + k*y;
   p_kf = (1-k)*p_kf;

   
   vt =vel_kf;
     Serial.print("vt: ");
     Serial.print(vt);
     Serial.print(",  outputVal:");
     Serial.println(outputVal);

}

#include <Encoder.h>
#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <AutoPID.h>


#define OUTPUT_MIN -3250
#define OUTPUT_MAX 3250
#define KP 0.2
#define KI 0.4
#define KD 0

double vt, setPoint, outputVal;
double vel_kf = 0;
double p_kf = 1.0;
double r_kf = 1.1;
double q_kf = 0.5;


AutoPID myPID(&vt, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

const int dirPin = 2;
const int brakePin = 3;

Adafruit_MCP4728 mcp;
int ii = 0;
bool flag = false; 
long oldPosition  = -999;
double theta_old = 0.0, time_old; // theta_predict = 0.0, P_k = 0.0;
double D = 0.1651, vel = 0.0;
double t, dt, dtheta, theta_observation, theta; //, dtheta_predicted

Encoder myEnc(4, 5);

void setup() {
  // put your setup code here, to run once:
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
 
  mcp.setChannelValue(MCP4728_CHANNEL_A, 4095);
  mcp.setChannelValue(MCP4728_CHANNEL_B, 2048);
  mcp.setChannelValue(MCP4728_CHANNEL_C, 1024);
  mcp.setChannelValue(MCP4728_CHANNEL_D, 0);
  flag = true;
  time_old = millis();
  pinMode(dirPin, OUTPUT);
  pinMode(brakePin, OUTPUT);
  myPID.setTimeStep(100);
  delay(100);
}

void motor(int s){
  if(s > 10){
    mcp.setChannelValue(MCP4728_CHANNEL_A, 300 + s);
    digitalWrite(dirPin, LOW);
    digitalWrite(brakePin, HIGH);
  }
  if(s<-10){
    mcp.setChannelValue(MCP4728_CHANNEL_A, 300 - s);
    digitalWrite(dirPin, HIGH);
    digitalWrite(brakePin, HIGH);
  }
  if(s>-10 && s<10){
    mcp.setChannelValue(MCP4728_CHANNEL_A, 1);
    digitalWrite(dirPin, LOW);
    digitalWrite(brakePin, LOW);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //double P, y_k, S_k, K_k;
  int pulses_per_rev = 60.0;
  //float Q_k = 0.001, R_k = 0.1;
  
  long newPosition = myEnc.read();

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
   if (Serial.available()>0)
    {      
    int jj=Serial.parseInt();
    if(jj != 0){
      setPoint = double(jj);
    }
    
     }
     Serial.print("setPoint:");
     Serial.print(setPoint);
     Serial.print(",  vt:");
     Serial.print(vt);
     Serial.print(",  outputVal:");
     Serial.println(outputVal);

  myPID.run();
  motor(int(outputVal));  
  
}

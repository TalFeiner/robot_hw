volatile byte state1 = LOW;
volatile byte state2 = LOW;
volatile byte state3 = LOW;
volatile byte state4 = LOW;

byte pin1 = 2, pin2 = 3, pin3 = 6, pin4 = 7;

volatile bool flag1 = false, flag2 = false, flag3 = false, flag4 = false;

void cb1(){
  state1 = !state1;
  flag1 = true;
}

void cb2(){
  state2 = !state2;
  flag2 = true;
}

void cb3(){
  state3 = !state3;
  flag3 = true;
}

void cb4(){
  state4 = !state4;
  flag4 = true;
}

void setup() {
  Serial1.begin(115200);
  //ATmega2560 External Interrupts: 2 (interrupt 0), 3 (interrupt 1), 18 (interrupt 5), 19 (interrupt 4), 20 (interrupt 3), and 21 (interrupt 2) 
  pinMode(pin1, INPUT_PULLUP);
  pinMode(pin2, INPUT_PULLUP);
  pinMode(pin3, INPUT_PULLUP);
  pinMode(pin4, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(pin1), cb1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin2), cb2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin3), cb3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin4), cb4, CHANGE);
}

void loop() {
if (flag1) {
    flag1 = false;
    Serial1.println("flag1");
  }
  else {
    flag1 = true;
    Serial1.println("Interrupt 1 are not receiving signals");
  }
if (flag2) {
    flag2 = false;
    Serial1.println("flag2");
  }
  else {
    flag1 = true;
    Serial1.println("Interrupt 2 are not receiving signals");
  }
if (flag3) {
    flag3 = false;
    Serial1.println("flag3");
  }
  else {
    flag1 = true;
    Serial1.println("Interrupt 3 are not receiving signals");
  }
if (flag4) {
    flag4 = false;
    Serial1.println("flag4");
  }
  else {
    flag4 = true;
    Serial1.println("Interrupt 4 are not receiving signals");
  }
}

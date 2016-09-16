#include <Servo.h>

Servo servo1; Servo servo2; 


void setup() {

  pinMode(1,OUTPUT);
  servo1.attach(13); //analog pin 0
  //servo1.setMaximumPulse(2000);
  //servo1.setMinimumPulse(700);

  servo2.attach(12); //analog pin 1
  Serial.begin(115200);
  Serial.println("Ready");

}

void loop() {

  static int v = 0;
  
  static int rp = 90;
  static int lp = 90;
  
  int d = 1;

  if ( Serial.available()) {
    char ch = Serial.read();

    switch(ch) {
      case '0'...'9':
        v = v * 10 + ch - '0';
        break;
      case 's':
        servo1.write(rp);
        break;
      case 'a':
        rp = rp + d;
        if(rp > 160){
          rp = 160;
        }
        servo1.write(rp);
        break;
      case 'd':
        rp = rp - d;;
        if(rp < 20){
          rp = 20;
        }
        servo1.write(rp);
        break;
      case 'x':
        servo2.write(lp);
        break;
      case 'z':
        lp = lp + d;
        if(lp > 160){
          lp = 160;
        }
        servo2.write(lp);
        break;
      case 'c':
        lp = lp - d;
        if(lp < 20){
          lp = 20;
        }
        servo2.write(lp);
        break;
      default:
        break;
    }
    Serial.write(ch);
  }

  //Servo::refresh();

} 

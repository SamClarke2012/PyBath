#include <math.h>

String in = "";
char inBuffer[32];

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
}

void loop() {
  getTime();
}

int getTime() {
  if (Serial.available() > 0) {  
    for (int i = 0; i < Serial.available(); i++) {
      char c = Serial.read();
      if (c == '\n') {
        int time = toFloat(in) * 1000;
        digitalWrite(13, HIGH);
        delay(time);
        digitalWrite(13, LOW);
        in = "";
        return 1;
      }
     else if (c == '$') {
        Serial.println(thermistor(analogRead(0)));
      } 
     else if (c == '!') {
        Serial.println(21.00);
      } else {
        in += c;
      }
    }
  } else {
    return 0;
  }
}

float toFloat( String in ) {
  in.toCharArray(inBuffer, sizeof(inBuffer));
  return atof(inBuffer);
}

double thermistor(int RawADC) {
 double temp;
 double avg;
 for (int i = 0; i < 10000; i++) {
   temp = log(((10240000/RawADC) - 10000));
   temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp))* temp);
   temp = temp - 273.15;
   avg += temp; 
 }   
 return avg / 10000;
}

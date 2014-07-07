#include <DHT.h>
#include <math.h>

#define DHTPIN 2     
#define DHTTYPE DHT22 
#define THERMISTOR_SAMPLES 1000
#define DHT22_SAMPLES 1000

DHT env_temp(DHTPIN, DHTTYPE);
char inBuffer[128];
String in = "";

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  env_temp.begin();
}

void loop() {
  getTime();
}

int getTime() {
  if (Serial.available() > 0) { 
    for (int i = 0; i <= Serial.available(); i++) {
      char c = Serial.read();
      if (c == '\n') {
        unsigned long time = toLong(in);
        digitalWrite(13, HIGH);
        delay(time); // delay wants ms
        digitalWrite(13, LOW);
        in = "";
        return 1;
      }
     else if (c == '$') {
        Serial.println(thermistor(analogRead(0)));
        in = "";
      } 
     else if (c == '!') {
        double total;
        float t = env_temp.readTemperature();
        if (isnan(t)){
          in = "";
          continue;
        } else {
           Serial.println(t);
           in = "";
        }
      } else {
        in += c;
      }
    }
  } else {
    return 0;
  }
}

float toLong( String in ) {
  in.toCharArray(inBuffer, sizeof(inBuffer));
  return atol(inBuffer);
}

double thermistor(int RawADC) {
 double temp;
 double total;
 for (int i = 0; i < THERMISTOR_SAMPLES; i++) { //
   temp = log(((10240000/RawADC) - 10000));
   temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * temp * temp))* temp);
   temp = temp - 273.15;
   total += temp; 
   delayMicroseconds(200); // Approx 2x read time (reduce thermistor heating)
 }   
 return total / THERMISTOR_SAMPLES;
}

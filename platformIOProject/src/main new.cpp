
#include <Arduino.h>

const int motor1Pin3 = 19;
const int motor1Pin4 = 18;

const int tdsPin = 26;
float temperature = 25.0;

bool isTDSReached = false;



void setup() {
    Serial.begin(9600);
    pinMode(motor1Pin3, OUTPUT);
    pinMode(motor1Pin4, OUTPUT);
    // pinMode(enable1Pin, OUTPUT);
    // digitalWrite(enable1Pin, HIGH);


}


void loop() {



    float analogValue = analogRead(tdsPin);
    float voltage = analogValue * (3.3 / 4095.0);
    float ec = (133.42 * voltage * voltage * voltage
                  - 255.86 * voltage * voltage
                  + 857.39 * voltage);

    float ecValue25 = ec / (1.0 + 0.02 * (temperature - 25.0));  // temperature compensation
    float tdsValue = ecValue25 * 0.5; // TDS value in ppm, 0.5 being the TDS factor for General Hydroponics

    Serial.println(tdsValue);
    Serial.println(isTDSReached);

    if (isTDSReached == false) {
        Serial.println("here");
        digitalWrite(motor1Pin3, HIGH);
        digitalWrite(motor1Pin4, LOW);
        delay(500);

    }
    
    if (tdsValue > 200) {
        //STOP PUMP
        digitalWrite(motor1Pin3, LOW);
        digitalWrite(motor1Pin4, LOW);
        isTDSReached = true;

    }

    





}
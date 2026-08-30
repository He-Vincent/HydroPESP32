#include "DFRobot_PH.h"
#include <EEPROM.h>

int phPin = 34;

DFRobot_PH ph;

float ph_act;
float temperature = 21.5;

void setup() {
    Serial.begin(9600);

    analogReadResolution(12);
    analogSetPinAttenuation(phPin, ADC_11db);
    
    EEPROM.begin(32);


    ph.begin();
}

bool pollpHSensor() {

    float analogValue = analogRead(phPin);

    // Convert ESP32 12-bit ADC reading to millivolts
    float voltage = analogValue * (3300.0 / 4095.0);

    // Calculate pH using DFRobot library
    ph_act = ph.readPH(voltage, temperature);

    Serial.print("PH Raw ADC: ");
    Serial.println(analogValue);

    Serial.print("PH Voltage: ");
    Serial.print(voltage, 1);
    Serial.println(" mV");

    Serial.print("pH Val: ");
    Serial.println(ph_act, 2);

    ph.calibration(voltage, temperature);


    delay(100);

    return true;
}

void loop() {
    // float adc = analogRead(phPin);
    // float voltage = adc * 3300.0 / 4095.0;

    // Serial.print("ADC = ");
    // Serial.print(adc);

    // Serial.print("    Voltage = ");
    // Serial.print(voltage, 3);
    // Serial.println(" V");
    pollpHSensor();

    delay(1000);
}
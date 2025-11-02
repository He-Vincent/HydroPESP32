#include <Arduino.h>

float calibration_value = 21.34-0.08+0.90-0.89;
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[10],temp;
 
float ph_act;
// for the OLED display
 

int phPin = 34;
int pH_Value;
float Voltage;

#define NUM_SAMPLES 50
int readings[NUM_SAMPLES];


float getMedianReading(int sensorPin) {
  // Fill buffer with samples
  for (int i = 0; i < NUM_SAMPLES; i++) {
    readings[i] = analogRead(sensorPin);
    delay(10); // Short delay between readings
  }

  // Copy and sort the buffer
  int sorted[NUM_SAMPLES];
  memcpy(sorted, readings, sizeof(readings));

  // Simple bubble sort
  for (int i = 0; i < NUM_SAMPLES - 1; i++) {
    for (int j = 0; j < NUM_SAMPLES - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        int temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }

  // Return median
  if (NUM_SAMPLES % 2 == 0) {
    return (sorted[NUM_SAMPLES/2 - 1] + sorted[NUM_SAMPLES/2]) / 2.0;
  } else {
    return sorted[NUM_SAMPLES/2];
  }
}

 
void setup() 
{ 
  Serial.begin(9600);
  pinMode(pH_Value, INPUT); 
} 
 
void loop() 
{ 
  // pH_Value = analogRead(phPin); 
  // Voltage = pH_Value * (3.3 / 4096.0); 
  // Serial.println(Voltage); 
  // delay(500); 

  //   for(int i=0;i<10;i++) 
  // { 
  // buffer_arr[i]=analogRead(phPin);
  // delay(30);
  // }
  // for(int i=0;i<9;i++)
  // {
  // for(int j=i+1;j<10;j++)
  // {
  // if(buffer_arr[i]>buffer_arr[j])
  // {
  // temp=buffer_arr[i];
  // buffer_arr[i]=buffer_arr[j];
  // buffer_arr[j]=temp;
  // }
  // }
  // }
  // avgval=0;
  // for(int i=2;i<8;i++)
  // avgval+=buffer_arr[i];

  float analogValue = getMedianReading(phPin);
  float volt= analogValue *3.3/4096.0; 
  // ph_act = -5.70 * volt + calibration_value;
  // ph_act = -5.1006 * volt + 19.855;
  // ph_act = -4.7064 * volt + 18.987; 
  // ph_act = 1.4219 * volt * volt - 12.412 * volt + 29.134;
  ph_act = 1.3633*volt*volt -12.11 * volt +28.752;
  
  Serial.print("pH Val: ");
  Serial.print(ph_act, 3);

  Serial.println("");
  Serial.print("Volt: ");
  Serial.print(volt, 3);
  Serial.println("");




  delay(1000);
}
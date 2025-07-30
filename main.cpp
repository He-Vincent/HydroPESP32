// #include <esp_task_wdt.h>
// #define N 10                     // Number of readings to track
// #define THRESHOLD 0.5  // Max allowed variation to consider it settled

// #include <OneWire.h>
// #include <DallasTemperature.h>
// #include <Arduino.h>

// float tempHistory[N];
// int idx = 0;
// bool settled = false;


// // Data wire is plugged into port 4 on the Arduino
// #define ONE_WIRE_BUS 4
// // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// OneWire oneWire(ONE_WIRE_BUS);

// // Pass our oneWire reference to Dallas Temperature. 
// DallasTemperature sensors(&oneWire);


// // #define WDT_TIMEOUT 5  // Timeout in seconds




// void setup(void) {

//   // esp_task_wdt_init(WDT_TIMEOUT, true); // Enable panic so ESP32 resets
//   // esp_task_wdt_add(NULL);     
//   // start serial port
//   Serial.begin(9600);
  
//   // // Start up the library
//   sensors.begin();
  
//   // wdt_enable(WDTO_2S); // Enable watchdog timer with a 2-second timeout


// }

// // let's say 25 C is what we want
// // so if it's 25 +- 1 degree, that's fine


// void loop(void) { 
  
//   sensors.requestTemperatures(); // Send the command to get temperatures
//   // Serial.print("Celsius temperature: ");
//   // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
//   float currentTemp = sensors.getTempCByIndex(0);

//   tempHistory[idx] = currentTemp;
//   idx = (idx + 1) % N;

//   if (idx == 0) { // Only check after filling the buffer
//     float tMin = tempHistory[0];
//     float tMax = tempHistory[0];

//     for (int i = 1; i < N; i++) {
//       if (tempHistory[i] < tMin) tMin = tempHistory[i];
//       if (tempHistory[i] > tMax) tMax = tempHistory[i];
//     }

//     if ((tMax - tMin) < THRESHOLD) {
//       if (!settled) {
//         Serial.println("Temperature has settled.");
//         // Serial.println(currentTemp);
//         settled = true;
//       }
     

//     } else {
//       if (settled) {
//         Serial.println("Temperature is changing.");
//         // Serial.println(currentTemp);
//         settled = false;
//       }
//     }
//     Serial.println(currentTemp);
//   }

//   delay(500); // Read every 0.5 seconds (adjust if needed)

// }


//TODO:
//1. fix calibration of Ph
//2. Median Filter TDS
//3. Median Filter PH


#include <Arduino.h>
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 5  

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 4 on the board
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//pin 35 for tds
int tdsPin = 35; // ADC pin for TDS sensor
int phPin = 34; // ADC pin for pH sensor

float temperature = 25.0;  // Replace with real temp if available

float calibration_value = 21.34 - 0.7 + 0.46;
int phval = 0; 
unsigned long int avgval; 
int buffer_arr[20],temp;
 
float ph_act;


void setup() {
  Serial.begin(9600);


  // Initialize watchdog for the current task (loopTask)
  // 1st param = timeout, 2nd = panic on timeout?, 3rd = reset system?
  esp_task_wdt_init(WDT_TIMEOUT, true); 
  esp_task_wdt_add(NULL);  // Add current thread (loopTask) to WDT

  sensors.begin();
  // Init sensors here
  Serial.println("Setup complete.");
}


// Simulated sensor functions
// ph sensor

bool pollpHSensor() {
  for(int i=0;i<20;i++) 
  { 
    buffer_arr[i]=analogRead(phPin);
    delay(30);
  }
  for(int i=0;i<19;i++)
  {
    for(int j=i+1;j<20;j++)
    {
      if(buffer_arr[i]>buffer_arr[j])
      {
        temp=buffer_arr[i];
        buffer_arr[i]=buffer_arr[j];
        buffer_arr[j]=temp;
      }
    }
  }
  avgval=0;
  for(int i=12;i<18;i++) {
    avgval+=buffer_arr[i];
  }
  float volt=(float)avgval*((3.3 / 4095.0))/6.0; 
   ph_act = -5.70 * volt + calibration_value;
  
  Serial.print("pH Val: ");
  Serial.println(ph_act);

  delay(100);
  return true;

}

bool polltempSensor() {
  sensors.requestTemperatures();
  float currentTemp = sensors.getTempCByIndex(0);
  Serial.print("Celsius temperature: ");
  Serial.println(currentTemp);
  temperature = currentTemp; // Update global temperature variable
  delay(100);
  return true;
}


bool pollTDSSensor() {
  int analogValue = analogRead(tdsPin);  // ADC pin
  float voltage = analogValue * (3.3 / 4095.0);
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;

  float tdsValue = (133.42 * pow(compensationVoltage, 3)
                  - 255.86 * pow(compensationVoltage, 2)
                  + 857.39 * compensationVoltage) * 0.5;

  Serial.print("TDS (ppm): ");
  Serial.println(tdsValue);

  delay(100);
  return true;
}


void loop() {
    // Poll temp sensor first, since can use temperature for compensation in TDS and pH calculations
    if (!polltempSensor()) {
      Serial.println("temp sensor failed!");
    }
  

  // Poll ph sensor
  if (!pollpHSensor()) {
    Serial.println("ph sensor failed!");
  }


  // Poll sensor 3
  if (!pollTDSSensor()) {
    Serial.println("TDS sensor failed!");
  }

  // Feed the watchdog if everything is ok
  esp_task_wdt_reset();

  delay(1000); // Wait 1 second
}
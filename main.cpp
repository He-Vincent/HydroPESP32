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



#include <Arduino.h>
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 10 

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 4 on the board
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


int tdsPin = 26; // ADC pin for TDS sensor
int phPin = 34; // ADC pin for pH sensor

float temperature = 25.0;  // Replace with real temp if available

float calibration_value = 21.34 - 0.7 + 0.46;
float avgval; 
float buffer_arr[20],temp;
 
float ph_act;

float beginningTDS = 0.0; // Initial TDS value
float tdsValue = 0.0; // Current TDS value

float tdsRequired = 0.0; // sum of beginning + TDS solution PPM based on growth 
// need tds of water + solution PPM based on growth stage
// could hardcore water ppm


// pumps 
//red to out1, out3
// blue to out2, out4

//solution pump
int tdsPumpIn1Pin = 22;
int tdsPumpIn2Pin = 21; 
int tdsPumpEnPin = 23;

// ph down pump
int phPumpIn3Pin = 18; 
int phPumpIn4Pin = 5;  
int phPumpEnPin = 19; 





// pumps



// Function prototypes

//median filter
#define NUM_SAMPLES 10

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



bool pollpHSensor() {
  // for(int i=0;i<20;i++) 
  // { 
  //   buffer_arr[i]=analogRead(phPin);
  //   delay(30);
  // }
  // for(int i=0;i<19;i++)
  // {
  //   for(int j=i+1;j<20;j++)
  //   {
  //     if(buffer_arr[i]>buffer_arr[j])
  //     {
  //       temp=buffer_arr[i];
  //       buffer_arr[i]=buffer_arr[j];
  //       buffer_arr[j]=temp;
  //     }
  //   }
  // }
  // avgval=0;
  // for(int i=12;i<18;i++) {
  //   avgval+=buffer_arr[i];
  // }

  float analogValue = getMedianReading(phPin); // median filtered pH for 10 samples
  float volt=(float)analogValue*((3.3 / 4095.0)); // Convert ADC value to voltage
  ph_act = -5.70 * volt + calibration_value;
  
  // Serial.print("Raw ADC: ");
  // Serial.print(analogValue);
  // Serial.print("  Voltage: ");
  // Serial.println(volt, 3);
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
void runTDSPump(){
  analogWrite(tdsPumpEnPin, 255); // Enable pump
  digitalWrite(tdsPumpIn1Pin, LOW); // Set pump direction
  digitalWrite(tdsPumpIn2Pin, HIGH); // Set pump direction
}

void stopTDSPump() {
  analogWrite(tdsPumpEnPin, 0); // Disable pump
  digitalWrite(tdsPumpIn1Pin, LOW); // Set pump direction
  digitalWrite(tdsPumpIn2Pin, LOW); // Set pump direction
}

bool pollTDSSensor() {
  float analogValue = getMedianReading(tdsPin);  // median filtered TDS for 10 samples
  float voltage = analogValue * (3.3 / 4095.0);
  Serial.print("Raw ADC: ");
  Serial.print(analogValue);
  Serial.print("  Voltage: ");
  Serial.println(voltage, 3);

  



  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;

  tdsValue = (133.42 * pow(compensationVoltage, 3)
                  - 255.86 * pow(compensationVoltage, 2)
                  + 857.39 * compensationVoltage) * 0.5;

  Serial.print("required tds");
  Serial.println(tdsRequired);

  Serial.print("TDS (ppm): ");
  Serial.println(tdsValue);

  if (tdsValue < tdsRequired) {
    // run pump 
    runTDSPump();
    delay(5000); // Run pump for 5 seconds
    stopTDSPump(); // Stop pump after adding nutrients
  }

  delay(100);
  return true;
}



typedef enum {
  SEEDLING,
  EARLY_GROWTH,
  LATE_GROWTH,
} GrowState;

float solutionPPM = 0.0;

GrowState currentState = SEEDLING; // Initial state

float ppmTapWater = 6.0; // PPM of tap water from my house




void setup() {
  //pinmode for tds pump
  pinMode(tdsPumpIn1Pin, OUTPUT);
  pinMode(tdsPumpIn2Pin, OUTPUT);
  pinMode(tdsPumpEnPin, OUTPUT);

  //pinmode for ph pump

  // Turn off motors - Initial state
  digitalWrite(tdsPumpIn1Pin, LOW);
  digitalWrite(tdsPumpIn2Pin, LOW);


  Serial.begin(9600);


  

  sensors.begin();
  // Init sensors here


  // check the tds here in the beginning and store
  
  // for(int i = 0; i < 10; i++) {
  //   pollTDSSensor(); // Get initial TDS value
  //   beginningTDS = tdsValue; // Stores initial TDS value
   
  //   delay(1000);


  // }

  //choose the proper solution PPM based on the current growth stage
  // based on General Hydroponics FloraNova Grow nutrient solution
  switch(currentState) {
    case SEEDLING:
      solutionPPM = 500; // PPM for seedling stage, 0.5mL/L
      break;
    case EARLY_GROWTH:
      solutionPPM = 1250; // PPM for early growth stage, 1.25mL/L
      break;
    case LATE_GROWTH:
      solutionPPM = 2500; // PPM for late growth stage, 2.5mL/L
      break;
  }


  tdsRequired = ppmTapWater + solutionPPM; // Calculate required TDS based on initial TDS and growth stage
  Serial.print("Required TDS: ");
  Serial.println(tdsRequired);
  
  // Initialize watchdog for the current task (loopTask)
  // 1st param = timeout, 2nd = panic on timeout?, 3rd = reset system?
  esp_task_wdt_init(WDT_TIMEOUT, true); 
  esp_task_wdt_add(NULL);  // Add current thread (loopTask) to WDT
  Serial.println("Setup complete.");
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


  // // Poll sensor 3
  if (!pollTDSSensor()) {
    Serial.println("TDS sensor failed!");
  }

  // Feed the watchdog if everything is ok
  esp_task_wdt_reset();

  delay(1000); // Wait 1 second
}
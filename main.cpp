//TODO:
// 15 minutes after adding nutrient to then check tds
// 15 minutes after adding ph down to then check ph

// include the library
#include <LiquidCrystal.h>
#include <GravityTDS.h>

// Creates an LCD object. Parameters: (rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(13, 2, 25, 32, 27, 14);
 
#define SENSOR_POLL_INTERVAL 1000 // 1 second cuz too lazy to wait 
#define BLYNK_SEND_INTERVAL  15000 

#include <Arduino.h>
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 100
// 
#include <OneWire.h>
#include <DallasTemperature.h>

#define BLYNK_TEMPLATE_ID "TMPL2BO6Ir-cT"
#define BLYNK_TEMPLATE_NAME "HydroP"
#define BLYNK_AUTH_TOKEN "ZisX61goKro1iq8lepCJnPokUHJ_hH2j"
#define BLYNK_FIRMWARE_VERSION        "0.1.0"

#include <BlynkSimpleEsp32.h>  // Blynk core
#include <SimpleTimer.h>
SimpleTimer timer;




#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG

#define APP_DEBUG

// #include "C:\Users\Vincent\Documents\PlatformIO\Projects\HydroPESP32\.pio\libdeps\upesy_wroom\Blynk\examples\Blynk.Edgent\Edgent_ESP32\BlynkEdgent.h"


#define PH_PIN V0
#define TDS_PIN V1
#define TEMP_PIN V2




// Data wire is plugged into port 4 on the board
#define ONE_WIRE_BUS 4
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


int tdsPin = 33; // ADC pin for TDS sensor
int phPin = 34; // ADC pin for pH sensor

float temperature = 21.9;  // Replace with real temp if available

float calibration_value = 21.34; 
float avgval; 
float buffer_arr[20],temp;
 
float ph_act;

float beginningTDS = 0.0; // Initial TDS value
float tdsValue = 0.0; // Current TDS value

float tdsRequired = 0.0; // sum of beginning + TDS solution PPM based on growth 
// need tds of water + solution PPM based on growth stage
// could hardcore water ppmx  

float phRequired = 6.0;


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


const int N = 5;
float voltages[N] = {0.01, 0.135, 0.43, 0.477, 0.812};
float ppmVals[N] = {0, 993, 3499, 1210, 398};




void printPH(float ph) {
  lcd.setCursor(0,1);
  lcd.print(ph);
}
void printPPM(float tds){
  lcd.setCursor(5,1);
  lcd.print(tds);

}

void printTempC(float temp) {
  lcd.setCursor(11,1);
  lcd.print(temp);
  // lcd.print("C");

}

void clearPH() {
  lcd.setCursor(0,1);
  lcd.print("     "); // 5 spaces
}

void clearPPM() {
  lcd.setCursor(5,1);
  lcd.print("      "); //6 spaces
}

void clearTempC() {
  lcd.setCursor(11,1);
  lcd.print("     "); //5 spaces
}

// pumps



// Function prototypes

//median filter
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


void runPHPump(){
  analogWrite(phPumpEnPin, 255); // Enable pump
  digitalWrite(phPumpIn3Pin, LOW); // Set pump direction
  digitalWrite(phPumpIn4Pin, HIGH); // Set pump direction
}

void stopPHPump() {
  analogWrite(phPumpEnPin, 0); // Disable pump
  digitalWrite(phPumpIn3Pin, LOW); // Set pump direction
  digitalWrite(phPumpIn4Pin, LOW); // Set pump direction
}


bool pollpHSensor() {

  float analogValue = getMedianReading(phPin); // median filtered pH for 10 samples
  float volt=(float)analogValue*((3.3 / 4095.0)); // Convert ADC value to voltage
  ph_act = -5.70 * volt + calibration_value;
  
  // Serial.print("Raw ADC: ");
  // Serial.print(analogValue);
  // Serial.print("  Voltage: ");
  // Serial.println(volt, 3);
  
  clearPH(); // Clear previous pH display
  printPH(ph_act); // Print pH value to LCD
  Serial.print("pH Val: ");
  Serial.println(ph_act);


  // if (ph_act > phRequired) {
  //   Serial.println("PH PUMP");
  //   // runPHPump(); // Run pH down pump
  //   delay(1000); // Run pump 
  //   stopPHPump(); // Stop pump after adjusting pH
  // }

  // delay(100);
  return true;

}

bool polltempSensor() {
  sensors.requestTemperatures();
  float currentTemp = sensors.getTempCByIndex(0);
  Serial.print("Celsius temperature: ");
  Serial.println(currentTemp);
  clearTempC(); // Clear previous temperature display
  printTempC(currentTemp); // Print temperature to LCD
  // temperature = currentTemp; // Update global temperature variable


 
  // delay(100);
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
  Serial.print("TDS Raw ADC: ");
  Serial.print(analogValue);
  Serial.print("  TDS Voltage: ");
  Serial.println(voltage, 3);

//   if (voltage <= voltages[0])  tdsValue = ppmVals[0];
//   if (voltage >= voltages[N-1])  tdsValue = ppmVals[N-1];

  
//   for (int i = 0; i < N-1; i++) {
//     if (voltage >= voltages[i] && voltage <= voltages[i+1]) {
//         float fraction = (voltage - voltages[i]) / (voltages[i+1] - voltages[i]);
//         tdsValue = ppmVals[i] + fraction * (ppmVals[i+1] - ppmVals[i]);
//     }
// }
// tdsValue = 0.0; // Default value if no match found

  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;

  // tdsValue = tdsValue / compensationCoefficient;

  // tdsValue 


  // tdsValue = 1097.2 - 1213.8 * voltage + 533.5 * voltage * voltage;
  // tdsValue = tdsValue * 1.077;
  // float tdsValue = 3.5 - 7250 * voltage + 4470 * voltage * voltage;
  // if (tdsValue < 0) tdsValue = 0;



  tdsValue = (133.42 * pow(compensationVoltage, 3)
                  - 255.86 * pow(compensationVoltage, 2)
                  + 857.39 * compensationVoltage) * 0.5;

  
  // Serial.print("required tds");
  // Serial.println(tdsRequired);
  clearPPM(); // Clear previous TDS display
  printPPM(tdsValue); // Print TDS value to LCD
  Serial.print("TDS (ppm): ");
  Serial.println(tdsValue);



  // if (tdsValue < tdsRequired) {
  //   // run pump 
  //   Serial.println("TDS PUMP");
  //   // runTDSPump();
  //   delay(1000); 
  //   stopTDSPump(); // Stop pump after adding nutrients
  // }

  // delay(100);
  return true;
}

void sendTDS(){
  Blynk.virtualWrite(V1, tdsValue); // Send processed value
}

void sendPH(){
  Blynk.virtualWrite(V0, ph_act); // Send processed value
}

void sendTemp(){
  Blynk.virtualWrite(V2, temperature); // Send processed value
}


typedef enum {
  SEEDLING,
  EARLY_GROWTH,
  LATE_GROWTH,
} GrowState;

float solutionPPM = 0.0;

GrowState currentState = SEEDLING; // Initial state

float ppmTapWater = 6.0; // PPM of tap water from my house

void pollSensors() {
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
  // Add any additional sensor polling here
}

void sendDataToBlynk() {
  sendPH(); // Send pH value to Blynk
  sendTDS(); // Send TDS value to Blynk
  sendTemp(); // Send temperature value to Blynk

}

void setup() {

  analogReadResolution(12);

  // Set ADC attenuation for this pin to 11 dB (0–3.3 V range)
  analogSetPinAttenuation(TDS_PIN, ADC_11db);

    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);

    // Clears the LCD screen
    lcd.clear();
  
    lcd.setCursor(0,0);
    lcd.print("pH");

    lcd.setCursor(5,0);
    lcd.print("PPM");

    lcd.setCursor(11,0);
    lcd.print("TempC");



  //pinmode for tds pump
  pinMode(tdsPumpIn1Pin, OUTPUT);
  pinMode(tdsPumpIn2Pin, OUTPUT);
  pinMode(tdsPumpEnPin, OUTPUT);

  //pinmode for ph pump
  pinMode(phPumpIn3Pin, OUTPUT);
  pinMode(phPumpIn4Pin, OUTPUT);
  pinMode(phPumpEnPin, OUTPUT);

  // Turn off pumps - Initial state
  digitalWrite(tdsPumpIn1Pin, LOW);
  digitalWrite(tdsPumpIn2Pin, LOW);
  digitalWrite(tdsPumpEnPin, LOW);

  digitalWrite(phPumpIn3Pin, LOW);
  digitalWrite(phPumpIn4Pin, LOW);
  digitalWrite(phPumpEnPin, LOW);

  

  Blynk.begin(BLYNK_AUTH_TOKEN,"felixdaddy 2.4G", "20041997");
    // timer.setInterval(1000L, sendProcessedValue); // every 1s


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

  // timer.setInterval(1000L, sendPH);
  // timer.setInterval(1000L, sendTDS);
  // timer.setInterval(1000L, sendTemp);

  timer.setInterval(SENSOR_POLL_INTERVAL, pollSensors);
timer.setInterval(BLYNK_SEND_INTERVAL, sendDataToBlynk);
  // Initialize watchdog for the current task (loopTask)
  // 1st param = timeout, 2nd = panic on timeout?, 3rd = reset system?
  esp_task_wdt_init(WDT_TIMEOUT, true); 
  esp_task_wdt_add(NULL);  // Add current thread (loopTask) to WDT
  Serial.println("Setup complete.");
}




void loop() {
  // BlynkEdgent.run();
  // Blynk.run();
  timer.run();

  //  // Print a message to the LCD.
  //  lcd.print(" Hello world!");

  //  // set the cursor to column 0, line 1
  //  // (note: line 1 is the second row, since counting begins with 0):
  //  lcd.setCursor(0, 1);
  //  // Print a message to the LCD.
  //  lcd.print(" LCD Tutorial");
}



// *** MAIN SETTINGS ***
// Replace this block with correct template settings.
// You can find it for every template here:
//
//   https://blynk.cloud/dashboard/templates

// #define BLYNK_TEMPLATE_ID "TMPL2XEiECiGL"
// #define BLYNK_TEMPLATE_NAME "LED ESP32 1"






// #define LED_PIN 2  // Use pin 2 for LED (change it, if your board uses another pin)


// // V0 is a datastream used to transfer and store LED switch state.
// // Evey time you use the LED switch in the app, this function
// // will listen and update the state on device
// BLYNK_WRITE(V0)
// {
//   // Local variable `value` stores the incoming LED switch state (1 or 0)
//   // Based on this value, the physical LED on the board will be on or off:
//   int value = param.asInt();

//   if (value == 1) {
//     digitalWrite(LED_PIN, HIGH);
//     Serial.print("value =");
//     Serial.println(value);
//   } else {
//     digitalWrite(LED_PIN, LOW);
//     Serial.print("value = ");
//     Serial.println(value);
//   }
// }
// void setup()
// {
//   pinMode(LED_PIN, OUTPUT);

//   // Debug console. Make sure you have the same baud rate selected in your serial monitor
//   Serial.begin(115200);
//   delay(100);

//   BlynkEdgent.begin();
// }

// void loop() {
//   BlynkEdgent.run();
//   delay(10);
// }


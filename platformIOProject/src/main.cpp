//TODO:
//1. fix calibration of Ph

// include the library
#include <LiquidCrystal.h>

// Creates an LCD object. Parameters: (rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(13, 2, 25, 32, 27, 14);

#include <Arduino.h>
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 100

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
  ph_act = -4.8176 * volt + 19.266;
  

  
  Serial.print("PH Raw ADC: ");
  Serial.print(analogValue);
  Serial.print("PH   Voltage: ");
  Serial.println(volt, 3);
  
  clearPH(); // Clear previous pH display
  printPH(ph_act); // Print pH value to LCD
  Serial.print("pH Val: ");
  Serial.println(ph_act);

  if (ph_act > phRequired) {
    Serial.println("PH PUMP");
    // runPHPump(); // Run pH down pump
    delay(1000); // Run pump 
    stopPHPump(); // Stop pump after adjusting pH
  }

  delay(100);
  return true;

}

bool polltempSensor() {
  sensors.requestTemperatures();
  float currentTemp = sensors.getTempCByIndex(0);
  Serial.print("Celsius temperature: ");
  Serial.println(currentTemp);
  clearTempC(); // Clear previous temperature display
  printTempC(currentTemp); // Print temperature to LCD
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
  // Serial.print("Raw ADC: ");
  // Serial.print(analogValue);
  // Serial.print("  Voltage: ");
  // Serial.println(voltage, 3);
  // float kValue = 0.98;
  // float kValue = 0.97;
  float kValue = 0.922;

  float ec = (133.42 * voltage * voltage * voltage
                  - 255.86 * voltage * voltage
                  + 857.39 * voltage) * kValue; 

  Serial.print("TDS :");
  Serial.println(ec);



  float ecValue25 = ec / (1.0 + 0.02 * (temperature - 25.0));  // temperature compensation

  Serial.print("TDS (ec adjusted): ");
  Serial.println(ecValue25);

  float tdsValue = ecValue25 * 0.5; // TDS value in ppm, 0.5 being the TDS factor for general hydroponics

  // float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  // float compensationVoltage = voltage / compensationCoefficient;

  // tdsValue = (133.42 * pow(compensationVoltage, 3)
  //                 - 255.86 * pow(compensationVoltage, 2)
  //                 + 857.39 * compensationVoltage) * 0.9;

  

  // Serial.print("required tds");
  // Serial.println(tdsRequired);
  clearPPM(); // Clear previous TDS display
  printPPM(tdsValue); // Print TDS value to LCD
  Serial.print("TDS (ppm): ");
  Serial.println(tdsValue);

  // if (tdsValue < tdsRequired) {
    // run pump 
    // Serial.println("TDS PUMP");
    // runTDSPump();
    // delay(1000); 
    // stopTDSPump(); // Stop pump after adding nutrients
  // }

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

  //  // Print a message to the LCD.
  //  lcd.print(" Hello world!");

  //  // set the cursor to column 0, line 1
  //  // (note: line 1 is the second row, since counting begins with 0):
  //  lcd.setCursor(0, 1);
  //  // Print a message to the LCD.
  //  lcd.print(" LCD Tutorial");


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
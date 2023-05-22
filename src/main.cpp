#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <DHT.h>
#include <DS1302.h>
#include <LiquidCrystal_I2C.h>
#include <ClickEncoder.h>
#include <TimerOne.h>

#define DEBOUNCE_TIME 50 // ms
#define SENSOR_CHECK_INTERVAL 50        // check sensors every 50ms
#define SW_CHECK_INTERVAL 50            // check SW every 50ms
#define RTC_CHECK_INTERVAL 1000         // check RTC every 1s
#define INTERACTION_TIMEOUT 5000        // timeout for interaction: 5s
#define LONG_PRESS_TIME 1000            // Time to consider a press as "long press"
#define SHORT_PRESS_TIME 100            // Time to consider a press as "short press"
#define ROTARY_ENCODER_INTERVAL 50      // check rotary encoder every 50ms
//BYTE ICON DEFINITIONS
uint8_t vent[8] = {0x00,0x00,0x19,0x0B,0x04,0x1A,0x13,0x00}; //I will reference going forward this as vent1 icon
uint8_t secondVent[8] = {0x00,0x00,0x0C,0x05,0x1F,0x14,0x06,0x00}; //I will reference going forward this as vent2 icon
uint8_t heat[8] = {0x0E,0x0A,0x0A,0x0E,0x0E,0x1F,0x1F,0x0E}; //I will reference going forward this as heat icon
uint8_t humid[8] = {0x00,0x04,0x0E,0x0E,0x1F,0x1F,0x1F,0x0E}; //I will reference going forward this as humid icon
uint8_t wifi[8] = {0x00,0x0E,0x11,0x04,0x0A,0x00,0x04,0x00}; //I will reference going forward this as wifi icon
uint8_t lightsOn[8] = {0x0E,0x1F,0x1F,0x1F,0x0E,0x0E,0x0E,0x04}; //I will reference going forward this as lightsOn icon
uint8_t lightsOff[8] = {0x0E,0x11,0x11,0x11,0x0A,0x0E,0x0E,0x04}; //I will reference going forward this as lightsOff icon
uint8_t watLevel[8] = {0x00,0x11,0x11,0x1F,0x1F,0x1F,0x1F,0x0E}; //I will reference going forward this as waterLevel icon
// Pin Definitions
#define DHT11_SENSOR_PIN       GPA0
#define POWER_SUPPLY_SWITCH    GPA1
#define WATER_PUMP_PIN         GPA2
#define VENTILATOR_PIN         GPA3
#define LIGHTS_RELE_PIN        GPA4
#define ROTARY_ENCODER_SW_PIN  GPA5
#define BUZZER_PIN             GPB0
#define LOW_WATER_INDICATOR    GPB1
#define WIFI_STATUS_PIN        GPB2
// NodeMCU Pin Connections:
#define SCL_PIN                D1
#define SDA_PIN                D2
#define RTC_CLK_PIN            D5
#define RTC_DAT_PIN            D6
#define RTC_RST_PIN            D7
#define SOIL_MOISTURE_SENSOR   A0
#define ROTARY_ENCODER_CLK_PIN D3
#define ROTARY_ENCODER_DT_PIN  D4
//STRUCTS DEFINITIONS
union bandera_t {
  struct {
    unsigned bit0 : 1;
    unsigned bit1 : 1;
    unsigned bit2 : 1;
    unsigned bit3 : 1;
    unsigned bit4 : 1;
    unsigned bit5 : 1;
    unsigned bit6 : 1;
    unsigned bit7 : 1;
  } unionBit;
  byte unionByte;
} bandera; 
bandera flag1;
bandera flag2;
//FLAG 1 DEFINITIONS EXAMPLE
//Controls the state of the system
#define LCD_ON flag1.unionBit.bit0 //LCD ON/OFF
#define HOURPASSED flag1.unionBit.bit1 //Hour passed flag
#define VEGSTAGE flag1.unionBit.bit2 //In Vegetation? flag
#define LIGHTS_ON flag1.unionBit.bit3 //Leds ON/OFF
#define VENT_ON flag1.unionBit.bit4 //Vent ON/OFF
#define WATER_ON flag1.unionBit.bit5 //Water ON/OFF
#define BUZZER_ON flag1.unionBit.bit6 //Buzzer ON/OFF
#define LOW_WATER_ON flag1.unionBit.bit7 //Power ON/OFF
#define SWPRESSED flag2.unionBit.bit0 //SW pressed flag
#define SWRELEASED flag2.unionBit.bit1 //SW released flag
// Structure to store stage-specific parameters
struct StageParams {
  uint8_t lightIntensityMin;    // in µmol/m²/s
  uint8_t lightIntensityMax;    // in µmol/m²/s
  uint8_t temperatureMin;       // in °C
  uint8_t temperatureMax;       // in °C
  uint8_t humidityMin;          // in % RH
  uint8_t humidityMax;          // in % RH
  uint8_t lightCycleOn;         // in hours
  uint8_t lightCycleOff;        // in hours
  uint8_t wateringTime;         // in seconds
  uint8_t wateringInterval;     // in hours
  uint8_t ventilationTime;      // in seconds
  uint8_t stageDurationMin;     // in weeks
  uint8_t stageDurationMax;     // in weeks
  uint8_t soilMoistureMin;      // in %
  uint8_t soilMoistureMax;      // in %
};
// Default parameters for each stage
StageParams seeding = {
  200, 400, 22, 26, 70, 80, 18, 6, 5, 4, 30, 1, 3, 70, 80
};
StageParams vegetative = {
  600, 1000, 20, 30, 40, 70, 18, 6, 5, 4, 30, 4, 8, 60, 70
};
StageParams flowering = {
  600, 1000, 20, 28, 40, 50, 12, 12, 5, 4, 30, 6, 12, 50, 60
};
// Custom parameters (modifiable by user)
StageParams custom = vegetative;
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
// Instantiate the MCP23017
Adafruit_MCP23017 mcp;
// Instantiate the RTC
DS1302 rtc(RTC_RST_PIN, RTC_DAT_PIN, RTC_CLK_PIN);
Time previousTime;
Time currentTime;
// Instantiate the DHT sensor
DHT dht(DHT11_SENSOR_PIN, DHT11);
// Instantiate the LCD
Adafruit_LiquidCrystal lcd(SDA_PIN, SCL_PIN);
ClickEncoder *encoder;
int16_t last, value;
//Variables definitions 
uint32_t lastSensorCheckMillis = 0;
uint32_t lastSWCheckMillis = 0;
uint32_t lastRotaryEncoderCheckMillis = 0;
uint32_t lastRTCCheckMillis = 0;
uint32_t lastInteractionMillis = 0;
uint32_t lastPressTime = 0; // Record last time the button was pressed
uint32_t lastReleaseTime = 0; // Record last time the button was released
uint8_t lightsOnHours;
uint8_t lightsOffHours;
uint8_t timeToNextWatering;
uint8_t timerWatering;
uint8_t timerVentilation;
//End of variables definitions
struct MenuItem {
  const char* name;
  void (*action)();
  MenuItem* submenu;
};
// Define the menu structure
MenuItem menu[] = {
  {"HOME", nullptr, nullptr},
  {"WIFIDATA", nullptr, (MenuItem[]){
    {"WIFIINFO", displayWifiInfo, nullptr},
    {"WIFISETTINGS", displayWifiSettings, nullptr},
    {"CHECKAPPCONNECTION", checkAppConnection, nullptr},
    {"VOLVER", nullptr, &menu[1]} // Return to parent item "WIFIDATA"
  }},
  {"SETTINGS", nullptr, (MenuItem[]){
    {"MODIFYSTARTINGHOUR", modifyStartingHour, nullptr},
    {"MODIFYLIGHTS", nullptr, (MenuItem[]){
      {"TIME ON", modifyLightsOnTime, nullptr},
      {"TIME OFF", modifyLightsOffTime, nullptr},
      {"VOLVER", nullptr, &menu[2]} // Return to parent item "MODIFYLIGHTS"
    }},
    {"MODIFYWATERING", nullptr, (MenuItem[]){
      {"HOURSBETWEENWATERING", modifyWateringInterval, nullptr},
      {"WATERINGTIME", modifyWateringTime, nullptr},
      {"WATERINGAMOUNT", modifyWateringAmount, nullptr},
      {"WATERREFILL", refillWater, nullptr},
      {"VOLVER", nullptr, &menu[2]} // Return to parent item "MODIFYWATERING"
    }},
    {"STAGE", nullptr, (MenuItem[]){
      {"CURRENTSTAGE", displayCurrentStage, nullptr},
      {"CHANGESTAGE", changeStage, nullptr},
      {"VOLVER", nullptr, &menu[2]} // Return to parent item "STAGE"
    }},
    {"VOLVER", nullptr, &menu[0]} // Return to parent item "HOME"
  }},
  {"ABOUT", nullptr, (MenuItem[]){
    {"VERSION", displayVersion, nullptr},
    {"VOLVER", nullptr, &menu[1]} // Return to parent item "SETTINGS"
  }},
  {"VOLVER", displayHomeInformation, nullptr} // End of menu structure
};
void timerIsr() { // Timer interrupt service routine for the rotary encoder
  encoder->service();
}

void setup() {
  // initialize the serial communication for debugging
  Serial.begin(9600);
  // initialize the MCP23017
  mcp.begin(); 
  // initialize the GPIO as output
  mcp.pinMode(DHT11_SENSOR_PIN, OUTPUT);
  mcp.pinMode(POWER_SUPPLY_SWITCH, OUTPUT);
  mcp.pinMode(WATER_PUMP_PIN, OUTPUT);
  mcp.pinMode(VENTILATOR_PIN, OUTPUT);
  mcp.pinMode(LIGHTS_RELE_PIN, OUTPUT);
  mcp.pinMode(BUZZER_PIN, OUTPUT);
  mcp.pinMode(LOW_WATER_INDICATOR, OUTPUT);
  mcp.pinMode(WIFI_STATUS_PIN, OUTPUT);
  // initialize the GPIO as input
  mcp.pinMode(ROTARY_ENCODER_SW_PIN, INPUT);
  // initialize DHT sensor
  dht.begin();
  // initialize RTC
  rtc.begin();
  // initialize LCD
  lcd.begin(16, 2);
  // Load custom characters into LCD memory
  lcd.createChar(0, vent);
  lcd.createChar(1, secondVent);
  lcd.createChar(2, heat);
  lcd.createChar(3, humid);
  lcd.createChar(4, wifi);
  lcd.createChar(5, lightsOn);
  lcd.createChar(6, lightsOff);
  lcd.createChar(7, watLevel);
  // NodeMCU pins setup
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, OUTPUT);
  pinMode(RTC_CLK_PIN, OUTPUT);
  pinMode(RTC_DAT_PIN, OUTPUT);
  pinMode(RTC_RST_PIN, OUTPUT);
  pinMode(SOIL_MOISTURE_SENSOR, INPUT);
  pinMode(ROTARY_ENCODER_CLK_PIN, INPUT);
  pinMode(ROTARY_ENCODER_DT_PIN, INPUT);
  encoder = new ClickEncoder(ROTARY_ENCODER_DT_PIN, ROTARY_ENCODER_CLK_PIN, ROTARY_ENCODER_SW_PIN);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);
  last = -1;
  currentTime = previousTime = rtc.getTime();
  lightsOffHours = custom.lightCycleOff;
  lightsOnHours = custom.lightCycleOn;
  timerWatering = custom.wateringTime;
  timerVentilation = custom.ventilationTime;
  timeToNextWatering = custom.water;
}

void loop() {
  uint32_t currentMillis = millis();
  //TIMING WITH MILLIS 
  // Check SW every swCheckInterval
  if (currentMillis - lastSWCheckMillis >= DEBOUNCE_TIME) {
    lastSWCheckMillis = currentMillis;
    checkSW();
  }
  // Check button state every rotryEncoderCheckInterval
  if (currentMillis - lastRotaryEncoderCheckMillis >= ROTARY_ENCODER_INTERVAL) {
    lastRotaryEncoderCheckMillis = currentMillis;
    value += encoder->getValue();
  }
  // Check sensors every sensorCheckInterval
  if (currentMillis - lastSensorCheckMillis >= SENSOR_CHECK_INTERVAL) {
    lastSensorCheckMillis = currentMillis;
    checkSensors();
  }
  // Check RTC every rtcCheckInterval
  if (currentMillis - lastRTCCheckMillis >= RTC_CHECK_INTERVAL) {
    lastRTCCheckMillis = currentMillis;
    checkRTC();
  }
  // Check interaction timeout
  if (currentMillis - lastInteractionMillis >= INTERACTION_TIMEOUT && LCD_ON) {
    lastInteractionMillis = currentMillis;
    turnOffLCD();
  }
  if(HOURPASSED){
    if(LIGHTS_ON){
      
    }
    HOURPASSED = 0;
    checkLights();
    checkWatering();
  }
}

void checkLights(){
  if()
}

void checkWatering(){

}

void checkSensors() {
  // Read and process sensor data here
  // ...
}

void checkSW() { // Check SW state and handle short and long press
  uint8_t swState = mcp.digitalRead(ROTARY_ENCODER_SW_PIN);
  if(swState && !SWPRESSED){
    SWPRESSED = 1;
    lastPressTime = millis();
  }else if(!swState && SWPRESSED){
    SWRELEASED = 1;
    SWPRESSED = 0;
    lastReleaseTime = millis();
  }
  if(SWRELEASED){
    if(lastReleaseTime - lastPressTime >= LONG_PRESS_TIME){
      handleLongPress();
    }else if(lastReleaseTime - lastPressTime >= SHORT_PRESS_TIME && lastReleaseTime - lastPressTime < LONG_PRESS_TIME){
      handleShortPress();
    }
    SWRELEASED = 0;
  }
}

void handleShortPress() { //Handle short press
  // Put code here to handle a short press of the button
  Serial.println("Short press detected!");
}

void handleLongPress() { //Handle long press
  // Put code here to handle a long press of the button
  Serial.println("Long press detected!");
}


void checkRTC() { //Determine if hour has passed
  // Read current time from RTC
  currentTime = rtc.getTime();
  // Calculate the time difference in seconds
  unsigned long timeDifference = currentTime.unixtime() - previousTime.unixtime();
  // Check if 3600 seconds or more have elapsed since the last check
  if (timeDifference >= 3600) {
    HOURPASSED = 1;
    previousTime = currentTime;
  }
}

void turnOffLCD() {
  lcd.noBacklight();
  LCD_ON = 0;
}


void displayHomeInformation() 
{
  lcd.clear();
  // Line 0
  lcd.setCursor(1, 0);
  lcd.write(ventState ? 5 : 6); // Display lightsOn or lightsOff depending on ventState
  lcd.print(String(custom.lightCycleOn).padStart(2, '0')); // Pad number with 0 if it's less than 10
  lcd.setCursor(5, 0);
  lcd.write(ventState); // Display vent1 or vent2 depending on ventState
  lcd.setCursor(7, 0);
  lcd.write(2); // Display heat icon
  lcd.print(String(custom.temperatureMin).padStart(2, '0')); // Pad number with 0 if it's less than 10
  lcd.print("C");
  lcd.setCursor(11, 0);
  lcd.write(3); // Display humid icon
  lcd.print(String(custom.humidityMin).padStart(2, '0')); // Pad number with 0 if it's less than 10
  lcd.print("%");
  // Line 1
  lcd.setCursor(1, 1);
  lcd.write(7); // Display watLevel icon
  lcd.print(String(custom.soilMoistureMin).padStart(2, '0')); // Pad number with 0 if it's less than 10
  lcd.print("%");
  lcd.setCursor(5, 1);
  lcd.write(4); // Display wifi icon
  lcd.print(":");
  lcd.print(WifiState); // Assuming WifiState is a global variable
}

// Menu actions
void displayWifiInfo() {
  // Display Wi-Fi information implementation
}

void displayWifiSettings() {
  // Display Wi-Fi settings implementation
}

void checkAppConnection() {
  // Check app connection implementation
}

void modifyStartingHour() {
  // Modify starting hour implementation
}

void modifyLightsOnTime() {
  // Modify lights on time implementation
}

void modifyLightsOffTime() {
  // Modify lights off time implementation
}

void modifyWateringInterval() {
  // Modify watering interval implementation
}

void modifyWateringTime() {
  // Modify watering time implementation
}

void modifyWateringAmount() {
  // Modify watering amount implementation
}

void refillWater() {
  // Refill water implementation
}

void displayCurrentStage() {
  // Display current stage implementation
}

void changeStage() {
  // Change stage implementation
}

void displayVersion() {
  // Display version implementation
}
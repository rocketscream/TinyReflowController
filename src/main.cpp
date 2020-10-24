/*******************************************************************************
  Title: Tiny Reflow Controller
  Version: 2.50
  Date: 18-10-2020
  Company: Rocket Scream Electronics
  Author: Lim Phang Moh
  Website: www.rocketscream.com

  Brief
  =====
  This is an example firmware for our Arduino compatible Tiny Reflow Controller.
  A big portion of the code is copied over from our Reflow Oven Controller
  Shield. We added both lead-free and leaded reflow profile support in this
  firmware which can be selected by pressing switch #2 (labelled as LF|PB on PCB)
  during system idle. The unit will remember the last selected reflow profile.
  You'll need to use the MAX31856 library for Arduino.

  This firmware owed very much on the works of other talented individuals as
  follows:
  ==========================================
  Brett Beauregard (www.brettbeauregard.com)
  ==========================================
  Author of Arduino PID library. On top of providing industry standard PID
  implementation, he gave a lot of help in making this reflow oven controller
  possible using his awesome library.

  ==========================================
  Limor Fried of Adafruit (www.adafruit.com)
  ==========================================
  Author of Arduino MAX31856 and SSD1306 libraries. Adafruit has been the source 
  of tonnes of tutorials, examples, and libraries for everyone to learn.

  ==========================================
  Spence Konde (www.drazzy.com/e/)
  ==========================================
  Maintainer of the ATtiny core for Arduino:
  https://github.com/SpenceKonde/ATTinyCore

  Disclaimer
  ==========
  Dealing with high voltage is a very dangerous act! Please make sure you know
  what you are dealing with and have proper knowledge before hand. Your use of
  any information or materials on this Tiny Reflow Controller is entirely at
  your own risk, for which we shall not be liable.

  Licences
  ========
  This Tiny Reflow Controller hardware and firmware are released under the
  Creative Commons Share Alike v3.0 license
  http://creativecommons.org/licenses/by-sa/3.0/
  You are free to take this piece of code, use it and modify it.
  All we ask is attribution including the supporting libraries used in this
  firmware.

  Required Libraries
  ==================
  - Arduino PID Library:
    >> https://github.com/br3ttb/Arduino-PID-Library
  - Adafruit MAX31856 Library:
    >> https://github.com/adafruit/Adafruit_MAX31856
  - Adafruit SSD1306 Library:
    >> https://github.com/adafruit/Adafruit_SSD1306
  - Adafruit GFX Library:
    >> https://github.com/adafruit/Adafruit-GFX-Library

  Revision  Description
  ========  ===========
  2.50      Support for more than 2 profiles
  2.00      Support V2 of the Tiny Reflow Controller:
            - Based on ATMega328P 3.3V @ 8MHz
            - Uses SSD1306 128x64 OLED
  1.00      Initial public release:
            - Based on ATtiny1634R 3.3V @ 8MHz
            - Uses 8x2 alphanumeric LCD

*******************************************************************************/

// ***** INCLUDES *****
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31856.h> 
#include <PID_v1.h>

#if VERSION == 1
  #include <LiquidCrystal.h>
#endif

#if VERSION == 2
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif

// ***** CONSTANTS *****

// ***** GENERAL PROFILE CONSTANTS *****
#define TEMPERATURE_ROOM 30
#define SENSOR_SAMPLING_TIME 1000

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100

// ***** DISPLAY SPECIFIC CONSTANTS *****
#define UPDATE_RATE 100

// ***** PID PARAMETERS *****
#define PID_SAMPLE_TIME 1000

// Profiles
const char profilename_0[] PROGMEM = "Ke. EP256";
const char profilename_1[] PROGMEM = "Lead-free";
const char profilename_2[] PROGMEM = "PLA Annea";
const char profilename_3[] PROGMEM = "PETG Anne";

const char *const profileNames[] PROGMEM = {profilename_0, profilename_1, profilename_2, profilename_3};

// pre-heat
const float profilePreHeatTemps[] PROGMEM = {150, 150, 63, 177};
const uint32_t profilePreHeatMTime[] PROGMEM = {0, 0, 120000, 120000};
const int profilePreHeatTStep[] PROGMEM = {120, 120, 1, 1};
const uint32_t profilePreHeatHoldTime[] PROGMEM = {0, 0, 0, 0};
const float profilePreHeatKP[] PROGMEM = {100, 100, 100, 100};
const float profilePreHeatKI[] PROGMEM = {0.025, 0.025, 0.025, 0.025};
const float profilePreHeatKD[] PROGMEM = {20, 20, 20, 20};

// Soak
const float profileSoakTemps[] PROGMEM = {180, 200, 63, 177};
const uint32_t profileSoakMTime[] PROGMEM = {6000, 6000, 0, 0};
const int profileSoakTStep[] PROGMEM = {5, 5, 5, 0};
const uint32_t profileSoakHoldTime[] PROGMEM = {0, 0, 3600000, 3600000};
const float profileSoakKP[] PROGMEM = {300, 300, 300, 300};
const float profileSoakKI[] PROGMEM = {0.05, 0.05, 0.05, 0.05};
const float profileSoakKD[] PROGMEM = {250, 250, 250, 250};

// Reflow
const float profileReflowTemps[] PROGMEM = {180, 180, 63, 177};
const uint32_t profileReflowMTime[] PROGMEM = {6000, 6000, 0, 0};
const int profileReflowTStep[] PROGMEM = {5, 5, 0, 0};
const uint32_t profileReflowHoldTime[] PROGMEM = {6000, 6000, 0, 0};
const float profileReflowKP[] PROGMEM = {300, 300, 300, 300};
const float profileReflowKI[] PROGMEM = {0.05, 0.05, 0.05, 0.05};
const float profileReflowKD[] PROGMEM = {350, 350, 350, 350};

// Cool-down
const float profileCoolDownTemps[] PROGMEM = {TEMPERATURE_ROOM, TEMPERATURE_ROOM, TEMPERATURE_ROOM, TEMPERATURE_ROOM};
const uint32_t profileCoolDownMTime[] PROGMEM = {6000, 6000, 420000, 420000}; 
const int profileCoolDownTStep[] PROGMEM = {-5, -5, -1, -1};
const uint32_t profileCoolDownHoldTime[] PROGMEM = {0, 0, 0, 0};
const float profileCoolDownKP[] PROGMEM = {300, 300, 300, 300};
const float profileCoolDownKI[] PROGMEM = {0.05, 0.05, 0.05, 0.05};
const float profileCoolDownKD[] PROGMEM = {350, 350, 350, 350};

// ***** TYPE DEFINITIONS *****
char dispbuffer[10]; // profile name and reflow state read buffer max size

typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COOL_COMPLETE,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;

typedef	enum SWITCH
{
  SWITCH_NONE,
  SWITCH_1,
  SWITCH_2
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;



#if VERSION == 2
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  #define X_AXIS_START 18 // X-axis starting position
#endif

// ***** LCD MESSAGES *****
const char ReflowStatus_0[] PROGMEM = "Ready ";
const char ReflowStatus_1[] PROGMEM = "Pre   ";
const char ReflowStatus_2[] PROGMEM = "Soak  ";
const char ReflowStatus_3[] PROGMEM = "Reflow";
const char ReflowStatus_4[] PROGMEM = "Cool  ";
const char ReflowStatus_5[] PROGMEM = "Cool 2";
const char ReflowStatus_6[] PROGMEM = "Done! ";
const char ReflowStatus_7[] PROGMEM = "Hot!  ";
const char ReflowStatus_8[] PROGMEM = "Error ";

// store in program mem to convserve SRAM
const char *const lcdMessagesReflowStatus[] PROGMEM = {ReflowStatus_0, ReflowStatus_1, ReflowStatus_2, ReflowStatus_3, ReflowStatus_4, ReflowStatus_5, ReflowStatus_6, ReflowStatus_7, ReflowStatus_8};

// ***** DEGREE SYMBOL FOR LCD *****
unsigned char degree[8]  = {
  140, 146, 146, 140, 128, 128, 128, 128
};

// ***** PIN ASSIGNMENT *****
#if VERSION == 1
  unsigned char ssrPin = 3;
  unsigned char thermocoupleCSPin = 2;
  unsigned char lcdRsPin = 10;
  unsigned char lcdEPin = 9;
  unsigned char lcdD4Pin = 8;
  unsigned char lcdD5Pin = 7;
  unsigned char lcdD6Pin = 6;
  unsigned char lcdD7Pin = 5;
  unsigned char buzzerPin = 14;
  unsigned char switchPin = A1;
  unsigned char ledPin = LED_BUILTIN;
#elif VERSION == 2
  unsigned char ssrPin = A0;
  unsigned char fanPin = A1;
  unsigned char thermocoupleCSPin = 10;
  unsigned char ledPin = 4;
  unsigned char buzzerPin = 5;
  unsigned char switchStartStopPin = 3;
  unsigned char switchLfPbPin = 2;
#endif

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
unsigned int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long updateLcd;
unsigned int nextStepTime;
unsigned int heldPeriod;
unsigned long buzzerPeriod;
unsigned char soakTemperatureMax;
unsigned char reflowTemperatureMax;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Reflow profile type
unsigned int reflowProfile;
// Switch debounce state machine state variable
debounceState_t debounceState;
// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
switch_t switchValue;
switch_t switchMask;
// Seconds timer
unsigned int timerSeconds;
// Thermocouple fault status
unsigned char fault;
#if VERSION == 2
  unsigned int timerUpdate;
  unsigned char temperature[SCREEN_WIDTH - X_AXIS_START];
  unsigned char x;
#endif

// PID control interface
PID reflowOvenPID(&input, &output, &setpoint, 100, 0.025, 20, DIRECT);
#if VERSION == 1
  // LCD interface
  LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);
#elif VERSION == 2
  Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
#endif
// MAX31856 thermocouple interface
Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(thermocoupleCSPin);


switch_t readSwitch(void)
{
#if VERSION == 1
  unsigned int switchAdcValue = 0;
  // Analog multiplexing switch
  switchAdcValue = analogRead(switchPin);

  // Add some allowance (+10 ADC step) as ADC reading might be off a little
  // due to 3V3 deviation and also resistor value tolerance
  if (switchAdcValue >= 1000) return SWITCH_NONE;
  if (switchAdcValue <= 10) return SWITCH_1;
  if (switchAdcValue <= 522) return SWITCH_2;

#elif VERSION == 2
  // Switch connected directly to individual separate pins
  if (digitalRead(switchStartStopPin) == LOW) return SWITCH_1;
  if (digitalRead(switchLfPbPin) == LOW) return SWITCH_2;

#endif

  return SWITCH_NONE;
}


void setup()
{
  // Check current selected reflow profile
  reflowProfile = 0;
  heldPeriod = 0;
  nextStepTime = 0;

  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  // Buzzer pin initialization to ensure annoying buzzer is off
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);

  // LED pins initialization and turn on upon start-up (active high)
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Initialize thermocouple interface
  thermocouple.begin();
  thermocouple.setThermocoupleType(MAX31856_TCTYPE_K);

  // Start-up splash
  digitalWrite(buzzerPin, HIGH);
#if VERSION == 1
  lcd.begin(8, 2);
  lcd.createChar(0, degree);
  lcd.clear();
  lcd.print(F(" Tiny  "));
  lcd.setCursor(0, 1);
  lcd.print(F(" Reflow "));
#elif VERSION == 2
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay(); // don't show adafruit splash
  oled.display();
#endif
  digitalWrite(buzzerPin, LOW);
  delay(2000);
#if VERSION == 1
  lcd.clear();
  lcd.print(F(" v2.50  "));
  lcd.setCursor(0, 1);
  lcd.print(F("18-10-20"));
  delay(2000);
  lcd.clear();
#elif VERSION == 2
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.println(F("     Tiny Reflow"));
  oled.println(F("     Controller"));
  oled.println();
  oled.println(F("       v2.50"));
  oled.println();
  oled.println(F("     18-10-2020"));
  oled.display();
  delay(2000);
  oled.clearDisplay();
#endif

  // Serial communication at 115200 bps
  Serial.begin(115200);

  // Turn off LED (active high)
  digitalWrite(ledPin, LOW);
  // Set window size
  windowSize = 2000;
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
  // Initialize LCD update timer
  updateLcd = millis();
}

// check stage state and process to next
void processStage(
  boolean heat,
  float tempSetpoint, // set temp for this reflow state
  uint32_t stepPeriod, // microtime until next temp change
  uint32_t holdPeriod, // total microtime to set max temp
  int stepSize, // temp step size per micrtime period
  float tempNextSetpoint, // temp setpoint of next reflow stage
  reflowState_t nextReflowState,  // next reflow state
  float nextPidKp, // next reflow state PID KP
  float nextPidKi, // next reflow state PID KI
  float nextPidKd // next reflow state PID KD
 )
{

      // Serial.print(F("heat: "));
      // Serial.println(heat);
      // Serial.print(F("tempSetpoint: "));
      // Serial.println(tempSetpoint);
      // Serial.print(F("Current Temp: "));
      // Serial.println(input);
      // Serial.print(F("Actial Set Temp: "));
      // Serial.println(setpoint);
      // Serial.print(F("stepPeriod: "));
      // Serial.println(stepPeriod);
      // Serial.print(F("holdPeriod: "));
      // Serial.println(holdPeriod);
      // Serial.print(F("heldPeriod: "));
      // Serial.println(heldPeriod);      
      // Serial.print(F("stepSize: "));
      // Serial.println(stepSize);
      // Serial.print(F("tempNextSetpoint: "));
      // Serial.println(tempNextSetpoint);
      // Serial.print(F("nextReflowState: "));
      // Serial.println(nextReflowState);
      // Serial.println(F("--------------------------------"));


  // have we reached the setpoint temp?
  if ((input >= setpoint && heat == true) || (input <= setpoint && heat == false)) {

      // have we reached the next time point to adjust the temperatue?
      if (millis() > nextStepTime)
      {
        // set the next time to adjust the temperature
        nextStepTime = millis() + stepPeriod;
        // check if temp has reached its final set point
        if ((input >= tempSetpoint && heat == true) || (input <= tempSetpoint && heat == false))
        {
          // increment held period
          heldPeriod += stepPeriod;

          // if we reached the hold time move to next reflowstate
          if (heldPeriod >= holdPeriod) {
            // have we reached total hold period
            heldPeriod = 0;

            nextStepTime = millis() + stepPeriod;
            // Set less agressive PID parameters for next ramp
            reflowOvenPID.SetTunings(nextPidKp, nextPidKi, nextPidKd);
            // Ramp up to next temperature
            setpoint = tempNextSetpoint;
            // Proceed to next state
            reflowState = nextReflowState;
          }
        } else {
          // Increment micro setpoint
          setpoint += stepSize;
        }
      }
  }
}


void loop()
{
  // Current time
  unsigned long now;

  // Time to read thermocouple?
  if (millis() > nextRead)
  {
    // Read thermocouple next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
    input = thermocouple.readThermocoupleTemperature();
    // Check for thermocouple fault
    fault = thermocouple.readFault();

    // If any thermocouple fault is detected
    if ((fault & MAX31856_FAULT_CJRANGE) ||
        (fault & MAX31856_FAULT_TCRANGE) ||
        (fault & MAX31856_FAULT_CJHIGH) ||
        (fault & MAX31856_FAULT_CJLOW) ||
        (fault & MAX31856_FAULT_TCHIGH) ||
        (fault & MAX31856_FAULT_TCLOW) ||
        (fault & MAX31856_FAULT_OVUV) ||
        (fault & MAX31856_FAULT_OPEN))
    {
      // Illegal operation
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
      Serial.println(F("Error"));
    }
  }

  if (millis() > nextCheck)
  {
    // Check input in the next seconds
    nextCheck += SENSOR_SAMPLING_TIME;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Toggle red LED as system heart beat
      digitalWrite(ledPin, !(digitalRead(ledPin)));
      // Increase seconds timer for reflow curve plot
      timerSeconds++;
      // Send temperature and time stamp to serial
      Serial.print(timerSeconds);
      Serial.print(F(","));
      Serial.print(setpoint);
      Serial.print(F(","));
      Serial.print(input);
      Serial.print(F(","));
      Serial.print(output);
      Serial.print(F(","));
      Serial.print(heldPeriod);
      Serial.print(F(","));
      Serial.println(nextStepTime);
    }
    else
    {
      // Turn off red LED
      digitalWrite(ledPin, LOW);
    }
  }

  if (millis() > updateLcd)
  {
    // Update LCD in the next 100 ms
    updateLcd += UPDATE_RATE;
#if VERSION == 1
    // Clear LCD
    lcd.clear();
    // Print current system state
    lcd.print(lcdMessagesReflowStatus[reflowState]);
    lcd.setCursor(6, 0);

    strcpy_P(dispbuffer, (char *)pgm_read_word(&(lcdMessagesReflowStatus[reflowState])));
    lcd.print(dispbuffer);
    lcd.setCursor(0, 1);
    
    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      // Thermocouple error (open, shorted)
      lcd.print(F("TC Error"));
    }
    else
    {
      // Display current temperature
      lcd.print(input);
#if ARDUINO >= 100
      // Display degree Celsius symbol
      lcd.write((uint8_t)0);
#else
      // Display degree Celsius symbol
      lcd.print(0, BYTE);
#endif
      lcd.print("C ");
    }
#elif VERSION == 2
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setCursor(0, 0);

    strcpy_P(dispbuffer, (char *)pgm_read_word(&(lcdMessagesReflowStatus[reflowState])));
    oled.print(dispbuffer);

    oled.setTextSize(1);
    oled.setCursor(72, 0);

    strcpy_P(dispbuffer, (char *)pgm_read_word(&(profileNames[reflowProfile])));
    oled.print(dispbuffer);
    
    // Temperature markers
    oled.setCursor(0, 18);
    oled.print(F("250"));
    oled.setCursor(0, 36);
    oled.print(F("150"));
    oled.setCursor(0, 54);
    oled.print(F("50"));
    // Draw temperature and time axis
    oled.drawLine(18, 18, 18, 63, WHITE);
    oled.drawLine(18, 63, 127, 63, WHITE);
    oled.setCursor(115, 0);

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR)
    {
      oled.setCursor(80, 9);
      oled.print(F("TC Error"));
    }
    else
    {
      // Right align temperature reading
      if (input < 10) oled.setCursor(91, 9);
      else if (input < 100) oled.setCursor(85,9);
      else oled.setCursor(80, 9);
      // Display current temperature
      oled.print(input);
      oled.print((char)247);
      oled.print(F("C"));
    }
    
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // We are updating the display faster than sensor reading
      if (timerSeconds > timerUpdate)
      {
        // Store temperature reading every 3 s
        if ((timerSeconds % 3) == 0)
        {
          timerUpdate = timerSeconds;
          unsigned char averageReading = map(input, 0, 250, 63, 19);
          if (x < (SCREEN_WIDTH - X_AXIS_START))
          {
            temperature[x++] = averageReading;
          }
        }
      }
    }
    
    unsigned char timeAxis;
    for (timeAxis = 0; timeAxis < x; timeAxis++)
    {
      oled.drawPixel(timeAxis + X_AXIS_START, temperature[timeAxis], WHITE);
    }
    
    // Update screen
    oled.display();
#endif
  }

  // Reflow oven controller state machine
  switch (reflowState)
  {
    case REFLOW_STATE_IDLE:
      // If oven temperature is still above room temperature
      if (input >= TEMPERATURE_ROOM)
      {
        reflowState = REFLOW_STATE_TOO_HOT;
      }
      else
      {
        // If switch is pressed to start reflow process
        if (switchStatus == SWITCH_1)
        {
          // Send header for CSV file
          Serial.println(F("Time,Setpoint,Input,Output,HeldP,NextStep"));
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
          
          #if VERSION == 2
          // Initialize reflow plot update timer
          timerUpdate = 0;
          
          for (x = 0; x < (SCREEN_WIDTH - X_AXIS_START); x++)
          {
            temperature[x] = 0;
          }
          // Initialize index for average temperature array used for reflow plot
          x = 0;
          #endif
          
          // Initialize PID control window starting time
          windowStartTime = millis();
         
          setpoint = TEMPERATURE_ROOM;

          reflowOvenPID.SetTunings(pgm_read_byte_near(profilePreHeatKP + reflowProfile), pgm_read_byte_near(profilePreHeatKI + reflowProfile), pgm_read_byte_near(profilePreHeatKD + reflowProfile));

          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
        }
      }
      break;

    case REFLOW_STATE_PREHEAT:

      reflowStatus = REFLOW_STATUS_ON;

      processStage(
        true,
        pgm_read_float_near(profilePreHeatTemps + reflowProfile),
        pgm_read_dword_near(profilePreHeatMTime + reflowProfile),
        pgm_read_dword_near(profilePreHeatHoldTime + reflowProfile),
        pgm_read_byte_near(profilePreHeatTStep + reflowProfile),
        pgm_read_float_near(profileSoakTemps + reflowProfile),
        REFLOW_STATE_SOAK,
        pgm_read_float_near(profileSoakKP + reflowProfile),
        pgm_read_float_near(profileSoakKI + reflowProfile),
        pgm_read_float_near(profileSoakKD + reflowProfile)
      );

      break;

    case REFLOW_STATE_SOAK:

      processStage(
        true,
        pgm_read_float_near(profileSoakTemps + reflowProfile),
        pgm_read_dword_near(profileSoakMTime + reflowProfile),
        pgm_read_dword_near(profileSoakHoldTime + reflowProfile),
        pgm_read_byte_near(profileSoakTStep + reflowProfile),
        pgm_read_float_near(profileReflowTemps + reflowProfile),
        REFLOW_STATE_REFLOW,
        pgm_read_float_near(profileReflowKP + reflowProfile),
        pgm_read_float_near(profileReflowKI + reflowProfile),
        pgm_read_float_near(profileReflowKD + reflowProfile)
      );

      break;

    case REFLOW_STATE_REFLOW:
      // We need to avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components

      processStage(
        true,
        pgm_read_float_near(profileReflowTemps + reflowProfile),
        pgm_read_dword_near(profileReflowMTime + reflowProfile),
        pgm_read_dword_near(profileReflowHoldTime + reflowProfile),
        pgm_read_byte_near(profileReflowTStep + reflowProfile),
        pgm_read_float_near(profileCoolDownTemps + reflowProfile),
        REFLOW_STATE_COOL,
        pgm_read_float_near(profileReflowKP + reflowProfile),
        pgm_read_float_near(profileReflowKI + reflowProfile),
        pgm_read_float_near(profileReflowKD + reflowProfile)
      );

      break;

    case REFLOW_STATE_COOL:

      processStage(
        false,
        pgm_read_float_near(profileCoolDownTemps + reflowProfile),
        pgm_read_dword_near(profileCoolDownMTime + reflowProfile),
        pgm_read_dword_near(profileCoolDownHoldTime + reflowProfile),
        pgm_read_byte_near(profileCoolDownTStep + reflowProfile),
        (double) TEMPERATURE_ROOM,
        REFLOW_STATE_COOL_COMPLETE,
        pgm_read_float_near(profileReflowKP + reflowProfile),
        pgm_read_float_near(profileReflowKI + reflowProfile),
        pgm_read_float_near(profileReflowKD + reflowProfile)
      );

      break;

    case REFLOW_STATE_COOL_COMPLETE:
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;
      break;

    case REFLOW_STATE_COMPLETE:
      if (millis() > buzzerPeriod)
      {
        // Turn off buzzer
        digitalWrite(buzzerPin, LOW);
        // Reflow process ended
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_TOO_HOT:
      // If oven temperature drops below room temperature
      if (input < TEMPERATURE_ROOM)
      {
        // Ready to reflow
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_ERROR:
      // Check for thermocouple fault
      fault = thermocouple.readFault();

      // If thermocouple problem is still present
      if ((fault & MAX31856_FAULT_CJRANGE) ||
          (fault & MAX31856_FAULT_TCRANGE) ||
          (fault & MAX31856_FAULT_CJHIGH) ||
          (fault & MAX31856_FAULT_CJLOW) ||
          (fault & MAX31856_FAULT_TCHIGH) ||
          (fault & MAX31856_FAULT_TCLOW) ||
          (fault & MAX31856_FAULT_OVUV) ||
          (fault & MAX31856_FAULT_OPEN))
      {
        // Wait until thermocouple wire is connected
        reflowState = REFLOW_STATE_ERROR;
      }
      else
      {
        // Clear to perform reflow process
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
  }

  // If switch 1 is pressed
  if (switchStatus == SWITCH_1)
  {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  }
  // Switch 2 is pressed
  else if (switchStatus == SWITCH_2)
  {
    // Only can switch reflow profile during idle
    if (reflowState == REFLOW_STATE_IDLE)
    {
      if (reflowProfile >= (sizeof(profileNames) / sizeof(profileNames[0])) - 1) {
        reflowProfile = 0;
      } else {
        reflowProfile++;
      }      
    }
  }
  // Switch status has been read
  switchStatus = SWITCH_NONE;

  // Simple switch debounce state machine (analog switch)
  switch (debounceState)
  {
    case DEBOUNCE_STATE_IDLE:
      // No valid switch press
      switchStatus = SWITCH_NONE;

      switchValue = readSwitch();

      // If either switch is pressed
      if (switchValue != SWITCH_NONE)
      {
        // Keep track of the pressed switch
        switchMask = switchValue;
        // Intialize debounce counter
        lastDebounceTime = millis();
        // Proceed to check validity of button press
        debounceState = DEBOUNCE_STATE_CHECK;
      }
      break;

    case DEBOUNCE_STATE_CHECK:
      switchValue = readSwitch();
      if (switchValue == switchMask)
      {
        // If minimum debounce period is completed
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN)
        {
          // Valid switch press
          switchStatus = switchMask;
          // Proceed to wait for button release
          debounceState = DEBOUNCE_STATE_RELEASE;
        }
      }
      // False trigger
      else
      {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;

    case DEBOUNCE_STATE_RELEASE:
      switchValue = readSwitch();
      if (switchValue == SWITCH_NONE)
      {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON)
  {
    now = millis();

    reflowOvenPID.Compute();

    if ((now - windowStartTime) > windowSize)
    {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if (output > (now - windowStartTime)) digitalWrite(ssrPin, HIGH);
    else digitalWrite(ssrPin, LOW);
  }
  // Reflow oven process is off, ensure oven is off
  else
  {
    digitalWrite(ssrPin, LOW);
  }
}

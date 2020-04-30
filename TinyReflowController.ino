/*******************************************************************************
  Title: Tiny Reflow Controller
  Version: 2.12u
  Date: 04-22-2020
  Company: Rocket Scream Electronics
  Author: Lim Phang Moh
  Website: www.rocketscream.com
    Unofficial mods: Pat Daderko

  Brief
  =====
  This is an example firmware for our Arduino compatible Tiny Reflow Controller.
  A big portion of the code is copied over from our Reflow Oven Controller
  Shield. We added both lead-free and leaded reflow profile support in this
  firmware which can be selected by pressing switch #2 (labelled as LF|PB on PCB)
  during system idle. The unit will remember the last selected reflow profile.
  You'll need to use the MAX31856 library for Arduino.

  Unofficial Changes
  ==================
  Below describes most of the substantial changes in this unofficial firmware
  version.  All were only tested on a V2 controller, and many only apply to a V2
  controller.  Code changes were kept to a minimum, and new code structure,
  style, naming, etc. attempted to mimic the original code.
  
  The fan can be toggled by pressing switch #2 when the oven is in any reflow
  state other than idle or error.  The animated fan icon in the upper right
  corner notifies you that the fan is enabled.  The fan automatically turns off
  when it returns to the idle state.

  Manual temperature mode can be toggled by pressing both switch #1 and #2
  simultaneously ("Manual" displayed on screen, with the selected function and
  setpoint shown to the right).  This can be entered from any state (other than
  error), and the temperature set point will set to the current temperature
  rounded down to the nearest 5 degrees C (or clipped to the min or max of 50C or
  250C).  Press switch #1 to change function and switch #2 to change value.  The
  functions are setpoint+25 (++), setpoint+5 (+), setpoint-5 (-), setpoint-25
  (--), and fan (F). The controller will return to its previous state when manual
  mode is exited.  This allows it to be used standalone for baking parts, or
  manually adjusting the reflow profile on the fly.

  The temperature plot now shows hash marks on the Y axis at 50C, 150C, and 250C,
  as well as every minute on the X axis.  The plot also continuously scrolls as
  time goes beyond the screen width.

  A soft power off and on function is available by pressing and holding switch #1.
  The unit can be turned off by holding the switch for 3 seconds from any state.
  The unit can be turned back on by holding the switch for 2 seconds.  Turning
  the unit off shuts everything down, resets the state, and sits in a loop waiting
  for the power on button press, though it is still safest and recommended to
  unplug the oven when done.  When the unit is powered back on, the software is
  restarted to ensure a fresh state.
  
  Lead-Free Reflow Curve
  ======================

  Temperature (Degree Celcius)                 Magic Happens Here!
  245-|                                               x  x
      |                                            x        x
      |                                         x              x
      |                                      x                    x
  200-|                                   x                          x
      |                              x    |                          |   x
      |                         x         |                          |       x
      |                    x              |                          |
  150-|               x                   |                          |
      |             x |                   |                          |
      |           x   |                   |                          |
      |         x     |                   |                          |
      |       x       |                   |                          |
      |     x         |                   |                          |
      |   x           |                   |                          |
  30 -| x             |                   |                          |
      |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
      | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)

  Leaded Reflow Curve (Kester EP256)
  ==================================

  Temperature (Degree Celcius)         Magic Happens Here!
  219-|                                       x  x
      |                                    x        x
      |                                 x              x
  180-|                              x                    x
      |                         x    |                    |   x
      |                    x         |                    |       x
  150-|               x              |                    |           x
      |             x |              |                    |
      |           x   |              |                    |
      |         x     |              |                    |
      |       x       |              |                    |
      |     x         |              |                    |
      |   x           |              |                    |
  30 -| x             |              |                    |
      |<  60 - 90 s  >|<  60 - 90 s >|<   60 - 90 s      >|
      | Preheat Stage | Soaking Stage|   Reflow Stage     | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _
                                                                 Time (Seconds)

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
  2.12u     Add soft power on/off
  2.11u     Add manual temperature control, plus tweaks to graphing and other minor functions
  2.10u     Add fan output function on V2
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
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Adafruit_GFX.h>      // Comment for VERSION 1
#include <Adafruit_SSD1306.h>  // Comment for VERSION 1 
#include <Adafruit_MAX31856.h> 
#include <PID_v1.h>

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_PREHEAT,
  REFLOW_STATE_SOAK,
  REFLOW_STATE_REFLOW,
  REFLOW_STATE_COOL,
  REFLOW_STATE_COMPLETE,
  REFLOW_STATE_TOO_HOT,
  REFLOW_STATE_MANUAL,
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
  SWITCH_2,
  SWITCH_BOTH
} switch_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_RELEASE
} debounceState_t;

typedef enum REFLOW_PROFILE
{
  REFLOW_PROFILE_LEADFREE,
  REFLOW_PROFILE_LEADED
} reflowProfile_t;

typedef enum MANUAL_ADJUST
{
  MANUAL_ADJUST_TEMP_FAST_UP,
  MANUAL_ADJUST_TEMP_UP,
  MANUAL_ADJUST_TEMP_DOWN,
  MANUAL_ADJUST_TEMP_FAST_DOWN,
  MANUAL_ADJUST_FAN
} manualAdjust_t;

// ***** CONSTANTS *****
// ***** GENERAL *****
#define VERSION 2 // Replace with 1 or 2

// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5

// ***** LEAD FREE PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_LF 200
#define TEMPERATURE_REFLOW_MAX_LF 250
#define SOAK_MICRO_PERIOD_LF 9000

// ***** LEADED PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_PB 180
#define TEMPERATURE_REFLOW_MAX_PB 224
#define SOAK_MICRO_PERIOD_PB 10000

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 50

// ***** DISPLAY SPECIFIC CONSTANTS *****
#define UPDATE_RATE 100

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

#if VERSION == 2
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define X_AXIS_START 18 // X-axis starting position
#endif

#define POWER_OFF_BTN_TIME 3000
#define POWER_ON_BTN_TIME 2000

// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
  "Ready",
  "Pre",
  "Soak",
  "Reflow",
  "Cool",
  "Done!",
  "Hot!",
  "Manual",
  "Error"
};

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
double manPrevSetpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long updateLcd;
unsigned long timerSoak;
unsigned long buzzerPeriod;
unsigned char soakTemperatureMax;
unsigned char reflowTemperatureMax;
unsigned long soakMicroPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState, manPrevState;
// Reflow oven controller status
reflowStatus_t reflowStatus, manPrevStatus;
// Reflow profile type
reflowProfile_t reflowProfile;
// Manual adjust setting
manualAdjust_t manualAdjust;
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
unsigned char xHead=0;
unsigned char xCnt=0;
unsigned char xScrollOffset=0;
#endif

// PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
#if VERSION == 1
// LCD interface
LiquidCrystal lcd(lcdRsPin, lcdEPin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);
#elif VERSION == 2
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
#endif
// MAX31856 thermocouple interface
Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(thermocoupleCSPin);

void(* resetFunc) (void) = 0; //not a true reset, but close enough

void setup()
{
  // Check current selected reflow profile
  unsigned char value = EEPROM.read(PROFILE_TYPE_ADDRESS);
  if ((value == 0) || (value == 1))
  {
    // Valid reflow profile value
    reflowProfile = value;
  }
  else
  {
    // Default to lead-free profile
    EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
    reflowProfile = REFLOW_PROFILE_LEADFREE;
  }

  //initialize state, status, and setpoint
  reflowState = REFLOW_STATE_IDLE;
  debounceState = DEBOUNCE_STATE_IDLE;
  reflowStatus = REFLOW_STATUS_OFF;
  setpoint = 0;

  // SSR pin initialization to ensure reflow oven is off
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

#if VERSION == 2
  // Fan pin initialization to ensure fan is off
  digitalWrite(fanPin, LOW);
  pinMode(fanPin, OUTPUT);
#endif

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
  oled.display();
#endif
  digitalWrite(buzzerPin, LOW);
  delay(1000);
#if VERSION == 1
  lcd.clear();
  lcd.print(F(" v1.00  "));
  lcd.setCursor(0, 1);
  lcd.print(F("26-07-17"));
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
  oled.println(F("      v2.12u"));
  oled.println();
  oled.println(F("      04-22-20"));
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
  // Tell the PID to range between 0 and the full window size
  reflowOvenPID.SetOutputLimits(0, windowSize);
  reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
  // Turn the PID on
  reflowOvenPID.SetMode(AUTOMATIC);
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermocouple reading variable
  nextRead = millis();
  // Initialize LCD update timer
  updateLcd = millis();
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
      Serial.println(output);
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
    if (reflowProfile == REFLOW_PROFILE_LEADFREE)
    {
	    lcd.print(F("LF"));
    }
    else
    {
      lcd.print(F("PB"));
    }
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
    oled.print(lcdMessagesReflowStatus[reflowState]);

    oled.setTextSize(1);
    //display fan symbol if fan enabled
    if (digitalRead(fanPin))
    {
      oled.setCursor(121, 0);
      if ((millis()>>10)&1) //animate roughly every second
        oled.print(F("x"));
      else
        oled.print(F("+"));
    }

    if (reflowState!=REFLOW_STATE_MANUAL) //normal mode
    {
      oled.setCursor(103, 0);
      if (reflowProfile == REFLOW_PROFILE_LEADFREE)
      {
        oled.print(F("LF"));
      }
      else
      {
        oled.print(F("PB"));
      }
    }
    else //manual mode
    {
      oled.setCursor(73, 0);
      switch (manualAdjust) //print adjustment mode
      {
        case MANUAL_ADJUST_TEMP_FAST_UP:
          oled.print(F("++"));
          break;
        case MANUAL_ADJUST_TEMP_UP:
          oled.print(F(" +"));
          break;
        case MANUAL_ADJUST_TEMP_DOWN:
          oled.print(F(" -"));
          break;
        case MANUAL_ADJUST_TEMP_FAST_DOWN:
          oled.print(F("--"));
          break;
        case MANUAL_ADJUST_FAN:
          oled.print(F(" F"));
          break;
      }
      //print set point
      if (setpoint < 100) oled.setCursor(97,0);
      else oled.setCursor(91, 0);
      oled.print((int)setpoint);
      oled.print(F("C"));
    }
    
    // Temperature markers
    oled.setCursor(0, 16);
    oled.print(F("250"));
    oled.setCursor(0, 33);
    oled.print(F("150"));
    oled.setCursor(6, 51);
    oled.print(F("50"));
    // Draw temperature and time axis
    oled.drawLine(18, 18, 18, 63, WHITE); //left vertical line
    oled.drawLine(18, 19, 20, 19, WHITE); //250 tick
    oled.drawLine(18, 36, 20, 36, WHITE); //150 tick
    oled.drawLine(18, 54, 20, 54, WHITE); //50 tick
    oled.drawLine(18, 63, 127, 63, WHITE); //bottom horizontal line
    //time markers, scroll with plot
    oled.drawLine(38-xScrollOffset, 63, 38-xScrollOffset, 61, WHITE);
    oled.drawLine(58-xScrollOffset, 63, 58-xScrollOffset, 61, WHITE);
    oled.drawLine(78-xScrollOffset, 63, 78-xScrollOffset, 61, WHITE);
    oled.drawLine(98-xScrollOffset, 63, 98-xScrollOffset, 61, WHITE);
    oled.drawLine(118-xScrollOffset, 63, 118-xScrollOffset, 61, WHITE);
    if (xScrollOffset>10)
      oled.drawLine(138-xScrollOffset, 63, 138-xScrollOffset, 61, WHITE);

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
          if (xCnt < (SCREEN_WIDTH - X_AXIS_START)) //haven't filled entire screen yet
          {
            temperature[xCnt++] = averageReading;
          }
          else //screen full, scroll graph
          {
            temperature[xHead++] = averageReading;
            if (xHead == (SCREEN_WIDTH - X_AXIS_START))
              xHead=0;
            xScrollOffset++;
            if (xScrollOffset>19)
              xScrollOffset=0;
          }
        }
      }
    }
    
    unsigned char timeAxis, tElem;
    tElem=xHead;
    for (timeAxis = 0; timeAxis < xCnt; timeAxis++)
    {
      oled.drawPixel(timeAxis + X_AXIS_START, temperature[tElem++], WHITE);
      if (tElem==(SCREEN_WIDTH - X_AXIS_START))
        tElem=0;
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
          Serial.println(F("Time,Setpoint,Input,Output"));
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;
          
#if VERSION == 2
          // Initialize reflow plot update timer
          timerUpdate = 0;
          // Initialize index for average temperature array used for reflow plot
          xHead = 0;
          xCnt = 0;
          xScrollOffset=0;
#endif
          
          // Initialize PID control window starting time
          windowStartTime = millis();
          // Set initial PID parameters for preheat ramp
          kp=PID_KP_PREHEAT;
          ki=PID_KI_PREHEAT;
          kd=PID_KD_PREHEAT;
          reflowOvenPID.SetTunings(kp, ki, kd);
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN+5;
          // Load profile specific constant
          if (reflowProfile == REFLOW_PROFILE_LEADFREE)
          {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_LF;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_LF;
            soakMicroPeriod = SOAK_MICRO_PERIOD_LF;
          }
          else
          {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_PB;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_PB;
            soakMicroPeriod = SOAK_MICRO_PERIOD_PB;
          }
          // Proceed to preheat stage
          reflowState = REFLOW_STATE_PREHEAT;
        }
#if VERSION == 2
        //if idle, make sure fan is off
        digitalWrite(fanPin, LOW);
#endif
      }
      break;

    case REFLOW_STATE_PREHEAT:
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (input >= TEMPERATURE_SOAK_MIN)
      {
        // Chop soaking period into smaller sub-period
        timerSoak = millis() + soakMicroPeriod;
        // Set less agressive PID parameters for soaking ramp
        kp=PID_KP_SOAK;
        ki=PID_KI_SOAK;
        kd=PID_KD_SOAK;
        reflowOvenPID.SetTunings(kp, ki, kd);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      // If micro soak temperature is achieved
      if (millis() > timerSoak)
      {
        timerSoak = millis() + soakMicroPeriod;
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > soakTemperatureMax)
        {
          // Set agressive PID parameters for reflow ramp
          kp=PID_KP_REFLOW;
          ki=PID_KI_REFLOW;
          kd=PID_KD_REFLOW;
          reflowOvenPID.SetTunings(kp, ki, kd);
          // Ramp up to first section of soaking temperature
          setpoint = reflowTemperatureMax;
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;

    case REFLOW_STATE_REFLOW:
      // We need to avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (input >= (reflowTemperatureMax - 5))
      {
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN-5;
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (input <= TEMPERATURE_COOL_MIN)
      {
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;
      }
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

    case REFLOW_STATE_MANUAL:
      //switch 1 pressed
      if (switchStatus == SWITCH_1)
      {
        //select adjustment
        switch (manualAdjust)
        {
          case MANUAL_ADJUST_TEMP_FAST_UP:
            manualAdjust=MANUAL_ADJUST_TEMP_UP;
            break;
          case MANUAL_ADJUST_TEMP_UP:
            manualAdjust=MANUAL_ADJUST_TEMP_DOWN;
            break;
          case MANUAL_ADJUST_TEMP_DOWN:
            manualAdjust=MANUAL_ADJUST_TEMP_FAST_DOWN;
            break;
          case MANUAL_ADJUST_TEMP_FAST_DOWN:
            manualAdjust=MANUAL_ADJUST_FAN;
            break;
          case MANUAL_ADJUST_FAN:
            manualAdjust=MANUAL_ADJUST_TEMP_FAST_UP;
            break;
        }
      }
      //switch 2 pressed
      else if (switchStatus == SWITCH_2)
      {
        //adjust value
        switch (manualAdjust)
        {
          case MANUAL_ADJUST_TEMP_FAST_UP:
            setpoint+=25; //increase temp
            if (setpoint>TEMPERATURE_REFLOW_MAX_LF)
              setpoint=TEMPERATURE_REFLOW_MAX_LF;
            break;
          case MANUAL_ADJUST_TEMP_UP:
            setpoint+=5; //increase temp
            if (setpoint>TEMPERATURE_REFLOW_MAX_LF)
              setpoint=TEMPERATURE_REFLOW_MAX_LF;
            break;
          case MANUAL_ADJUST_TEMP_DOWN:
            setpoint-=5; //decrease temp
            if (setpoint<TEMPERATURE_ROOM)
              setpoint=TEMPERATURE_ROOM;
            break;
          case MANUAL_ADJUST_TEMP_FAST_DOWN:
            setpoint-=25; //decrease temp
            if (setpoint<TEMPERATURE_ROOM)
              setpoint=TEMPERATURE_ROOM;
            break;
          case MANUAL_ADJUST_FAN:
            digitalWrite(fanPin, !digitalRead(fanPin)); //toggle fan
            break;
        }
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
#if VERSION == 2
      //if error, make sure fan is off
      digitalWrite(fanPin, LOW);
#endif
      break;
  }

  if (reflowState!=REFLOW_STATE_MANUAL) //normal mode
  {
    //both switches pressed
    if (switchStatus == SWITCH_BOTH)
    {
      manPrevState=reflowState; //save current state
      manPrevStatus=reflowStatus; //save current status
      manPrevSetpoint=setpoint; //save current setpoint
      reflowState=REFLOW_STATE_MANUAL; //go into manual mode
      reflowStatus=REFLOW_STATUS_ON; //turn oven control on
      manualAdjust=MANUAL_ADJUST_TEMP_FAST_UP; //set adjustment mode

      //check if it's starting new (REFLOW_STATUS_OFF) or continuing existing run (REFLOW_STATUS_ON)
      if (manPrevStatus == REFLOW_STATUS_OFF)
      {
        // Intialize seconds timer for serial debug information
        timerSeconds = 0;
#if VERSION == 2
        // Initialize reflow plot update timer
        timerUpdate = 0;
        // Initialize index for average temperature array used for reflow plot
        xHead = 0;
        xCnt = 0;
        xScrollOffset=0;
#endif
        // Initialize PID control window starting time
        windowStartTime = millis();
      }
      
      // Set agressive PID parameters for manual mode
      reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
      setpoint = (int)input-(((int)input)%5); //configure setpoint to current temp (rounded down to nearest multiple of 5)
      if (setpoint>TEMPERATURE_REFLOW_MAX_LF)
        setpoint=TEMPERATURE_REFLOW_MAX_LF;
      else if (setpoint<TEMPERATURE_ROOM)
        setpoint=TEMPERATURE_ROOM;
    }
    // If switch 1 is pressed
    else if (switchStatus == SWITCH_1)
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
        // Currently using lead-free reflow profile
        if (reflowProfile == REFLOW_PROFILE_LEADFREE)
        {
          // Switch to leaded reflow profile
          reflowProfile = REFLOW_PROFILE_LEADED;
          EEPROM.write(PROFILE_TYPE_ADDRESS, 1);
        }
        // Currently using leaded reflow profile
        else
        {
          // Switch to lead-free profile
          reflowProfile = REFLOW_PROFILE_LEADFREE;
          EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
        }
      }
#if VERSION == 2
      else if (reflowState != REFLOW_STATE_ERROR) //if button pressed when not idle (and not error), toggle fan
      {
        digitalWrite(fanPin, !digitalRead(fanPin));
      }
#endif
    }
  }
  else //manual mode
  {
    //both switches pressed
    if (switchStatus == SWITCH_BOTH)
    {
      //go back to normal mode (restore previous state)
      reflowState=manPrevState;
      reflowStatus=manPrevStatus;
      //restore previous setpoint
      setpoint=manPrevSetpoint;
      //restore PID values
      reflowOvenPID.SetTunings(kp, ki, kd);
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
      //button held for 3 seconds, soft power button "pressed", turn oven off
      if ((switchMask== SWITCH_1)&&(switchValue == SWITCH_1)&&((millis() - lastDebounceTime) > POWER_OFF_BTN_TIME))
      {
        //make sure all oven stuff is off
        digitalWrite(ssrPin, LOW);
#if VERSION == 2
        digitalWrite(fanPin, LOW);
#endif
        digitalWrite(buzzerPin, LOW);
        digitalWrite(ledPin, LOW);

        //turn off display
        oled.ssd1306_command(SSD1306_DISPLAYOFF);

        //clear state just to be safe (though it should never leave this loop until software restart)
        reflowState=REFLOW_STATE_IDLE;
        reflowStatus = REFLOW_STATUS_OFF;
        setpoint = 0;

        //do nothing until button held again
        while (1)
        {
          if (readSwitch() == SWITCH_1)
          {
            lastDebounceTime = millis();
            while (readSwitch() == SWITCH_1)
            {
              if ((millis() - lastDebounceTime) >= POWER_ON_BTN_TIME)
                resetFunc(); //to be safe, reboot instead of simply going back to main loop
            }
          }
        }
      }
      //button released
      if (switchValue == SWITCH_NONE)
      {
        // Valid switch press (report when button(s) released so button hold can have different functionality)
        switchStatus = switchMask;
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

switch_t readSwitch(void)
{
  int switchAdcValue = 0;
#if VERSION == 1
  // Analog multiplexing switch
  switchAdcValue = analogRead(switchPin);

  // Add some allowance (+10 ADC step) as ADC reading might be off a little
  // due to 3V3 deviation and also resistor value tolerance
  if (switchAdcValue >= 1000) return SWITCH_NONE;
  if (switchAdcValue <= 10) return SWITCH_1;
  if (switchAdcValue <= 522) return SWITCH_2;

#elif VERSION == 2
  // Switch connected directly to individual separate pins
  if ((digitalRead(switchStartStopPin) == LOW)&&(digitalRead(switchLfPbPin) == LOW)) return SWITCH_BOTH;
  if (digitalRead(switchStartStopPin) == LOW) return SWITCH_1;
  if (digitalRead(switchLfPbPin) == LOW) return SWITCH_2;

#endif

  return SWITCH_NONE;
}

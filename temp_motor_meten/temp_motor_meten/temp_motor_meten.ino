#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RunningAverage.h"


#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */

// Declare the states in meaningful English. Enums start enumerating
// at zero, incrementing in steps of 1 unless overridden. We use an
// enum 'class' here for type safety and code readability
enum class measurmentState : uint8_t {
  IDLE,             // defaults to 0, wait for permission, stop loop after 10min consecutive measurments, charge battery
  STARTLOAD,        // defaults to 1, start 1A load
  TAKEMEASUREMENT,  // defaults to 2, take measurement 0.1s (40 samples)? 1s to long?
  STOPLOAD,         // defaults to 3, stop 1A load
  SENDRESULT,       // defaults to 4, calculate temperature and send/show results with mqtt and display them on oled
  COOLDOWN,         // defaults to 5, wait 1s for lm317 cool down, check permission wile waiting (if permission start load, else disconntect motor and idle)
  // stop loop after 10min consecutively measurments
};

// Keep track of the current State (it's a measurmentState variable)
measurmentState currState = measurmentState::IDLE;

/* Be sure to update this value based on the IC and the gain settings! */
const float multiplier = 0.0078125F; /* ADS1115  @ +/- +/- 0.256V (16-bit results) */

bool trottlePermission = false;  // permission from trottle to take a measurment
bool lastTrottlePermission = trottlePermission;
bool motorConnected = false;     // 1A load and ads1115 connected to motor
bool tenMinCoolDown = false;     // stop taking measurments after 10min consecutively measuring
bool chargeBattery = false;      // charge battery when in idle 
bool currentSourceOn = false;    // current for measuring temperture of motor

int16_t measurementRaw = 0;
float voltagemV = 0;

void setup(void) {
  Serial.begin(112500);
  Serial.println(F("Hello!"));

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    //  for (;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  Serial.println(F("Getting differential reading from AIN0 (P) and AIN1 (N)"));
  Serial.println(F("ADC Range: +/- 0.256V (1 bit = 0.0078125mV)"));

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  // https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/   ADS1115
  //                                                                -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V          0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V          0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V          0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V          0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V          0.015625mV
  ads.setGain(GAIN_SIXTEEN);  // 16x gain  +/- 0.256V          0.0078125mV

  // ads.setDataRate(RATE_ADS1115_8SPS);    // (0x0000)    8 samples per second
  // ads.setDataRate(RATE_ADS1115_16SPS);   // (0x0020)   16 samples per second
  // ads.setDataRate(RATE_ADS1115_32SPS);   // (0x0040)   32 samples per second
  // ads.setDataRate(RATE_ADS1115_64SPS);   // (0x0060)   64 samples per second
  // ads.setDataRate(RATE_ADS1115_128SPS);  // (0x0080)  128 samples per second (default)
  // ads.setDataRate(RATE_ADS1115_250SPS);  // (0x00A0)  250 samples per second
  // ads.setDataRate(RATE_ADS1115_475SPS);  // (0x00C0)  475 samples per second
  ads.setDataRate(RATE_ADS1115_860SPS);  // (0x00E0)  860 samples per second  // about 400 samples per second

  if (!ads.begin()) {
    Serial.println(F("Failed to initialize ADS."));
    //  while (1);
  }
}

void loop(void) {
  // Process according to our State Diagram
  switch (currState) {

    // Initial state (or final returned state)
    case measurmentState::IDLE:

    if((!trottlePermission || tenMinCoolDown) && motorConnected) {
      // todo: disconntectMotor()
      motorConnected = false;
    }
    if((!trottlePermission || tenMinCoolDown) && !motorConnected) {
      // todo: sendMotorState() (motorConnected true or false) to trottle with ESPNOW The boat is allowed to seal!
    }
    if((!trottlePermission || tenMinCoolDown) && !chargeBattery) {
      // todo: turnOnBattery()
      chargeBattery = true;
    }

    // reset tenMinCoolDown if trottlePermission changes 
    if(trottlePermission != lastTrottlePermission) {
      tenMinCoolDown = false;
      lastTrottlePermission = trottlePermission;
    }

    // if allowed prepare for measurment
    if(trottlePermission && !tenMinCoolDown) {
      if(chargeBattery) {
        // todo: turnOffBattry()
        chargeBattery = false;
      }
      if(!motorConnected) {
        // todo: connectMotor()
        motorConnected = true;
      }
      // if everything is ready, start the cooldown timer, the load and tell the trottle we are measuring
      if(!chargeBattery && !motorConnected) {
        // todo: sendMotorState() (motorConnected true or false) to trottle with ESPNOW The boat is not allowed to seal!
        currState = measurmentState::STARTLOAD;
      }
    }

      break;

    case measurmentState::STARTLOAD:
    static uint32_t currentSourceOnTime = millis();
    const static int8_t currentSourceOnDelay = 20;    // wait 20ms for current to settle (unsure if 20ms is enough)

    if (!currentSourceOn) {
      // todo: turnOnCurrentSource();
      currentSourceOn = true;
    }
    // wait ...ms for current to settle
    if(currentSourceOn) {
      static uint32_t tenMinCoolDownTime = millis();
      if(millis() - currentSourceOnTime >= currentSourceOnDelay) {
      currState = measurmentState::TAKEMEASUREMENT;
      tenMinCoolDownTime = millis();
      }
    }
 
      break;

    case measurmentState::TAKEMEASUREMENT:

      break;

    case measurmentState::STOPLOAD:

      break;

    case measurmentState::SENDRESULT:

      break;

    case measurmentState::COOLDOWN:

      break;

  }

  //measurementcasmeasurementRaw * multiplier;
  Serial.print("Differential: ");
  Serial.print(measurementRaw);
  Serial.print("(");
  Serial.print(voltagemV);
  Serial.println("mV)");

  display.clearDisplay();
  display.setTextSize(2);  // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  // Start at top-left corner
  display.print(voltagemV, 3);
  display.display();
  delay(2000);
}
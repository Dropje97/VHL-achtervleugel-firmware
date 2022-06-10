#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "RunningAverage.h"
#include <esp_now.h>
#include <WiFi.h>

#define DEBUG

/*
ideën:
1. voorkom dat de boot niet snel reageert wanneer je bij de stijger aan het manuvreren bent.
door bv. door alleen te gaan meten wanneer de trottle recentelijk over de 30% is geweest. of dat de boot langere tijd in de zelde richting vaart
2. de temp meting mag geen invloed hebben op de noodstop.
3. de schoef moet compleet stil staan anders produceert die zelf stroom als een dynamo.
*/

RunningAverage myRA(100);
RunningAverage myRA_min(60);
RunningAverage myRA_s(9);

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
measurmentState currState = measurmentState::IDLE;  // Keep track of the current State (it's a measurmentState variable)

const int8_t READY_PIN = 13;  // Pin connected to the ALERT/RDY signal for new sample notification.
const int8_t LOAD_PIN = 27; // Pin to turn the 1A load on

const float multiplier = 0.0078125F;     /* ADS1115  @ +/- +/- 0.256V (16-bit results). Be sure to update this value based on the IC and the gain settings! */
const float referenceTemperature = 20;   // temperature when the referenceVoltagemV was measured
const float referenceVoltagemV = 1;      // voltage between the motor windings at the referenceTemperature
const float temperatureCoefficient = 2;  //

float motorTemperature = 0;  // current calculated temperture ftom the resistance of the motor windings
float measurmentRawAvg = 0;

bool trottlePermission = true;  // permission from trottle to take a measurment
bool lastTrottlePermission = trottlePermission;
bool motorConnected = false;   // 1A load and ads1115 connected to motor
bool tenMinCoolDown = false;   // stop taking measurments after 10min consecutively measuring
bool chargeBattery = false;    // charge battery when in idle
bool currentSourceOn = false;  // current for measuring temperature =  of motor

uint32_t tenMinCoolDownTime = millis();
int16_t measurementRaw = 0;
float voltagemV = 0;

uint8_t broadcastAddress[] = { 0x9C, 0x9C, 0x1F, 0xDD, 0x4D, 0xD0 };  // MAC Address of receiver esp32 (own MAC Address: AC:67:B2:36:B2:D0)
String success;                                                       // Variable to store if sending data was successful
esp_now_peer_info_t peerInfo;                                         // store info of throttle esp

// This is required on ESP32 to put the ISR in IRAM. Define as
// empty for other platforms. Be careful - other platforms may have
// other requirements.
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&trottlePermission, incomingData, sizeof(trottlePermission));
  //Serial.print("Bytes received: ");
  //Serial.println(len);
}

// Forward declaration of all functions
void displayState(String currState);
void sendMotorState();

volatile bool newMeasurment = false;
void IRAM_ATTR NewDataReadyISR() {
  newMeasurment = true;
}
void setup(void) {
  Serial.begin(112500);
  Serial.println(F("Hello!"));

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    //Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
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
    ads.setDataRate(RATE_ADS1115_860SPS);  // (0x00E0)  860 samples per second  // about 400 samples per second in non continuous mode

  if (!ads.begin()) {
    Serial.println(F("Failed to initialize ADS."));
  }

  pinMode(LOAD_PIN, OUTPUT);
  pinMode(READY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);  // We get a falling edge every time a new sample is ready.

  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, /*continuous=*/true);  // Start continuous conversions.
}

void loop(void) {

  switch (currState) {

    // Initial state (or final returned state)
    case measurmentState::IDLE:
      displayState("IDLE state");  // todo: voeg dit overal toe. mischien ifdef debug?

      if ((!trottlePermission || tenMinCoolDown) && motorConnected) {
        // todo: disconntectMotor()
        motorConnected = false;
      }
      if ((!trottlePermission || tenMinCoolDown) && !motorConnected) {
        sendMotorState();
        // todo: sendMotorState() (motorConnected true or false) to trottle with ESPNOW The boat is allowed to seal!
      }
      if ((!trottlePermission || tenMinCoolDown) && !chargeBattery) {
        // todo: turnOnBattery()
        chargeBattery = true;
      }

      // reset tenMinCoolDown if trottlePermission changes
      if (trottlePermission != lastTrottlePermission) {
        tenMinCoolDown = false;
        lastTrottlePermission = trottlePermission; 
      }

      // if allowed prepare for measurment
      if (trottlePermission && !tenMinCoolDown) {
        if (chargeBattery) {
          // todo: turnOffBattry()
          chargeBattery = false;
        }
        if (!motorConnected) {
          // todo: connectMotor()
          motorConnected = true;
        }
        // if everything is ready, start the cooldown timer, the load and tell the trottle we are measuring
        if (!chargeBattery && motorConnected) {
          // todo: sendMotorState() (motorConnected true or false) to trottle with ESPNOW The boat is not allowed to seal!
          currState = measurmentState::STARTLOAD;
          tenMinCoolDownTime = millis();  // reset 10 min cooldown timer
        }
      }

      break;

    case measurmentState::STARTLOAD:
      static uint32_t currentSourceOnTime = millis();
      static uint32_t lastMeasurementTime = millis();
      const static int8_t currentSourceOnDelay = 20;  // wait 20ms for current to settle (unsure if 20ms is enough (update 2022: it is enough))

      if (!currentSourceOn) {
        digitalWrite(LOAD_PIN, HIGH);
        currentSourceOnTime = millis();
        currentSourceOn = true;
      }
      // wait ...ms for current to settle
      if (currentSourceOn) {
        if (millis() - currentSourceOnTime >= currentSourceOnDelay) {
          currState = measurmentState::TAKEMEASUREMENT;
          lastMeasurementTime = millis();
        }
      }

      break;

    case measurmentState::TAKEMEASUREMENT:
      // todo: na te veel tijd geen meting hebben gehad stop met meten zodat de boot weer kan varen en stuur een error. misschien een negative temperatuur als error?

      const static int16_t  measurementTime = 100;
      static int16_t i = 0;
      if (newMeasurment && (millis() - lastMeasurementTime < measurementTime)) {  // wait until there is an new measurement and stop after ... measurments
        newMeasurment = false;             // reset newMeasurement
        measurementRaw = ads.getLastConversionResults();
        myRA.addValue(measurementRaw);
#ifdef DEBUG
        Serial.print(i);
        Serial.print("\t");
        Serial.println(measurementRaw);
#endif
        // todo: add measurementRaw to measurmentRawAvg?? float problem? solution first calculate temperature = ? I don't know
        i++;  // add 1 to the measurement counter
      }
      if (millis() - lastMeasurementTime >= measurementTime) {
        i = 0;
        currState = measurmentState::STOPLOAD;
#ifdef DEBUG
        Serial.println(i);
#endif
      }

      break;

    case measurmentState::STOPLOAD:

      if (currentSourceOn) {
        digitalWrite(LOAD_PIN, LOW);
        currentSourceOn = false;
      }
      if (!currentSourceOn) {
        currState = measurmentState::SENDRESULT;
      }

      break;

    case measurmentState::SENDRESULT:
      measurmentRawAvg = myRA.getAverage();
      static float measurmentRawStandardDeviation = 0;
      static float minAvgStandardDeviation = 0;
      static float minAvg2 = 0;
      static float secAvg = 0;
      
      measurmentRawStandardDeviation = myRA.getStandardDeviation();
      myRA_min.addValue(measurmentRawAvg);
      minAvgStandardDeviation = myRA_min.getStandardDeviation();
      minAvg2 = myRA_min.getAverage();
      secAvg = myRA_s.getAverage();

      motorTemperature = (measurmentRawAvg - referenceVoltagemV) / referenceVoltagemV * temperatureCoefficient + referenceTemperature;
      //Serial.println(motorTemperature);
      // todo: send mqtt motorTemperature and measurmentRawAvg
      // todo: send espnow motorTemperature to fancy display
      //

      display.clearDisplay();
      display.setTextSize(1);  // Draw 2X-scale text
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);  // Start at top-left corner
      display.print(measurmentRawAvg*multiplier, 3);
      display.setCursor(0, 12);  // Start at top-left corner
      display.print(measurmentRawStandardDeviation*multiplier, 3);
      display.setCursor(0, 24);
      display.print(minAvg2*multiplier, 3); display.print(" "); display.print(minAvgStandardDeviation*multiplier, 3);
      display.display();

      Serial.println(measurmentRawAvg*multiplier, 3);

      currState = measurmentState::COOLDOWN;
      myRA.clear();

      break;

    case measurmentState::COOLDOWN:
    // todo: add permission check and 10mincooldown
    if(millis() - currentSourceOnTime > 1000) {
      currState = measurmentState::STARTLOAD;
    }
      break;
  }

  //measurementcasmeasurementRaw * multiplier;

  //Serial.print(measurementRaw);
}

void sendMotorState() {
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&motorConnected, sizeof(motorConnected));

#ifdef DEBUG
  /* if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  } */
#endif
}

// Helper routine to track state machine progress
void displayState(String currState) {
  static String prevState = "";

  if (currState != prevState) {
    Serial.println(currState);
    prevState = currState;
  }
}

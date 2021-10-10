#include "DualVNH5019MotorShield.h"
#include "can.h"
#include "mcp2515.h"

#define HOME_DEBUG

MCP2515 mcp2515(PB12);  //compleet willekeurige pin want ER WAS NOG GEEN PIN
DualVNH5019MotorShield md(2, 4, 9, 6, A0, 7, 8, 10, 6, A1);

//const uint8_t pot_pin = A2;
const uint8_t pinA_links = PB1;  // Rotary encoder links Pin A
const uint8_t pinB_links = PB2;  // Rotary encoder links Pin B
const uint8_t pinA_rechts = PB3;  // Rotary encoder 2 rechts Pin A
const uint8_t pinB_rechts = PB4;  // Rotary encoder 2 rechts Pin B

const int8_t kp = 10;  // pid instellingen
const float ki = 0.1;
const int8_t kd = 0;

const int16_t max_pulsen = 17500;             // 156 pulsen per rotatie * 12 max rotaties vanaf home = 1872 pulsen in totaal (m4 is 0,7mm per rotatie dus 8,4mm totaal).
const int16_t start_PWM = 150;                // de motor begint direct te draaien op de startwaarde of langzamer als er minder gas nodig is, daana neemt de smoothing het over.
const uint8_t smoothing_time = 5;           // tijd in millis tussen het verhogen van het PWM van de motor zodat deze rustig versneld. hogere waarde is langzamer versnellen.
const uint8_t amps_poll_interval = 1;        // tijd tussen de metingen van het stroomverbuik.
const uint8_t serial_print_interval = 100;    // tijd tussen de serial prints.
const uint8_t direction_change_delay = 200;  // tijd die de motor om de rem staat wanneer die van richting verandert.
const uint8_t PID_interval = 10;             // iedere 10ms wordt de PID berekend. het veranderen van deze waarde heeft invloed op de I en D hou daar rekening mee.
const uint8_t CAN_send_interval = 10;        // de CAN berichten worden 100x per seconden verzonden.
const uint16_t CAN_read_interval = 50;     // de CAN berichten worden 1000x per seconden ontvangen.

const uint16_t CAN_ID = 51;               // CAN ID van setpoint_PWM
const uint16_t CAN_ID_amps_achter = 250;  // CAN ID van CAN_ID_amps_achter
const uint16_t CAN_ID_home_achter = 300;  // CAN ID van home_achter

volatile int encoder_pulsen_links = 0;
volatile int encoder_pulsen_rechts = 0;

//uint16_t pot_val = 0;
volatile bool ENC_links_A;
volatile bool ENC_links_B;
volatile bool ENC_rechts_A;
volatile bool ENC_rechts_B;

uint32_t last_smoothing_links;
uint32_t last_smoothing_links_time = 0;
uint32_t last_direction_change_links = 0;
uint32_t last_PID_links = 0;

uint32_t last_smoothing_rechts;
uint32_t last_smoothing_rechts_time = 0;
uint32_t last_direction_change_rechts = 0;
uint32_t last_PID_rechts = 0;

uint32_t last_amps_poll = 0;
uint32_t last_serial_print = 0;
uint32_t last_CAN_send = 0;
uint32_t last_CAN_read = 0;
uint32_t timer = millis();  // wordt gelijk gesteld aan millis zodat millis niet elke keer opgevraagd wordt want dat kost veel cpu tijd

int32_t smoothing_PWM_links = 0;  //
int32_t setpoint_PWM_links = 0;
int32_t setpoint_PID_PWM_links = 0;
int32_t setpoint_home_PWM_links = 0;
int32_t PWM_links = 0;
int32_t last_PWM_links = 0;
int32_t amps_links = 0;

int32_t smoothing_PWM_rechts = 0;  //
int32_t setpoint_PWM_rechts = 0;
int32_t setpoint_PID_PWM_rechts = 0;
int32_t setpoint_home_PWM_rechts = 0;
int32_t PWM_rechts = 0;
int32_t last_PWM_rechts = 0;
int32_t amps_rechts = 0;

uint16_t overcurrent_limit = 0;  // waarde word berekend in loop en is afhankelijk van de PWM

int32_t setpoint_pulsen_links = 0;
int32_t error_links = 0;
int32_t previus_error_links = 0;
int32_t diff_error_links = 0;
int32_t P_links = 0;
float I_links = 0;
float D_links = 0;
int32_t PID_links = 0;

int32_t setpoint_pulsen_rechts = 0;
int32_t error_rechts = 0;
int32_t previus_error_rechts = 0;
int32_t diff_error_rechts = 0;
int32_t P_rechts = 0;
float I_rechts = 0;
float D_rechts = 0;
int32_t PID_rechts = 0;

int16_t i16;

bool direction_change_links = false;
bool direction_links = 1;  // 0= negatief 1=positief
bool previus_direction_links = direction_links;

bool direction_change_rechts = false;
bool direction_rechts = 1;  // 0= negatief 1=positief
bool previus_direction_rechts = direction_rechts;

bool overcurrent = false;
bool homeing = false;
bool has_homed = false;
uint8_t CAN_error = 0;  //1= motor disconnect

struct can_frame ret;
struct can_frame canMsg;
int16_t CAN_setpoint_pulsen = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  //  pinMode(pot_pin, INPUT);
  pinMode(pinA_links, INPUT_PULLUP);  // Set Pin_A as input
  pinMode(pinB_links, INPUT_PULLUP);  // Set Pin_B as input
  pinMode(pinA_rechts, INPUT_PULLUP);  // Set Pin_A as input
  pinMode(pinB_rechts, INPUT_PULLUP);  // Set Pin_B as input

  // Atach a CHANGE interrupt to PinB and exectute the update function when this change occurs.
  attachInterrupt(digitalPinToInterrupt(pinA_links), encoderA_links_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_links), encoderB_links_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinA_rechts), encoderA_rechts_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB_rechts), encoderB_rechts_ISR, CHANGE);
  /*
    // md.setSpeeds(400, 400);
    md.setM1Speed(400);
    md.setM2Speed(400);
    delay(5000);
    md.setSpeeds(0, 0);
    delay(1000);
  */
}

void encoderB_links_ISR() {
  ENC_links_A = digitalRead(pinA_links);
  ENC_links_B = digitalRead(pinB_links);

  if (ENC_links_A && ENC_links_B) {
    encoder_pulsen_links--;  //decrement the encoder's position count
  } else if (!ENC_links_A && !ENC_links_B) {
    encoder_pulsen_links--;
  } else if (ENC_links_A && !ENC_links_B) {
    encoder_pulsen_links++;
  } else if (!ENC_links_A && ENC_links_B) {
    encoder_pulsen_links++;
  }
}

void encoderA_links_ISR() {
  ENC_links_A = digitalRead(pinA_links);
  ENC_links_B = digitalRead(pinB_links);

  if (ENC_links_A && ENC_links_B) {
    encoder_pulsen_links++;  //increment the encoder's position count
  } else if (!ENC_links_A && !ENC_links_B) {
    encoder_pulsen_links++;
  } else if (ENC_links_A && !ENC_links_B) {
    encoder_pulsen_links--;
  } else if (!ENC_links_A && ENC_links_B) {
    encoder_pulsen_links--;
  }
}
void encoderB_rechts_ISR() {
  ENC_rechts_A = digitalRead(pinA_rechts);
  ENC_rechts_B = digitalRead(pinB_rechts);

  if (ENC_rechts_A && ENC_rechts_B) {
    encoder_pulsen_rechts--;  //decrement the encoder's position count
  } else if (!ENC_rechts_A && !ENC_rechts_B) {
    encoder_pulsen_rechts--;
  } else if (ENC_rechts_A && !ENC_rechts_B) {
    encoder_pulsen_rechts++;
  } else if (!ENC_rechts_A && ENC_rechts_B) {
    encoder_pulsen_rechts++;
  }
}

void encoderA_rechts_ISR() {
  ENC_rechts_A = digitalRead(pinA_rechts);
  ENC_rechts_B = digitalRead(pinB_rechts);

  if (ENC_rechts_A && ENC_rechts_B) {
    encoder_pulsen_rechts++;  //increment the encoder's position count
  } else if (!ENC_rechts_A && !ENC_rechts_B) {
    encoder_pulsen_rechts++;
  } else if (ENC_rechts_A && !ENC_rechts_B) {
    encoder_pulsen_rechts--;
  } else if (!ENC_rechts_A && ENC_rechts_B) {
    encoder_pulsen_rechts--;
  }
}

void loop() {
  timer = millis();
  //======================= lees potmeter ==================================
  /*
    pot_val = 0.05 * analogRead(pot_pin) + 0.95 * pot_val;
    // setpoint_PWM_links = map(pot_val, 0, 1023, -400, 400);
    setpoint_pulsen_links = map(pot_val, 0, 1023, -400, 400);
  */

  //====================== smoothing acceleratie + debug ======================================

  if (homeing) {
    home();
    setpoint_PWM_links = setpoint_home_PWM_links;
    setpoint_PWM_rechts = setpoint_home_PWM_rechts;
  } else if (has_homed) {
    setpoint_PWM_links = setpoint_PID_PWM_links;
    setpoint_PWM_rechts = setpoint_PID_PWM_rechts;
  }

  // =============================== links low-pass filter ==================================================

  int16_t setpoint_PWM_links_last_PWM = abs(setpoint_PWM_links) - abs(last_PWM_links);

  smoothing_PWM_links = sqrt(abs(setpoint_PWM_links_last_PWM));

  if ((setpoint_PWM_links > start_PWM) && (setpoint_PWM_links - last_PWM_links >= smoothing_PWM_links)) {
    if (last_PWM_links < start_PWM) {
      PWM_links = start_PWM;
      last_PWM_links = start_PWM - smoothing_PWM_links;
    }
    if (timer - last_smoothing_links_time >= smoothing_time) {
      PWM_links = last_PWM_links + smoothing_PWM_links;
      last_PWM_links = PWM_links;
      last_smoothing_links_time = timer;
    }

  } else if (setpoint_PWM_links > 0) {
    PWM_links = setpoint_PWM_links;

    if ((PWM_links > 200) && (amps_links < 200) && (PWM_links - last_PWM_links > 0)) {  // voor debug
      CAN_error = 1;
    } else {
      CAN_error = 0;
    }
    last_PWM_links = PWM_links;
  }

  if ((setpoint_PWM_links < -start_PWM) && (setpoint_PWM_links - last_PWM_links <= smoothing_PWM_links)) {
    if (last_PWM_links > -start_PWM) {
      PWM_links = -start_PWM;
      last_PWM_links = -start_PWM + smoothing_PWM_links;
    }
    if (timer - last_smoothing_links_time >= smoothing_time) {
      PWM_links = last_PWM_links - smoothing_PWM_links;
      last_PWM_links = PWM_links;
      last_smoothing_links_time = timer;
    }

  } else if (setpoint_PWM_links < 0) {
    PWM_links = setpoint_PWM_links;
    if ((PWM_links < -200) && (amps_links < 200) && (PWM_links - last_PWM_links < 0)) {  // voor debug
      CAN_error = 1;
    } else {
      CAN_error = 0;
    }
    last_PWM_links = PWM_links;
  }

  // ======================================== rechts low-pass filter ================================

  int16_t setpoint_PWM_rechts_last_PWM = abs(setpoint_PWM_rechts) - abs(last_PWM_rechts);

  smoothing_PWM_rechts = sqrt(abs(setpoint_PWM_rechts_last_PWM));

  if ((setpoint_PWM_rechts > start_PWM) && (setpoint_PWM_rechts - last_PWM_rechts >= smoothing_PWM_rechts)) {
    if (last_PWM_rechts < start_PWM) {
      PWM_rechts = start_PWM;
      last_PWM_rechts = start_PWM - smoothing_PWM_rechts;
    }
    if (timer - last_smoothing_rechts_time >= smoothing_time) {
      PWM_rechts = last_PWM_rechts + smoothing_PWM_rechts;
      last_PWM_rechts = PWM_rechts;
      last_smoothing_rechts_time = timer;
    }

  } else if (setpoint_PWM_rechts > 0) {
    PWM_rechts = setpoint_PWM_rechts;

    if ((PWM_rechts > 200) && (amps_rechts < 200) && (PWM_rechts - last_PWM_rechts > 0)) {  // voor debug
      CAN_error = 1;
    } else {
      CAN_error = 0;
    }
    last_PWM_rechts = PWM_rechts;
  }

  if ((setpoint_PWM_rechts < -start_PWM) && (setpoint_PWM_rechts - last_PWM_rechts <= smoothing_PWM_rechts)) {
    if (last_PWM_rechts > -start_PWM) {
      PWM_rechts = -start_PWM;
      last_PWM_rechts = -start_PWM + smoothing_PWM_rechts;
    }
    if (timer - last_smoothing_rechts_time >= smoothing_time) {
      PWM_rechts = last_PWM_rechts - smoothing_PWM_rechts;
      last_PWM_rechts = PWM_rechts;
      last_smoothing_rechts_time = timer;
    }

  } else if (setpoint_PWM_rechts < 0) {
    PWM_rechts = setpoint_PWM_rechts;
    if ((PWM_rechts < -200) && (amps_rechts < 200) && (PWM_rechts - last_PWM_rechts < 0)) {  // voor debug
      CAN_error = 1;
    } else {
      CAN_error = 0;
    }
    last_PWM_rechts = PWM_rechts;
  }

  //===================== overcurrent detectie =============================

  if (timer - last_amps_poll >= amps_poll_interval) {
    last_amps_poll = timer;
    amps_links = amps_links * 0.997 + md.getM1CurrentMilliamps() * 0.003;
    amps_rechts = amps_rechts * 0.997 + md.getM2CurrentMilliamps() * 0.003;

    amps_links = constrain(amps_links, 0, 30000);
    amps_rechts = constrain(amps_rechts, 0, 30000);

    overcurrent_limit = 3500;

    if ((amps_links > overcurrent_limit) || (amps_rechts > overcurrent_limit)) {
      overcurrent = true;
      md.setBrakes(400, 400);
    }
  }

  //================= change direction detectie ============================

  if (PWM_links > 0) {
    direction_links = 1;  // PWM_links is groter dan 0 dus positief
  } else {
    direction_links = 0;  // PWM_links is kleiner dan 0 dus negatef
  }
  if (PWM_links > 0) {
    direction_rechts = 1;  // PWM_links is groter dan 0 dus positief
  } else {
    direction_rechts = 0;  // PWM_links is kleiner dan 0 dus negatef
  }

  if (direction_links != previus_direction_links) {
    I_links = 0;
    direction_change_links = true;
    previus_direction_links = direction_links;
  } else if (timer - last_direction_change_links >= direction_change_delay) {
    last_direction_change_links = timer;
    direction_change_links = false;
  }
  if (direction_rechts != previus_direction_rechts) {
    I_rechts = 0;
    direction_change_rechts = true;
    previus_direction_rechts = direction_rechts;
  } else if (timer - last_direction_change_rechts >= direction_change_delay) {
    last_direction_change_rechts = timer;
    direction_change_rechts = false;
  }

  //=============================== PWM naar motor ========================

  if ((((PWM_links < 55) && (PWM_links > -55)) || (direction_change_links == true) || (abs(error_links) <= 2)) && !homeing) {
    md.setM1Brake(400);
    overcurrent = false;
  } else if (overcurrent == false) {
    md.setM1Speed(PWM_links);
  }
  if ((((PWM_rechts < 55) && (PWM_rechts > -55)) || (direction_change_rechts == true) || (abs(error_rechts) <= 2)) && !homeing) {
    md.setM2Brake(400);
    overcurrent = false;
  } else if (overcurrent == false) {
    md.setM2Speed(PWM_rechts);
  }

  //==================================== PID compute ===============================

  if (timer - last_PID_links >= PID_interval) {
    last_PID_links = timer;

    setpoint_pulsen_links = constrain(CAN_setpoint_pulsen, 0, max_pulsen);

    error_links = setpoint_pulsen_links - encoder_pulsen_links;
    //diff_error_links = 0.2 * (error_links - previus_error_links) + 0.8 * diff_error_links;
    //previus_error_links = error_links;

    P_links = kp * error_links;
    if (((abs(P_links) < 400) || (abs(PID_links) < 400)) && (direction_change_links == false)) {  // update de I alleen wanneer de motor nog niet op vol vermogen draait en niet op de rem staat omdat ie van richting verandert.
      I_links = ki * float(error_links) + I_links;
      if (abs(error_links) <= 2) {
        I_links = 0;
      }
    }
    //D_links = kd * diff_error_links;

    I_links = constrain(I_links, -200.0, 200.0);

    PID_links = P_links + long(I_links) + long(D_links);
    PID_links = constrain(PID_links, -400, 400);
    setpoint_PID_PWM_links = PID_links;

    // =========================== rechts

    setpoint_pulsen_rechts = constrain(CAN_setpoint_pulsen, 0, max_pulsen);

    error_rechts = setpoint_pulsen_rechts - encoder_pulsen_rechts;
    //diff_error_rechts = 0.2 * (error_rechts - previus_error_rechts) + 0.8 * diff_error_rechts;
    //previus_error_rechts = error_rechts;

    P_rechts = kp * error_rechts;
    if (((abs(P_rechts) < 400) || (abs(PID_rechts) < 400)) && (direction_change_rechts == false)) {  // update de I alleen wanneer de motor nog niet op vol vermogen draait en niet op de rem staat omdat ie van richting verandert.
      I_rechts = ki * float(error_rechts) + I_rechts;
      if (abs(error_rechts) <= 2) {
        I_rechts = 0;
      }
    }
    //D_rechts = kd * diff_error_rechts;

    I_rechts = constrain(I_rechts, -200.0, 200.0);

    PID_rechts = P_rechts + long(I_rechts) + long(D_rechts);
    PID_rechts = constrain(PID_rechts, -400, 400);
    setpoint_PID_PWM_rechts = PID_rechts;
  }

  //============================================================SerialPrints============================================
  if (timer - last_serial_print >= serial_print_interval) {
    last_serial_print = timer;

    //Serial.println(amps_rechts);
    /*
        Serial.print(overcurrent_limit);
        Serial.print(" - ");
        Serial.print(amps_links);
        Serial.print(" - ");
        Serial.print(CAN_setpoint_pulsen);
        Serial.print(" - ");
        Serial.print(encoder_pulsen_links);
        Serial.print(" - ");
        Serial.println(PWM_links);
    */


    Serial.print(overcurrent_limit);
    Serial.print(" - ");
    Serial.print(amps_rechts);
    Serial.print(" - ");
    Serial.print(CAN_setpoint_pulsen);
    Serial.print(" - ");
    Serial.print(encoder_pulsen_rechts);
    Serial.print(" - ");
    Serial.println(PWM_rechts);


    /*
        P_links= constrain(P, -400, 400);
        //D = constrain(D, -400, 400);
        Serial.print(encoder_pulsen_links);
        Serial.print(" - ");
        Serial.print(PID);
        Serial.print(" - ");
        Serial.print(P);
        Serial.print(" - ");
        Serial.println(I);
        //  Serial.print(" - ");
        //  Serial.println(D, 0);
    */
    /*
      //Serial.print(last_PWM_links);
      //Serial.print(" - ");

      Serial.print(error_links);
      Serial.print(" - ");
      Serial.print(encoder_pulsen_links);
      Serial.print(" - ");
      Serial.print(setpoint_PWM_links);
      Serial.print(" - ");
      Serial.print(PWM_links);
      Serial.print(" - ");
      Serial.print(amps_links);
      Serial.print(" - ");
      Serial.print(md.getM2CurrentMilliamps());
      Serial.print(" - ");
      Serial.println(overcurrent_limit);
    */
  }

  //============================================== send/read can data ===========================================================================

  //=========================== send_CAN_setpoint_PWM
  //send_CAN_setpoint_PWM();

  //========================= send current
  //  send_CAN_current();

  //========================= read CAN
  if (timer - last_CAN_read >= CAN_read_interval) {
    last_CAN_read = timer;

    read_CAN_data();
    read_CAN_data();
    read_CAN_data();
  }
}

void home() {
  const static uint16_t min_home_time = 5000;
  static uint32_t last_home_time = timer;

  if (setpoint_home_PWM_links == 0) {  // als het homen nog niet begonnen is
#ifdef HOME_DEBUG
    Serial.println("Start homing");
#endif
    overcurrent = false;
    setpoint_home_PWM_links = -400;                            // begin met homen
    setpoint_home_PWM_rechts = -400;                           // begin met homen
    last_home_time = timer;                              // reset last_home_time zodat de 100ms van mnin_home_time nu in gaat
  } else if (timer - last_home_time >= min_home_time) {  // wacht 5000ms zodat amps niet meer 0 is door het filter.
    if ((amps_links < 300) && (amps_rechts < 300)) {     // als het stroom verbruik onder de 100mA is dan is de overcurrent getriggered en is de vleugel op zijn minimale stand.
#ifdef HOME_DEBUG
      Serial.print F(("mili amps links: "));
      Serial.println(amps_links);
      Serial.print F(("mili amps rechts: "));
      Serial.println(amps_rechts);
#endif
      delay(500);               // wacht 500ms zodat de motor stil staat.
      /*  if ((setpoint_home_PWM_links == -400) && (setpoint_home_PWM_rechts == -400)) {
        #ifdef HOME_DEBUG
          Serial.println F(("snel homen klaar"));
        #endif
          md.setSpeeds(0, 0);
          delay(500);
          md.setSpeeds(400, 400);
          delay(1000);
          md.setSpeeds(0, 0);
          delay(200);
          //  amps_links = 0;
          //  amps_rechts = 0;
          setpoint_home_PWM_links = -200;                            // begin met homen
          setpoint_home_PWM_rechts = -200;                           // begin met homen
          last_home_time = timer;
        } else { */
      encoder_pulsen_links = -50;       // reset de pulsen.
      setpoint_pulsen_links = 0;      // reset het setpoint.
      setpoint_home_PWM_links = 0;    // stop met gas geven. de volgdende keer dat de void home() gedaan wordt zal de 100ms timer weer worden gereset.
      I_links = 0;                    // zet de I van de PID op 0 zodat de motor niet spontaan begint te draaien.
      setpoint_PID_PWM_links = 0;     // zet de PID_PWM op 0 zodat de motor niet spontaan begint te draaien.
      amps_links = 0;                 // zet het stroomsterkte filter weer op 0.

      encoder_pulsen_rechts = -10;       // reset de pulsen.
      setpoint_pulsen_rechts = 0;      // reset het setpoint.
      setpoint_home_PWM_rechts = 0;    // stop met gas geven. de volgdende keer dat de void home() gedaan wordt zal de 100ms timer weer worden gereset.
      I_rechts = 0;                    // zet de I van de PID op 0 zodat de motor niet spontaan begint te draaien.
      setpoint_PID_PWM_rechts = 0;     // zet de PID_PWM op 0 zodat de motor niet spontaan begint te draaien.

      amps_rechts = 0;                 // zet het stroomsterkte filter weer op 0.
      CAN_setpoint_pulsen = 0;  // zet CAN_setpoin_pulsen op 0 zodat de vleugel niet direct terug gaat naar de vorige positie maar op het CAN bericht wacht.
      overcurrent = false;      // overcurrent is false na het homen zodat de motor weer kan draaien.
      homeing = false;          // homen is klaar.
      has_homed = true;
#ifdef HOME_DEBUG
      Serial.println("homed");
#endif

    }
  }
}

void send_CAN_setpoint_PWM() {
  byte bytes[sizeof(int16_t)];                     //make an array and reserve the size of the datatype we want to send
  memcpy(bytes, &setpoint_PWM_links, sizeof(int16_t));   // copy the content of i16 to the array bytes until we hit the size of the int16 datatype
  for (uint8_t i = 0; i < sizeof(int16_t); i++) {  //basic counter
    ret.data[i] = bytes[i];                        //copy the data from bytes to their respective location in ret.bytes
  }
  ret.can_id = CAN_ID;            //set the can id of "ret" to our can id
  ret.can_dlc = sizeof(int16_t);  //set the dlc to the size of our data type (int16)
  //  return ret; //return the frame
  mcp2515.sendMessage(&ret);  //we send the setpoint_PWM as set by the PID to can ID 51
}

void send_CAN_current() {
  byte bytes[sizeof(uint16_t)];                     //make an array and reserve the size of the datatype we want to send
  memcpy(bytes, &amps_links, sizeof(uint16_t));           // copy the content of i16 to the array bytes until we hit the size of the int16 datatype
  for (uint8_t i = 0; i < sizeof(uint16_t); i++) {  //basic counter
    ret.data[i] = bytes[i];                         //copy the data from bytes to their respective location in ret.bytes
  }
  ret.can_id = CAN_ID;             //set the can id of "ret" to our can id
  ret.can_dlc = sizeof(uint16_t);  //set the dlc to the size of our data type (int16)
  //  return ret; //return the frame
  mcp2515.sendMessage(&ret);  //we send the setpoint_PWM as set by the PID to can ID 51
}

void read_CAN_data() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("CAN frame id: ");
    Serial.println(canMsg.can_id);
    if (canMsg.can_id == 0xC8) {                                             //is can msg ID is 200 in hex
      Serial.print("CAN frame setpulsen: ");
      CAN_setpoint_pulsen = int16_from_can(canMsg.data[4], canMsg.data[5]);  //byte 4-5 is int16_t pulsen achter
      Serial.println(CAN_setpoint_pulsen);
    }
    if (canMsg.can_id == 0x12c) {  //300
      homeing = canMsg.data[0];    // byte 0 is bool homen achter
      Serial.print("CAN frame homing: ");
      Serial.println(homeing);
    }

  }
}
/*
  can_frame int_to_frame(int16_t i16, uint16_t can_id) {
  byte bytes[sizeof(int16_t)]; //make an array and reserve the size of the datatype we want to send
  memcpy(bytes, &i16, sizeof(int16_t)); // copy the content of i16 to the array bytes until we hit the size of the int16 datatype
  can_frame ret; //make a frame called "ret"
  for (uint8_t i = 0; i < sizeof(int16_t); i++) { //basic counter
    ret.data[i] = bytes[i]; //copy the data from bytes to their respective location in ret.bytes
  }
  ret.can_id = can_id; //set the can id of "ret" to our can id
  ret.can_dlc = sizeof(int16_t); //set the dlc to the size of our data type (int16)
  return ret; //return the frame
  }
*/

int16_t int16_from_can(uint8_t b1, uint8_t b2) {
  // maakt van twee bytes een int16_t
  int16_t ret;
  ret = b1 | (int16_t)b2 << 8;
  return ret;
}

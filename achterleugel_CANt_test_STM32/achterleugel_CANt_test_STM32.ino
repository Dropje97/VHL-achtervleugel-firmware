#include "DualVNH5019MotorShield.h"
#include "can.h"
#include "mcp2515.h"

MCP2515 mcp2515(10);  //compleet willekeurige pin want ER WAS NOG GEEN PIN
DualVNH5019MotorShield md(7, 8, 9, 6, A1, 7, 8, 10, 6, A1);

//const uint8_t pot_pin = A2;
const uint8_t pinA = 2;  // Rotary encoder Pin A
const uint8_t pinB = 3;  // Rotary encoder Pin B

const int8_t kp = -50;
const int8_t ki = -1;
const int8_t kd = -0;

const int16_t start_PWM = 80;                // de motor begint direct te draaien op de startwaarde of langzamer als er minder gas nodig is, daana neemt de smoothing het over.
const uint8_t smoothing_time = 20;           // tijd in millis tussen het verhogen van het PWM van de motor zodat deze rustig versneld. hogere waarde is langzamer versnellen.
const uint8_t amps_poll_interval = 1;        // tijd tussen de metingen van het stroomverbuik.
const uint8_t serial_print_interval = 50;    // tijd tussen de serial prints.
const uint8_t direction_change_delay = 200;  // tijd die de motor om de rem staat wanneer die van richting verandert.
const uint8_t PID_interval = 10;             // iedere 10ms wordt de PID berekend. het veranderen van deze waarde heeft invloed op de I en D hou daar rekening mee.
const uint8_t CAN_send_interval = 10;        // de CAN berichten worden 100x per seconden verzonden.
const uint16_t CAN_ID = 51;                  // CAN ID van setpoint_PWM
int16_t max_pulsen = 1872;                   // 156 pulsen per rotatie * 12 max rotaties vanaf home = 1872 pulsen in totaal (m4 is 0,7mm per rotatie dus 8,4mm totaal).

volatile int encoder_pulsen = 0;
volatile int encoder_pulsen_prev = encoder_pulsen;

uint16_t pot_val = 0;
volatile bool ENC_A;
volatile bool ENC_B;

int16_t smoothing_PWM = 0;  //
int16_t setpoint_PWM = 0;
int16_t setpoint_PID_PWM = 0;
int16_t setpoint_home_PWM = 0;
int16_t PWM = 0;
int16_t last_PWM = 0;
uint32_t last_smoothing;
uint32_t last_smoothing_time = 0;  //
uint32_t last_amps_poll = 0;
uint32_t last_serial_print = 0;
uint32_t last_direction_change = 0;
uint32_t last_PID = 0;
uint32_t last_CAN_SEND;
uint32_t timer = 0;  // wordt gelijk gesteld aan millis zodat millis niet elke keer opgevraagd wordt want dat kost veel cpu tijd
uint16_t amps = 0;
uint16_t overcurrent_limit = 0;  // waarde word berekend in loop en is afhankelijk van de PWM
int32_t setpoint_pulsen = 0;
int32_t error = 0;
int32_t previus_error = 0;
int32_t diff_error = 0;
int32_t P = 0;
float I = 0;
float D = 0;
int32_t PID = 0;

int16_t i16;

bool overcurrent = false;
bool direction_change = false;
bool direction = 0;  // 0= negatief 1=positief
bool previus_direction = direction;
bool homeing = false;
uint8_t CAN_error = 0;  //1= motor disconnect

struct can_frame ret;
struct can_frame canMsg;
int16_t CAN_setpoint_pulsen;

void setup() {
  Serial.begin(2000000);
  Serial.println("Dual VNH5019 Motor Shield");
  md.init();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  //  pinMode(pot_pin, INPUT);
  pinMode(pinA, INPUT_PULLUP);  // Set Pin_A as input
  pinMode(pinB, INPUT_PULLUP);  // Set Pin_B as input

  // Atach a CHANGE interrupt to PinB and exectute the update function when this change occurs.
  attachInterrupt(digitalPinToInterrupt(pinA), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), encoderB_ISR, CHANGE);
}
void encoderA_ISR() {
  ENC_A = digitalRead(pinA);
  ENC_B = digitalRead(pinB);

  if (ENC_A && ENC_B) {
    encoder_pulsen--;  //decrement the encoder's position count
  } else if (!ENC_A && !ENC_B) {
    encoder_pulsen--;
  } else if (ENC_A && !ENC_B) {
    encoder_pulsen++;
  } else if (!ENC_A && ENC_B) {
    encoder_pulsen++;
  }
}

void encoderB_ISR() {
  ENC_A = digitalRead(pinA);
  ENC_B = digitalRead(pinB);

  if (ENC_A && ENC_B) {
    encoder_pulsen++;  //increment the encoder's position count
  } else if (!ENC_A && !ENC_B) {
    encoder_pulsen++;
  } else if (ENC_A && !ENC_B) {
    encoder_pulsen--;
  } else if (!ENC_A && ENC_B) {
    encoder_pulsen--;
  }
}

void loop() {
  timer = millis();

  //======================= lees potmeter ==================================
  /*
    pot_val = 0.05 * analogRead(pot_pin) + 0.95 * pot_val;
    // setpoint_PWM = map(pot_val, 0, 1023, -400, 400);
    setpoint_pulsen = map(pot_val, 0, 1023, -400, 400);
  */
  //====================== smoothing acceleratie + debug ======================================

  if (homeing) {
    setpoint_PWM = setpoint_home_PWM;
  } else {
    setpoint_PWM = setpoint_PID_PWM;
  }

  int16_t setpoint_PWM_last_PWM = abs(setpoint_PWM) - abs(last_PWM);

  smoothing_PWM = sqrt(abs(setpoint_PWM_last_PWM));

  if ((setpoint_PWM > start_PWM) && (setpoint_PWM - last_PWM >= smoothing_PWM)) {
    if (last_PWM < start_PWM) {
      PWM = start_PWM;
      last_PWM = start_PWM - smoothing_PWM;
    }
    if (timer - last_smoothing_time >= smoothing_time) {
      PWM = last_PWM + smoothing_PWM;
      last_PWM = PWM;
      last_smoothing_time = timer;
    }

  } else if (setpoint_PWM > 0) {
    PWM = setpoint_PWM;

    if ((PWM > 200) && (amps < 200) && (PWM - last_PWM > 0)) {  // voor debug
      CAN_error = 1;
    } else {
      CAN_error = 0;
    }
    last_PWM = PWM;
  }

  if ((setpoint_PWM < -start_PWM) && (setpoint_PWM - last_PWM <= smoothing_PWM)) {
    if (last_PWM > -start_PWM) {
      PWM = -start_PWM;
      last_PWM = -start_PWM + smoothing_PWM;
    }
    if (timer - last_smoothing_time >= smoothing_time) {
      PWM = last_PWM - smoothing_PWM;
      last_PWM = PWM;
      last_smoothing_time = timer;
    }

  } else if (setpoint_PWM < 0) {
    PWM = setpoint_PWM;
    if ((PWM < -200) && (amps < 200) && (PWM - last_PWM < 0)) {  // voor debug
      CAN_error = 1;
    } else {
      CAN_error = 0;
    }
    last_PWM = PWM;
  }

  //===================== overcurrent detectie =============================

  if (timer - last_amps_poll > amps_poll_interval) {
    last_amps_poll = timer;
    amps = amps * 0.95 + md.getM2CurrentMilliamps() * 0.05;

    overcurrent_limit = (-3.8137 * PWM * PWM + 2456.2 * abs(PWM) + 159013) * 0.001 + 1000;
    if (amps > overcurrent_limit) {
      overcurrent = true;
      md.setM2Brake(400);
    }
  }

  //=================change direction detectie============================

  if (PWM > 0) {
    direction = 1;  // PWM is groter dan 0 dus positief
  } else {
    direction = 0;  // PWM is kleiner dan 0 dus negatef
  }

  if (direction != previus_direction) {
    direction_change = true;
    previus_direction = direction;

  } else if (timer - last_direction_change >= direction_change_delay) {
    last_direction_change = timer;
    direction_change = false;
  }

  //=============================== PWM naar motor ========================

  if (((PWM < 65) && (PWM > -65)) || (direction_change == true)) {
    md.setM2Brake(400);
    overcurrent = false;
  } else if (overcurrent == false) {
    md.setM2Speed(PWM);
  }

  //==================================== PID compute ===============================

  if (timer - last_PID >= PID_interval) {
    last_PID = timer;

    setpoint_pulsen = constrain(CAN_setpoint_pulsen, 0, max_pulsen);

    error = setpoint_pulsen - encoder_pulsen;
    //diff_error = 0.2 * (error - previus_error) + 0.8 * diff_error;
    //previus_error = error;

    P = kp * error;
    if (((abs(P) < 400) && (abs(PID) < 400)) || (direction_change == false)) {  // update de I alleen wanneer de motor nog niet op vol vermogen draait en niet op de rem staat omdat ie van richting verandert.
      I = ki * error + I;
    }
    //D = kd * diff_error;

    I = constrain(I, -200, 200);

    PID = P + I + D;
    PID = constrain(PID, -400, 400);
    setpoint_PID_PWM = PID;
  }

  //============================================================SerialPrints============================================

  if (timer - last_serial_print >= serial_print_interval) {
    last_serial_print = timer;

    P = constrain(P, -400, 400);
    //D = constrain(D, -400, 400);

    Serial.print(PID);
    Serial.print(" - ");
    Serial.print(P);
    Serial.print(" - ");
    Serial.print(I);
    Serial.print(" - ");
    Serial.println(D);

    /*
      //Serial.print(last_PWM);
      //Serial.print(" - ");

      Serial.print(error);
      Serial.print(" - ");
      Serial.print(encoder_pulsen);
      Serial.print(" - ");
      Serial.print(setpoint_PWM);
      Serial.print(" - ");
      Serial.print(PWM);
      Serial.print(" - ");
      Serial.print(amps);
      Serial.print(" - ");
      Serial.print(md.getM2CurrentMilliamps());
      Serial.print(" - ");
      Serial.println(overcurrent_limit);
    */
  }

  //============================================== send/read can data ===========================================================================

  //=========================== send_CAN_setpoint_PWM
  send_CAN_setpoint_PWM();

  //========================= send current
  send_CAN_current();

  //========================= read CAN
  read_CAN_data();  //read can data
}

void home() {
  const static uint8_t min_home_time = 100;
  static uint32_t last_home_time = timer;

#ifdef HOME_DEBUG
  Serial.println("homing");
#endif
  if (setpoint_home_PWM == 0) {                          // als het homen nog niet begonnen is
    setpoint_home_PWM = 400;                             // begin met homen
    last_home_time = timer;                              // reset last_home_time zodat de 100ms van mnin_home_time nu in gaat
  } else if (timer - last_home_time >= min_home_time) {  // wacht 100ms zodat amps niet meer 0 is door het filter.
    if (amps < 100) {                                    // als het stroom verbruik onder de 100mA is dan is de overcurrent getriggered en is de vleugel op zijn minimale stand.
      encoder_pulsen = 0;                                // reset de pulsen
      setpoint_pulsen = 0;                               // reset het setpoint
      setpoint_home_PWM = 0; // stop met gas geven. de volgdende keer dat de void home() gedaan wordt zal de 100ms timer weer worden gereset.
      CAN_setpoint_pulsen; // zet CAN_setpoin_pulsen op 0 zodat de vleugel niet direct terug gaat naar de vorige positie maar op het CAN bericht wacht
      I = 0; // zet de I van de PID op 0 zodat de motor niet spontaan begint te draaien
      homeing = false; // homen is klaar
#ifdef HOME_DEBUG
      Serial.println("homed");
#endif
    }
  }
}

void send_CAN_setpoint_PWM() {
  byte bytes[sizeof(int16_t)];                     //make an array and reserve the size of the datatype we want to send
  memcpy(bytes, &setpoint_PWM, sizeof(int16_t));   // copy the content of i16 to the array bytes until we hit the size of the int16 datatype
  for (uint8_t i = 0; i < sizeof(int16_t); i++) {  //basic counter
    ret.data[i] = bytes[i];                        //copy the data from bytes to their respective location in ret.bytes
  }
  ret.can_id = CAN_ID;            //set the can id of "ret" to our can id
  ret.can_dlc = sizeof(int16_t);  //set the dlc to the size of our data type (int16)
  //  return ret; //return the frame
  mcp2515.sendMessage(&ret);  //we send the setpoint_PWM as set by the PID to can ID 51
}

void send_CAN_current() {
  byte bytes[sizeof(int16_t)];                     //make an array and reserve the size of the datatype we want to send
  memcpy(bytes, &amps, sizeof(int16_t));           // copy the content of i16 to the array bytes until we hit the size of the int16 datatype
  for (uint8_t i = 0; i < sizeof(int16_t); i++) {  //basic counter
    ret.data[i] = bytes[i];                        //copy the data from bytes to their respective location in ret.bytes
  }
  ret.can_id = CAN_ID;            //set the can id of "ret" to our can id
  ret.can_dlc = sizeof(int16_t);  //set the dlc to the size of our data type (int16)
  //  return ret; //return the frame
  mcp2515.sendMessage(&ret);  //we send the setpoint_PWM as set by the PID to can ID 51
}

void read_CAN_data() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    if (canMsg.can_id == 0xC8) {                                             //is can msg ID is 200 in hex
      CAN_setpoint_pulsen = int16_from_can(canMsg.data[4], canMsg.data[5]);  //byte 4-5 is int16_t pulsen achter
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

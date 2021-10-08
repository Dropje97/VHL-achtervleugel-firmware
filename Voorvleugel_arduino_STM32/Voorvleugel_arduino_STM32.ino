/*
   project.cpp

    Created on: Jun 16, 2021
        Author: john
*/
#include "mcp2515.h"
//#include "DualVNH5019MotorShield.h"
MCP2515 mcp2515(PB12);  //compleet willekeurige pin want ER WAS NOG GEEN PIN
//DualVNH5019MotorShield motor_shield();

#include "types.h"
#include "motor_config.h"
#include "mcp2515.h"
#include "can.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>

//#define HOME_DEBUG

#define ENC1_PIN_1 PB2
#define ENC1_PIN_2 PB1
#define ENC2_PIN_1 PB15
#define ENC2_PIN_2 PB14

int delay_time = 50;

//String read_buffer = "";
//String cmd = "";
const int bufferSize = 200;
const char cmd_sep = '|';
int error_max = 2;

bool drawing_graph = false;

bool has_homed = false;
float pulses_max = 19000;

Motor motor_1 = get_motor(M1);
Motor motor_2 = get_motor(M2);

can_frame canMsg1;
can_frame canMsg2;

//CAN_Error err1;
//CAN_Error err2;

//vars for command parser
const int8_t parser_bufsize = 100;
const char *cmnds_strs[] = {"HOME", "MOVE", "STOP"};
struct {
  char readbuf[parser_bufsize] = "HOME|MOVsE 1000|STOsP|BLA|";
  char parsebuf[parser_bufsize];
  enum { home, move, stop} cmnds_enum;

} parser;


//DualVNH5019MotorShield motor_shield;

void ret() {
  printf("\n");
}

void encoder1_ISR() {
  if (digitalRead(ENC1_PIN_2)) {
    motor_1.encoder_pulses++;
  } else {
    motor_1.encoder_pulses--;
  }
}

void encoder2_ISR() {
  if (digitalRead(ENC2_PIN_2)) {
    motor_2.encoder_pulses++;
  } else {
    motor_2.encoder_pulses--;
  }
}


void compute_pid(Motor m) {
  float error =  (float)(m.setpoint + m.offset - m.encoder_pulses - m.offset_fixed);
  if (error > error_max * -1 && error < error_max) {
    m.set_speed(0);
    m.pwm = 0;
    if (drawing_graph) {
      drawing_graph = false;
      printf("DONE GRAPH|\n");
    }
    return;
  }

  ulong cycle_time = HAL_GetTick() - m.millis_last;

  m.i_term_result += m.i_term * error * (float) cycle_time;
  if (m.i_term_result > m.out_max)
    m.i_term_result = m.out_max;       // that the I term from PID gets too big
  else if (m.i_term_result < m.out_min)
    m.i_term_result = m.out_min;

  float speed = (error - m.pref_error) / (float)cycle_time;
  m.pid_out = m.p_term * error + m.i_term_result - m.d_term * speed;// PID m.pid_out is the sum of P I and D values

  m.pref_error = error;

  if (m.pid_out > 0) {
    if (m.pid_out > m.out_max)
      m.pid_out = m.out_max;
    m.pwm = (int)m.pid_out + m.pwm_min;
  } else {
    if (m.pid_out < m.out_min)
      m.pid_out = m.out_min;
    m.pwm = (int)m.pid_out - m.pwm_min;
  }
  m.set_speed(m.pwm);
  m.millis_last = HAL_GetTick();
  printf("%d \n", m.setpoint);
}

int sample_motor_current(Motor m) {
  // samples the motor_1 current 3 times 1 ms apart and take the average.
  // this is done in order to improve stablility
  int current = 0;
  const int num_measurements = 50;
  for (int i = 0; i < num_measurements; i++) {
    current += m.get_current();
    //HAL_Delay(1);
  }
  return current / num_measurements - m.current_offset;
}

void home_actuators() {
  // homing sequence blocking routine

  const int motor_1_stop_current = 70;
  const int motor_2_stop_current = 150;

#ifdef HOME_DEBUG
  printf("homing\n");
#endif
  motor_1.set_speed(motor_1.home_pwm_high);
  motor_2.set_speed(motor_2.home_pwm_high);
  int motor_current_1 = 0;
  int motor_current_2 = 0;
  HAL_Delay(100);
  motor_current_1 = sample_motor_current(motor_1);
  motor_current_2 = sample_motor_current(motor_2);
  printf("cur %d %d \n", motor_current_1, motor_current_2);
  while (motor_current_1 > motor_1_stop_current ||
         motor_current_2 > motor_2_stop_current) {
#ifdef HOME_DEBUG
    //printf("%d %d", encoder1_pulses, encoder2_pulses);
    printf("cur %d %d \n", motor_current_2, motor_current_2);
#endif
    HAL_Delay(100);
    motor_current_1 = sample_motor_current(motor_1);
    motor_current_1 = sample_motor_current(motor_2);
  }
#ifdef HOME_DEBUG
  printf("stage 1 done\n");
#endif
  motor_1.set_speed( -1 * motor_1.home_pwm_high);
  motor_2.set_speed( -1 * motor_2.home_pwm_high);
  HAL_Delay(1000);
  motor_1.set_speed(motor_1.home_pwm_high);
  motor_2.set_speed(motor_2.home_pwm_high);
  HAL_Delay(20);
  motor_current_1 = sample_motor_current(motor_1);
  motor_current_2 = sample_motor_current(motor_2);

  while (motor_current_1 > motor_1_stop_current ||
         motor_current_2 > motor_2_stop_current) {
#ifdef HOME_DEBUG
    printf("stage 2\n");
#endif
    delay(97);
    motor_current_1 = sample_motor_current(motor_1);
    motor_current_2 = sample_motor_current(motor_2);
  }
  motor_1.encoder_pulses = 0;
  motor_2.encoder_pulses = 0;
  motor_1.setpoint = 0;
  motor_2.setpoint = 0;
  motor_1.set_speed(0);
  motor_2.set_speed(0);

  has_homed = true;
#ifdef HOME_DEBUG
  printf("homed\n");
#endif

}
/*
void user_init() {
  canMsg1.can_id  = 0x0F6;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0xEE;
  canMsg1.data[1] = 0xFF;
  canMsg1.data[2] = 0xEE;
  canMsg1.data[3] = 0xEE;
  canMsg1.data[4] = 0xEE;
  canMsg1.data[5] = 0xEE;
  canMsg1.data[6] = 0xEE;
  canMsg1.data[7] = 0xEE;

  canMsg2.can_id  = 0x036;
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0xAA;
  canMsg2.data[1] = 0xFF;
  canMsg2.data[2] = 0xAA;
  canMsg2.data[3] = 0xAA;
  canMsg2.data[4] = 0xAA;
  canMsg2.data[5] = 0xAA;
  canMsg2.data[6] = 0xAA;
  canMsg2.data[7] = 0xAA;

 // motor_shield = DualVNH5019MotorShield();

}
*/
/*
void parse_uart_cmds() {
  /* TODOS
     read uart into buffer char by char.
     extract possible commands with strtok
     if it finds something remove everything until the first null char from the string (find with sizeof?)
     split the string in two (trim before use)
     check first command string for command type
   * */
/*
  //char cmdbuf[100];
  strcpy(parser.parsebuf, parser.readbuf);
  if (strtok(parser.parsebuf, "|") != NULL) { // if it found a cmd sep
    // copies command buffer without detected command back to read buffer
    strcpy(parser.readbuf, parser.parsebuf + strlen(parser.parsebuf) + 1);

    printf(parser.parsebuf);
    ret();
    printf(parser.readbuf);
    ret();

    if (strncmp(parser.parsebuf, cmnds_strs[parser.home], strlen(cmnds_strs[parser.home])) == 0) {
      printf("found home");
      ret();
      home_actuators();
    } else if (strncmp(parser.parsebuf, cmnds_strs[parser.stop], strlen(cmnds_strs[parser.stop])) == 0) {
      printf("found stop");
      ret();
      motor_1.setpoint = motor_1.encoder_pulses;
      motor_2.setpoint = motor_2.encoder_pulses;
    } else if (strncmp(parser.parsebuf, cmnds_strs[parser.move], strlen(cmnds_strs[parser.move])) == 0) {
      printf("found move");
      ret();
    } else {
      printf("no valid cmd found");
      ret();
    }
  } else {
    printf("no cmd found");
    ret();
  }
}
*/
/*
void user_while() {
  //	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  //	printf("%d\n", MCP_sendMessage(&canMsg1));
  //	printf("%d\n", MCP_sendMessage(&canMsg2));

  //printf("start\n");

  
  	parse_uart_cmds();
  	parse_uart_cmds();
  	parse_uart_cmds();
  	parse_uart_cmds();
  	parse_uart_cmds();
  

  //	printf("done\n");

  //home_actuators();
  HAL_Delay(5000);
}
*/

void setup() {
  Serial.begin(115200);
  // configure interrupts and timers
  attachInterrupt(digitalPinToInterrupt(ENC1_PIN_1), encoder1_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC2_PIN_1), encoder2_ISR, FALLING);
  motor_shield.init();
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
  delay(10);

  Serial.println("ready");
}

void loop() {
  delay(delay_time);
 // process_serial_cmd();

#ifdef PRINT_PULSE
  Serial.print(motor_1.encoder_pulses);
  Serial.print(" ");
  Serial.println(motor_2.encoder_pulses);
#endif
  if (has_homed) {
    compute_pid(motor_1);
    compute_pid(motor_2);
  }


#ifdef PRINT_A
  Serial.print(analogRead(pos_pin));
  Serial.print(" ");
  Serial.println(analogRead(offset_pin));
#endif

/*

  if (drawing_graph) {
    Serial.print(motor_0.pwm);
    Serial.print(' ');
    Serial.println(motor_0.encoder_pulses - motor_0.setpoint);
  }
  */
}

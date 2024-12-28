#define ARRLEN(x) (sizeof(x) / sizeof(x[0]))
#define MAX(X,Y)((X)>(Y)?(X):(Y))

#define DEBUG 0   // SET TO 0 TO REMOVE TRACES

#if DEBUG
#define DBG(FCT,...) Serial.FCT(__VA_ARGS__)
#else
#define DBG(FCT,...) do { } while (0)
#endif

#include "Arduino.h"


uint32_t now_;

/**********************************
 *                                *
 *           P  W  M              *
 *                                *
 **********************************/

#define PWM_MIN_DUTY 10
#define PWM_MAX_DUTY 254
#define PWM_DEFAULT_DUTY PWM_MIN_DUTY
 
#define PWM_FREQ 30000UL

struct pwm_state {
  uint8_t duty;
 } pwm[1];


void pwm_set_duty(int duty_0) 
{
  if (duty_0 < PWM_MIN_DUTY) 
    duty_0 = PWM_MIN_DUTY;
  if (duty_0 > PWM_MAX_DUTY)
    duty_0 = PWM_MAX_DUTY;
  TCA0.SPLIT.HCMP0  = pwm[0].duty = duty_0;  // set duty cycle 0
//  TCA0.SPLIT.HCMP1  = duty_1;  // set duty cycle 1
//  TCA0.SPLIT.HCMP2  = duty_2;  // set duty cycle 2
}

void pwm_init()
{
  takeOverTCA0();

  pinMode(PIN_PA3, OUTPUT);                       // activate PWM_0
//  pinMode(PIN_PA4, OUTPUT);                       // activate PWM_1
//  pinMode(PIN_PA5, OUTPUT);                       // activate PWM_2

//  PORTA.PIN4CTRL |= PORT_INVEN_bm;                  // invert PWM_1 output
//  PORTA.PIN5CTRL |= PORT_INVEN_bm;                  // invert PWM_2 output

  TCA0.SPLIT.CTRLA &= ~(TCA_SPLIT_ENABLE_bm);     // disable TCA0
  TCA0.SPLIT.CTRLESET |= TCA_SPLIT_CMD_RESET_gc;  // reset TCA0

  TCA0.SPLIT.CTRLD  = TCA_SPLIT_SPLITM_bm;        // set Split Mode
  TCA0.SPLIT.CTRLA  = TCA_SPLIT_CLKSEL_DIV1_gc;   // set clock divider (about 16kHz @5MHz)
  TCA0.SPLIT.CTRLB  = 
                        TCA_SPLIT_HCMP0EN_bm      // activate CMP output hi 0 aka WO3
//                      | TCA_SPLIT_HCMP1EN_bm      // activate CMP output hi 1 aka WO4
//                      | TCA_SPLIT_HCMP2EN_bm      // activate CMP output hi 2 aka WO5
                      ;
  TCA0.SPLIT.HPER   = 255;                        // set period  
  pwm_set_duty(PWM_DEFAULT_DUTY);                 // set duty
  TCA0.SPLIT.CTRLA |= TCA_SPLIT_ENABLE_bm;       // enable TCA0
}


/**********************************
 *                                *
 *         E N C O D E R          *
 *                                *
 **********************************/
#define CONFIG_ENCODER_2TICKS 1

#define ENCODER_FLAG_MIDSTEP    0x01
#define ENCODER_FLAG_FORW_EDGE1 0x02
#define ENCODER_FLAG_BACK_EDGE1 0x04
#define ENCODER_FLAG_FORW_EDGE2 0x08
#define ENCODER_FLAG_BACK_EDGE2 0x10

// encoder value change map
// each  encoder state flag combination is mapped to its 
// resulting value change.
// Requirements for value change: 
//   both, first and last edge, or at least one edge 
//   and the middle state of only one direction must be set 
//   if an edge is missing accepting the middle state
//   will reject bounces and false movements

int8_t enc_map[32] {
                       0,  0,  CONFIG_ENCODER_2TICKS,  1, -CONFIG_ENCODER_2TICKS, -1,  0,  0, // 00  - 07
   CONFIG_ENCODER_2TICKS,  1,                      1,  1,                      0,  0,  0,  0, // 08  - 0f
  -CONFIG_ENCODER_2TICKS, -1,                      0,  0,                     -1, -1,  0,  0, // 10  - 17
                       0,  0,                      0,  0,                      0,  0,  0,  0  // 18  - 1f
};

struct enc {
  uint8_t last_pos;
  uint8_t state;
  int32_t value;

} enc;

void enc_init(int32_t value) {
  pinMode(PIN_PB0, INPUT_PULLUP);
  pinMode(PIN_PB1, INPUT_PULLUP);
  enc.last_pos = PORTB_IN & 0x03;
  enc.state = 0;
  enc.value = value;
}
/*
 * returns nonzero on encoder input 
 */
uint8_t enc_update() {

  uint8_t enc_cur_pos = PORTB_IN & 0x03;
  uint8_t ret = 0;

  if (enc_cur_pos != enc.last_pos) {

    if (enc.last_pos == 0x00) {
      if      (enc_cur_pos == 0x02) {
        enc.state |= ENCODER_FLAG_FORW_EDGE1;
      }
      else if (enc_cur_pos == 0x01) {
        enc.state |= ENCODER_FLAG_BACK_EDGE1;
      }
    } else if (enc.last_pos == 0x03) {
      // this is the second edge
      if      (enc_cur_pos == 0x01) {
        enc.state |= ENCODER_FLAG_FORW_EDGE2;
      }
      else if (enc_cur_pos == 0x02) {
        enc.state |= ENCODER_FLAG_BACK_EDGE2;
      }
    }
    if ((enc_cur_pos == 0x03 && ! CONFIG_ENCODER_2TICKS)) {
      enc.state |= ENCODER_FLAG_MIDSTEP;
    }

    if (enc_cur_pos == 0x00 || ((enc_cur_pos == 0x03) && CONFIG_ENCODER_2TICKS))
    {
      // this is when the encoder is in a 'rest' state
      int8_t add = enc_map[enc.state & 0x1F];
      if (add) {
        enc.value += add;

        DBG(print, "Enc "); DBG(print, add); DBG(print, " -> "); DBG(println, enc.value);
        ret = 1;
      }
      enc.state = 0; // reset for next time
    }
  }
  enc.last_pos = enc_cur_pos;
  return ret; 
}


/**********************************
 *                                *
 *       D I R B U T T O N        *
 *                                *
 **********************************/
#define PIN_DIRBTN_FWD PIN_PA6
#define PIN_DIRBTN_REV PIN_PA7
#define PIN_DIRBTN_VAL ((PORTA_IN & 0xC0) >> 6)

#define PIN_MOTOR_FWD PIN_PA1
#define PIN_MOTOR_REV PIN_PA2


void dir_init(void) {
  pinMode(PIN_DIRBTN_FWD, INPUT_PULLUP);
  pinMode(PIN_DIRBTN_REV, INPUT_PULLUP);

  pinMode(PIN_MOTOR_FWD, OUTPUT);
  pinMode(PIN_MOTOR_REV, OUTPUT);
  PORTA.PIN2CTRL |= PORT_INVEN_bm;
  PORTA.PIN1CTRL |= PORT_INVEN_bm;

}

void dir_update(void) {
  uint8_t dir = PIN_DIRBTN_VAL;
  PORTA_OUT = (PORTA_OUT & ~0x06) | (dir << 1);
}

/**********************************
 *                                *
 *            P  I  D             *
 *                                *
 **********************************/
#define PID_FACTOR 0.05f
#define PID_P_DEFAULT ( 2.00f * PID_FACTOR)
#define PID_I_DEFAULT ( 0.00f * PID_FACTOR)
#define PID_D_DEFAULT ( 0.00f * PID_FACTOR)

#if 0   // if 1 we calculate the PID with fractional numbers if 0 we use float.
        // spoiler alert: on an ATtiny817@20MHz floats are faster. (~110µs  vs. fractional: ~130µs)

typedef int32_t pid_frac;

#define PID_FRAC_BITS 5  // PIDs calculation fixcomma position

#define PID_INT16_MUL_FRAC_TO_FRAC(x,frac) (((((int32_t) (x) << PID_FRAC_BITS)) * ((pid_frac) (frac))) >> PID_FRAC_BITS)
#define PID_MAKE_FRAC_FROM_FLOAT(x) ((pid_frac) ((x) * ((float) (1 << PID_FRAC_BITS))))
#define PID_MAKE_INT16_FROM_FRAC(x) ((int16_t) ((x) >> PID_FRAC_BITS))

#else

typedef float pid_frac;

#define PID_INT16_MUL_FRAC_TO_FRAC(x,y) ((float) (x) * y)
#define PID_MAKE_FRAC_FROM_FLOAT(x) (x)
#define PID_MAKE_INT16_FROM_FRAC(x) ((int16_t) (x))

#endif

struct pid
{
  int16_t  d_state; // Last position input
  int16_t  i_state; // Integrator state
  int16_t  i_max;   // Maximum and
  int16_t  i_min;   // Minimum allowable integrator state
  pid_frac i_gain;  // integral gain
  pid_frac p_gain;  // proportional gain
  pid_frac d_gain;  // derivative gain
} pid;

void pid_init(float p_gain, float i_gain, float d_gain, int16_t i_min, int16_t i_max, int16_t init_state) {
  pid.p_gain = PID_MAKE_FRAC_FROM_FLOAT(p_gain);
  pid.i_gain = PID_MAKE_FRAC_FROM_FLOAT(i_gain);
  pid.d_gain = PID_MAKE_FRAC_FROM_FLOAT(d_gain);
  pid.i_min = i_min;
  pid.i_max = i_max;
  pid.d_state = init_state;
  pid.i_state = 0;
}

int16_t pid_update(int16_t error, int16_t position)
{
  pid_frac pTerm, dTerm, iTerm;

  pTerm = PID_INT16_MUL_FRAC_TO_FRAC(error, pid.p_gain); // calculate the proportional term
  
  pid.i_state += error; // calculate the integral state with appropriate limiting
  if (pid.i_state > pid.i_max) // Limit the integrator state if necessary
    pid.i_state = pid.i_max;
  else if (pid.i_state < pid.i_min)
    pid.i_state = pid.i_min;

  iTerm = PID_INT16_MUL_FRAC_TO_FRAC(pid.i_state, pid.i_gain); // calculate the integral term

  dTerm = PID_INT16_MUL_FRAC_TO_FRAC(pid.d_state - position, pid.d_gain); // calculate the derivative

  pid.d_state = position;

  return PID_MAKE_INT16_FROM_FRAC(pTerm + dTerm + iTerm);
}


/**********************************
 *                                *
 *        V E L O C I T Y         *
 *                                *
 **********************************/
#define VEL_PIN_A PIN_PA4
#define VEL_PIN_B PIN_PA5

#define VEL_PIN_EVAL ((PORTA_IN & 0x30) >> 4)

#define VEL_DEFAULT_VAL 250 // velocity in encoder pulse per second

#define VEL_PROBE_FREQ 25 // probe period in Hz

volatile uint16_t vel_cnt_a;
volatile uint16_t vel_cnt_b;

ISR(PORTA_PORT_vect) {
  uint8_t flags = PORTA.INTFLAGS;   // read which pins triggered the int
  if (flags & 0b00010000)               
    ++vel_cnt_a;
  if (flags & 0b00100000)               
    ++vel_cnt_b;
  PORTA.INTFLAGS = flags;          // Clear the flags by writing a 1 to them (not a 0);
}

struct velocity_state {
  uint16_t nom_velocity;
  uint8_t  gpi_last_state;
  int32_t  last_duty;
  uint32_t last_velocity;
} vel;

void vel_init(void) {
  pinMode(VEL_PIN_A, INPUT);
  pinMode(VEL_PIN_B, INPUT);

  // Enable the interrupt on change.
  PORTA.PIN4CTRL |= PORT_ISC_BOTHEDGES_gc; // PA4
  PORTA.PIN5CTRL |= PORT_ISC_BOTHEDGES_gc; // PA5

  vel.nom_velocity   = VEL_DEFAULT_VAL / VEL_PROBE_FREQ;
  vel.last_velocity  = 0;
  vel.last_duty      = 128;
  vel.gpi_last_state = VEL_PIN_EVAL;
}

void vel_set_nominal(uint32_t nom_velocity) {
  vel.nom_velocity = nom_velocity;
}

/*
 * controls pwm duty cycle
 */
void vel_update(void) {
  static uint32_t last_check = now_;
  if (now_ - last_check > (1000 / VEL_PROBE_FREQ)) {
    int32_t next_duty;
    int16_t pid;
    uint16_t act_velocity = vel_cnt_a;
    vel_cnt_a = 0;
    vel_cnt_b = 0;
    pid = pid_update(vel.nom_velocity - act_velocity, act_velocity);
    next_duty = vel.last_duty + pid;
    if (next_duty < 0)
      next_duty = 0;
    if (next_duty > 254)
      next_duty = 254;    
      
    DBG(print, now_ - last_check);
    DBG(print, ":   Vel ");
    DBG(print, vel.last_velocity);
    DBG(print, "->");
    DBG(print, act_velocity);
    DBG(print, "    PID ");
    DBG(print, pid);
    DBG(print, "     Duty ");
    DBG(print, vel.last_duty);
    DBG(print, "->");
    DBG(println, next_duty);
    
    vel.last_velocity = act_velocity;
    vel.last_duty     = next_duty;   
    last_check        = now_;
    pwm_set_duty(next_duty); 
  }
}



/**********************************
 *                                *
 *          M  A  I N             *
 *                                *
 **********************************/

void setup() {
  now_ = micros();

  DBG(begin,115200);
  delay(750);

  DBG(print, "PORT A - Dir: 0x");
  DBG(print, PORTA_DIR, HEX);
  DBG(print, ", Val: 0x");
  DBG(print, PORTA_IN, HEX);
  DBG(print, "    PORT B - Dir: 0x");
  DBG(print, PORTB_DIR, HEX);
  DBG(print, "  Val: 0x");
  DBG(println, PORTB_IN, HEX);

  DBG(print, "Init... "); 
  pwm_init();
  vel_init();
  enc_init(vel.nom_velocity);
  dir_init();
  pid_init(PID_P_DEFAULT, PID_I_DEFAULT, PID_D_DEFAULT, -128, 128, vel.nom_velocity);
  DBG(println, " ...Done.");

  DBG(print, "PORT A - Dir: 0x");
  DBG(print, PORTA_DIR, HEX);
  DBG(print, ", Val: 0x");
  DBG(print, PORTA_IN, HEX);
  DBG(print, "    PORT B - Dir: 0x");
  DBG(print, PORTB_DIR, HEX);
  DBG(print, "  Val: 0x");
  DBG(println, PORTB_IN, HEX);
  DBG(println, "Run..."); 
}


void loop() {
  now_ = millis();

  enc_update();
  vel.nom_velocity = enc.value; 

#if 1
  pwm_set_duty(enc.value);
#else
  vel_update();
#endif

  dir_update();
}
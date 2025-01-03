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

#define PWM_MIN_DUTY 20
#define PWM_MAX_DUTY 254
#define PWM_DEFAULT_DUTY PWM_MIN_DUTY
 
#define PWM_FREQ 30000UL

void pwm_set_duty(int duty) 
{
  if (duty < PWM_MIN_DUTY) 
    duty = PWM_MIN_DUTY;
  if (duty > PWM_MAX_DUTY)
    duty = PWM_MAX_DUTY;

//  TCA0.SPLIT.HCMP0  = duty;  // set duty cycle for WO3
//  TCA0.SPLIT.HCMP1  = duty;  // set duty cycle for WO4
  TCA0.SPLIT.HCMP2  = duty;  // set duty cycle for WO5
}

void pwm_init()
{
  takeOverTCA0();

//  pinMode(PIN_PA3, OUTPUT);                       // activate PWM for WO3
//  pinMode(PIN_PA4, OUTPUT);                       // activate PWM for WO4
  pinMode(PIN_PA5, OUTPUT);                       // activate PWM for WO5

//  PORTA.PIN3CTRL |= PORT_INVEN_bm;                  // invert PWM output for WO3
//  PORTA.PIN4CTRL |= PORT_INVEN_bm;                  // invert PWM output for WO4
//  PORTA.PIN5CTRL |= PORT_INVEN_bm;                  // invert PWM output for WO5

  TCA0.SPLIT.CTRLA &= ~(TCA_SPLIT_ENABLE_bm);     // disable TCA0
  TCA0.SPLIT.CTRLESET |= TCA_SPLIT_CMD_RESET_gc;  // reset TCA0

  TCA0.SPLIT.CTRLD  = TCA_SPLIT_SPLITM_bm;        // set Split Mode
  TCA0.SPLIT.CTRLA  = TCA_SPLIT_CLKSEL_DIV1_gc;   // set clock divider (about 16kHz @5MHz)
  TCA0.SPLIT.CTRLB  = 0;

//  TCA0.SPLIT.CTRLB  |= TCA_SPLIT_HCMP0EN_bm;      // activate CMP output hi 0 aka WO3
//  TCA0.SPLIT.CTRLB  |= TCA_SPLIT_HCMP1EN_bm;      // activate CMP output hi 1 aka WO4
  TCA0.SPLIT.CTRLB  |= TCA_SPLIT_HCMP2EN_bm;      // activate CMP output hi 2 aka WO5

  TCA0.SPLIT.HPER   = 255;                        // set period  
  pwm_set_duty(PWM_DEFAULT_DUTY);                 // set duty
  TCA0.SPLIT.CTRLA |= TCA_SPLIT_ENABLE_bm;       // enable TCA0
}



/**********************************
 *                                *
 *          M O T O R             *
 *                                *
 **********************************/
#define MOTOR_PIN_FWD PIN_PA6
#define MOTOR_PIN_REV PIN_PA7

#define MOTOR_DIR_NONE 0
#define MOTOR_DIR_FWD  1
#define MOTOR_DIR_REV  2
#define MOTOR_DIR_STOP 3


void motor_init(void) {
  pinMode(MOTOR_PIN_FWD, OUTPUT);
  pinMode(MOTOR_PIN_REV, OUTPUT);
  PORTA.PIN2CTRL |= PORT_INVEN_bm;
  PORTA.PIN1CTRL |= PORT_INVEN_bm;

  pwm_init();
}

#define motor_set_vel(VEL) pwm_set_duty(VEL)

void motor_set_dir(uint8_t dir) {
  PORTA_OUT = (PORTA_OUT & ~0b11000000U) | (dir << 6);
}





/**********************************
 *                                *
 *         E N C O D E R          *
 *                                *
 **********************************/
#define ENCODER_PIN_A PIN_PB0
#define ENCODER_PIN_B PIN_PB1
#define ENCODER_PIN_VAL (PORTB_IN & 0x03)

// lookup table for combined values of last and act enc:
// last val: bit 2/3 
// act  val: bit 0/1

int8_t enc_stepper_map[16] = {
 //   act == 0   1   2   3 
             0,  1, -1,  0,   // 0 == last
            -1,  0,  0,  1,   // 1
             1,  0,  0, -1,   // 2
             0, -1,  1,  0    // 3
};

struct enc {
  uint8_t last_state;
  int32_t value;
} enc;

void enc_init(int32_t value) {
  pinMode(ENCODER_PIN_A, INPUT);
  pinMode(ENCODER_PIN_B, INPUT);
  enc.last_state = ENCODER_PIN_VAL;
  enc.value = value;
}
/*
 * returns nonzero on encoder input 
 */
int8_t enc_update() {
  uint8_t act_state = ENCODER_PIN_VAL;
  int8_t  ret = enc_stepper_map[act_state | (enc.last_state << 2)];
  enc.value += ret; 
  enc.last_state = act_state;
  return ret; 
}



/**********************************
 *                                *
 *            P  I  D             *
 *                                *
 **********************************/
#define PID_FACTOR 0.05f
#define PID_P_DEFAULT ( 20.00f * PID_FACTOR)
#define PID_I_DEFAULT (  0.00f * PID_FACTOR)
#define PID_D_DEFAULT (  0.00f * PID_FACTOR)

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
  now_ = millis();

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
  //pid.p_gain = PID_FACTOR * (float) enc.value + PID_P_DEFAULT; 

#if 0
  pwm_set_duty(enc.value);
#else
  vel_update();
#endif

  dir_update();
}
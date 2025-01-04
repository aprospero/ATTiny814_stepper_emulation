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
#define PWM_DUTY_DEFAULT 128   // 50% duty 
#define PWM_DUTY_MAX 254       // must be in [1..254]

#define PWM_FREQ 30000UL

uint8_t pwm_get_duty() 
{
//  if (id == 0) return TCA0.SPLIT.HCMP0;  // get duty cycle for WO3
//  if (id == 1) return TCA0.SPLIT.HCMP1;  // get duty cycle for WO4
//  if (id == 2) 
  return TCA0.SPLIT.HCMP2;  // get duty cycle for WO5
}

void pwm_set_duty(/* uint8_t id, */uint8_t duty) 
{
//  if (id == 0) TCA0.SPLIT.HCMP0  = duty;  // set duty cycle for WO3
//  if (id == 1) TCA0.SPLIT.HCMP1  = duty;  // set duty cycle for WO4
//  if (id == 2) 
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

  TCA0.SPLIT.HPER   = PWM_DUTY_MAX + 1;           // set period  
//  pwm_set_duty(0, PWM_DUTY_DEFAULT);              // set duty for WO3
//  pwm_set_duty(1, PWM_DUTY_DEFAULT);              // set duty for WO4
  pwm_set_duty(/*2, */PWM_DUTY_DEFAULT);              // set duty for WO5
  TCA0.SPLIT.CTRLA |= TCA_SPLIT_ENABLE_bm;        // enable TCA0
}



/**********************************
 *                                *
 *          M O T O R             *
 *                                *
 **********************************/
#define MOTOR_PIN_FWD PIN_PA6
#define MOTOR_PIN_REV PIN_PA7

#define MOTOR_TORQUE_MIN 30
#define MOTOR_TORQUE_MAX PWM_DUTY_MAX


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

void motor_set_torque(uint32_t torque) {
  if (torque < MOTOR_TORQUE_MIN) 
    torque = MOTOR_TORQUE_MIN;
  if (torque > MOTOR_TORQUE_MAX)
    torque = MOTOR_TORQUE_MAX;
  pwm_set_duty(torque);
}

#define motor_get_torque pwm_get_duty

void motor_set_dir(uint8_t dir) {
  PORTA_OUT = (PORTA_OUT & ~0b11000000U) | (dir << 6);
}

uint8_t motor_get_dir() {
  return PORTA_OUT >> 6;
}





/**********************************
 *                                *
 *         E N C O D E R          *
 *                                *
 **********************************/
#define ENCODER_PIN_A PIN_PB0
#define ENCODER_PIN_B PIN_PB1
#define ENCODER_PIN_VAL (PORTB_IN & 0x03)

/* 
 * lookup table for combinations of last and act enc states.
 * Encoder A & B states are treated as a 2bit field.
 * delta = enc_stepper_map[last][act];
 */
int8_t enc_stepper_map[4][4] = {
 //   act == 0   1   2   3 
          {  0,  1, -1,  0 },   // 0 == last
          { -1,  0,  0,  1 },   // 1
          {  1,  0,  0, -1 },   // 2
          {  0, -1,  1,  0 }    // 3
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
  int8_t  delta = enc_stepper_map[enc.last_state][act_state];
  enc.value += delta; 
  enc.last_state = act_state;
  return delta; 
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
 * S T E P   T R A N S L A T O R  *
 *                                *
 **********************************/
#define STEP_PROBE_FREQ 25       // in Hz - how frequently do we update motor torque
#define STEP_TOLERATED_ERROR 8   // in ticks - the absolute max accepted error between nominal and actual step.

#define STEP_PIN_STEP PIN_PA3
#define STEP_PIN_DIR  PIN_PA2

#define STEP_PIN_DIR_VAL ((PORTA_IN & 0b00000100) >> 2)
#define STEP_PIN_STEP_MASK 0b00001000

volatile int32_t step_cnt;

ISR(PORTA_PORT_vect) {
  step_cnt += (STEP_PIN_DIR_VAL << 1) - 1;  // dir pin state changes sign
  PORTA.INTFLAGS = STEP_PIN_STEP_MASK;
}

void step_init(void) {
  pinMode(STEP_PIN_STEP, INPUT);
  pinMode(STEP_PIN_DIR,  INPUT);
  
  PORTA.PIN3CTRL |= PORT_ISC_RISING_gc;   // Enable interrupt on step input pin.
  step_cnt = 0;

  enc_init(0);
  pid_init(PID_P_DEFAULT, PID_I_DEFAULT, PID_D_DEFAULT, -128, 128, 0);
  motor_init();
}

/*
 * controls pwm duty cycle
 */
void step_update(void) {
  static uint32_t last_check = now_;

  enc_update();

  if (now_ - last_check > (1000 / STEP_PROBE_FREQ)) {
    int32_t pos_err = step_cnt - enc.value;
    int16_t pid = pid_update(pos_err, enc.value);
    if (pos_err < STEP_TOLERATED_ERROR)
      motor_set_dir(MOTOR_DIR_REV);
    else if (pos_err > STEP_TOLERATED_ERROR)
      motor_set_dir(MOTOR_DIR_FWD);
    else
      motor_set_dir(MOTOR_DIR_NONE);
    motor_set_torque(abs(pid));
      
    DBG(print, now_ - last_check);
    DBG(print, ":   Step ");
    DBG(print, enc.value);
    DBG(print, "->");
    DBG(print, step_cnt);
    DBG(print, "    PID ");
    DBG(print, pid);
    DBG(print, "     Duty ");
    DBG(println, pid);
    
    last_check        = now_;
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
  step_init();
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

  step_update();
}
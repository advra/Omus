#ifndef _OMUS5_H
#define	_OMUS5_H

#include <xc.h>        // include processor files - each processor file is guarded.
#include <p32xxxx.h>   // include chip specific header file
#include <plib.h>      // include peripheral library functions
#include <stdlib.h>

// ************ DEFINES ****************
#define PWM_FREQ    16000
#define SYSCLK      40000000L
#define AINPUTS     0x0E37

// ************ DIR CONTROL **************
#define SPD_0      0
#define SPD_1      250
#define SPD_2      500
#define SPD_3      750
#define SPD_4      1000
#define SPD_5      1250
#define SPD_6      1500
#define SPD_7      1750
#define SPD_7_5    1875
#define SPD_8      2000
#define SPD_9      2250
#define SPD_10     2500

#define FRWD        1
#define REV         0

#define OFF         0
#define REV_LEFT    1
#define REV_RIGHT   2
#define ROT_LEFT    3
#define ROT_RIGHT   4
#define FWD         5
#define PIVOT       6
#define REV_STR     7
#define FULL_PIV    8
#define PIVOT_L     9
#define PIVOT_R     10
#define DODGE       11
#define ROT_LEFT_F  12
#define ROT_RIGHT_F 13

// ********** STATES *************
#define WAIT            0
#define ESCAPE          1
#define SEEK            2
#define ATTACK1         3
#define ATTACK2         4

// **** SUB STATES (WAIT))
#define WAIT_PRE        0
#define WAIT_0          1
#define WAIT_1          2
#define WAIT_2          3

// ******** THRESHOLDS *************
#define LINE_THRESH     250
#define PROX_THRESH     350

// ************** GLOBALS *****************
int an0, an1, an2, an3, an4, an5, an9, an10, an11,
    state, motor,
    revcount, pivcount, fullpivcount, counter, startcounter,
    waitstate,
    val,
    en, resetflag, lpivflag, rpivflag,
    i,
    statebreak, attackbreak, started;

// **************** PROTOTYPES ******************

/*
 * Function: init_gpio
 * ----------------------------
 *   Initializes all 9 analog channels as input
 *   Sets digital out pins for phase of both motors
 *   Initializes digital in for IR diode
 *   Peripheral Pin select RB5/RB6 to use Output Compare 2/4
 *   Configure PWM mode for OC 2/4
 *
 */
void init_gpio(void);

/*
 * Function: perp_enable
 * ----------------------------
 *   Enable Timer2 (PWM), ADC, Output Compare 2/4
 *
 */
void perp_enable(void);

/*
 * Function: timer_config
 * ----------------------------
 *   Initialize timers and their corresponding interrupts
 *
 *   TIMER1 -> 256 Prescale -> PR of 156 -> Interrupt priority 1
 *   TIMER4 -> 256 Prescale -> PR of 156 -> Interrupt priority 3
 *
 */
void timer_config(void);

/*
 * Function: adc_int_config
 * ----------------------------
 *   Initialize ADC interrupt
 *
 *   Priority level is set to 3, with a sub priority level of 0.
 *
 */
void adc_int_config(void);

/*
 * Function: adc_config_auto_scan
 * ----------------------------
 *   Configure ADC to autoscan analog channel inputs.
 *
 *   adcpins: pins that need to be enabled for channel scan select
 *   numpins: amount of pins to scan before adc interrupt gets triggered
 *
 */
void adc_config_auto_scan(unsigned, unsigned);

/*
 * Function: motor_control
 * ----------------------------
 *   Configure ADC to autoscan analog channel inputs.
 *
 *   leftspeed: PWM speed for the left motors
 *   rightspeed: PWM speed for the right motors
 *   leftphase: Direction of left motors
 *   rightphase: Direction of right motors
 *
 */
void motor_control(int , int , int , int );

/*
 * Function: escape_border
 * ----------------------------
 *   State that decides which way to escape the edge of the doyo.
 *
 *   FWD     -> adjust motors for forward direction
 *   REV_STR -> adjust motors to reverse straight
 *   PIVOT   -> adjust motors so left side reverses and right side stays straight
 *
 */
void escape_border(void);

/*
 * Function: seek
 * ----------------------------
 *   State that seeks the opponent. This is the default state of OMUS
 *
 *   ROT_LEFT     -> adjust left motor to a slower pwm than right
 *   ROT_RIGHT    -> adjust right motor to a slower pwm than left
 *   FWD          -> adjust motors for forward direction
 *   FULL_PIVOT   -> same as pivot, but increase the speed of the motors
 *   PIVOT        -> adjust motors so left side reverses and right side stays straight
 *
 */
void seek(void);


/*
 * Function: ir_recieve
 * ----------------------------
 *   Starts timer3 and initialzes the wait state of the bit. This state will
 *   not exit until the correct bit string pattern is entered by the remote,
 *   which is the power button. Once read, the next state will be the seek state.
 *
 */
void ir_receive(void);


#endif	/* _OMUS5_H */

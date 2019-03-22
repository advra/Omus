#include "omus5.h"

// Initialize all GPIO
void init_gpio(void) {

    // Set analog inputs
    TRISAbits.TRISA0 = 1;       // (2) set RA0 (AN0) as an input RR-G ***
    TRISAbits.TRISA1 = 1;       // (3) set RA1 (AN1) as an input R-IR
    TRISBbits.TRISB0 = 1;       // (4) set RB0 (AN2) as an input B-IR
    TRISBbits.TRISB2 = 1;       // (6) set RB2 (AN4) as an input F-IR
    TRISBbits.TRISB3 = 1;       // (7) set RB3 (AN5) as an input L-IR
    TRISBbits.TRISB15 = 1;      // (26) set RB15(AN9) as an input FR-G ***
    TRISBbits.TRISB14 = 1;      // (25) set RB14 (AN10) as an input FL-G ***
    TRISBbits.TRISB13 = 1;      // (24) set RB13 (AN11) as an input RL-G ***

    // Outputs for Motor Driver Phases (reverse/forward)
    TRISBbits.TRISB7 = 0;       // (16) set RB7 as an output (note RB7 is a digital only pin) R
    TRISBbits.TRISB8 = 0;       // (17) set RB8 as an output (note RB8 is a digital only pin) L

    // Input for IR Diode
    TRISBbits.TRISB4 = 1;      // (11) set RB4 as input for IR diode signal

    // Set OC2 to pin RB5 with peripheral pin select
    // LEFT MOTOR
    RPB5Rbits.RPB5R = 0x0005;

    // Set OC4 to pin RB6 with peripheral pin select
    // RIGHT MOTOR
    RPB6Rbits.RPB6R = 0x0005;

    // Configure standard PWM mode for output compare module 2
    // LEFT MOTOR
    OC2CON = 0x0006;

    // Configure standard PWM mode for output compare module 4
    // RIGHT MOTOR
    OC4CON = 0x0006;
}

// Enable all technologies
void perp_enable(void) {

    T2CONSET = 0x8000;                  // Enable Timer2, prescaler 1:1 (PWM)
    AD1CON1SET = 0x8000;                // Enable ADC
    OC2CONSET = 0x8000;                 // Enable Output Compare Module 2
    OC4CONSET = 0x8000;                 // Enable Output Compare Module 4
}

// Configure the timers and their associated interrupts
void timer_config(void) {
    // Init Timer1 to internal source clock with 1:256 prescaler and a period of
    // 156 (interrupt every 1ms)
    OpenTimer1(T1_ON | T1_SOURCE_INT | T1_PS_1_256, 0x009C);

    // Configure Interrupt timer with a priority of 1
    ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_1);

    // Init Timer4 to internal source clock with 1:256 prescaler and a period of
    // 156 (interrupt every 1ms)
    OpenTimer4(T4_ON | T4_SOURCE_INT | T4_PS_1_256, 0x009C);

    // Configure Interrupt timer with a priority of 3
    ConfigIntTimer4(T4_INT_ON | T4_INT_PRIOR_3);
}

// Configure the ADC interrupt
void adc_int_config(void) {
    // Configure Interrupts
    // In datasheet: TABLE 7-1: INTERRUPT IRQ, VECTOR AND BIT LOCATION

    IEC0CLR = 0x10000000;       // disable ADC int, IEC0<28>
    IFS0CLR = 0x10000000;       // clear ADC int flag, IFS0<28>
    IPC5CLR = 0x1F000000;       // clear ADC priority/subpriority fields IPC5<28:24>
    IPC5SET = 0x0C000000;       // set ADC int priority = 3, IPC5<28:26>
    IPC5SET = 0x00000000;       // set ADC int subpriority = 0, IPC2<1:0>
    IEC0SET = 0x10000000;       // enable ADC int, IEC0<28>
}

// Configure ADC for autoscan of the channel inputs
void adc_config_auto_scan( unsigned adcPINS, unsigned numPins){
    AD1CON1 = 0x0000; // disable ADC

    // AD1CON1<2>, ASAM    : Sampling begins immediately after last conversion completes
    // AD1CON1<7:5>, SSRC  : Internal counter ends sampling and starts conversion (auto convert)
    // AD1CON1<4>, CLRASM  : Stop conversion when firstADC interrupt is generated
    AD1CON1SET = 0x00F4;

    // AD1CON2<10>, CSCNA  : Scan inputs
    AD1CON2 = 0x0400;

    // AD2CON2<5:2>, SMPI  : Interrupt flag set at after numPins completed conversions
    AD1CON2SET = (numPins - 1) << 2;

    // AD1CON3<7:0>, ADCS  : TAD = TPB * 2 * (ADCS<7:0> + 1) = 4 * TPB in this example
    // AD1CON3<12:8>, SAMC : Acquisition time = AD1CON3<12:8> * TAD = 15 * TAD in this example
    AD1CON3 = 0x0f01;

    // AD1CHS is ignored in scan mode
    AD1CHS = 0;

    // select which pins to use for scan mode
    AD1CSSL = adcPINS;
} // END adcConfigureManual()



// Control the direction and speed of the motor
void motor_control(int left, int right, int leftPhase, int rightPhase) {

    // Forward/Reverse
    LATBbits.LATB8 = leftPhase;
    LATBbits.LATB7 = rightPhase;

    // Speed
    OC2RS = left + 1;
    OC4RS = right + 1;
}



// Escape from the border when the white perimeter gets hit
void escape_border(void) {

    switch(motor) {
        case FWD:
            motor_control(SPD_6, SPD_6, FRWD, FRWD);
            break;
        case REV_STR:
            motor_control(SPD_3, SPD_3, REV, REV);
            break;
        case PIVOT_L:
            motor_control(SPD_5, SPD_5, REV, FRWD);
            break;
        case PIVOT_R:
            motor_control(SPD_5, SPD_5, FRWD, REV);
            break;
    }

}

// Constant mode that our robot will be in when its searching for others
void seek(void) {

    switch(motor) {
        case ROT_LEFT:
            // Adjust right motor for faster rotation
            motor_control(SPD_6, SPD_1, FRWD, FRWD);
            break;
        case ROT_RIGHT:
            // Adjust right motor for faster rotation
            motor_control(SPD_1, SPD_6, FRWD, FRWD);
            break;
        case FWD:
            motor_control(SPD_6, SPD_6, FRWD, FRWD);
            break;
        case PIVOT:
            motor_control(SPD_4, SPD_4, REV, FRWD);
            break;
        case FULL_PIV:
            motor_control(SPD_4, SPD_4, REV, FRWD);
            break;
        case ROT_LEFT_F:
            motor_control(SPD_6, SPD_3, FRWD, FRWD);
            break;
        case ROT_RIGHT_F:
            motor_control(SPD_3, SPD_6, FRWD, FRWD);
            break;
        case OFF:
            motor_control(SPD_0, SPD_0, FRWD, FRWD);
            break;
    }
}


// IR Receive function wont start the match until the power button from the
// remote enables the device.
void ir_receive(void) {

    // Init Timer4 to internal source clock with 1:256 prescaler and a period of
    // 16 (interrupt every .1ms)
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_256, 0x0010);

    // Configure Interrupt timer with a priority of 1
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_1);
}

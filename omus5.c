#include "omus5.h"


// Configuration Bits
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
                                    // see figure 8.1 in datasheet for more info
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config ICESEL = ICS_PGx1    // ICE/ICD Comm Channel Select
#pragma config JTAGEN = OFF         // Disable JTAG
#pragma config FSOSCEN = OFF        // Disable Secondary Oscillator
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK

// Pre defined bit string for the power button and array to hold the input
// from the remote.
char *powerstr = "00010000111011111101100000100111";
char bitstr[33] = {0};

int main(void) {

    init_gpio();                        // Initialize neccessary Inputs/Outputs

    INTEnableSystemMultiVectoredInt();  // Multivectored mode for multiple interrupts

    PR2 = (SYSCLK / PWM_FREQ) - 1;      // Set period register for PWM

    adc_config_auto_scan(AINPUTS, 8);   // Configure ADC
    adc_int_config();                   // Configure ADC Interrupts

    started = 0;
    state = WAIT;                       // Init the state of the system

    // This is the main loop for our program which incorporates a state machine
    while( 1){

        switch(state) {
            case WAIT:
                if(started != 1) {
                    ir_receive();
                    started = 1;
                }
                break;
            case SEEK:
                seek();
                break;
            case ESCAPE:
                escape_border();
                break;
            case ATTACK1:
                motor_control(SPD_7_5, SPD_7_5, FRWD, FRWD);
                break;
            case ATTACK2:
                motor_control(SPD_9, SPD_9, FRWD, FRWD);
                break;
        }
    }

    return 1;
}


// TIMER1 Interrupt Handler for the line sensors. They have a higher priority
// then all other sensors. (1ms)
void __ISR(4, ipl1) timer_1_handler(void) {
    mT1ClearIntFlag();



    // Check if we are stuck in a state
    if(motor == FULL_PIV || motor == ROT_LEFT || motor == ROT_RIGHT) {
        statebreak++;
        if(statebreak > 1500) {     // 1.5s check
            statebreak = 0;
            state = ESCAPE;
            motor = REV_STR;
        }
    }
    else {
        statebreak = 0;
    }


    // ******** GROUND SENSORS *******
    // FL-G sensor check
    if(an10 < LINE_THRESH) {
        state = ESCAPE;
        motor = REV_STR;
        lpivflag = 1;
    }

    // FR-G sensor check
    else if(an9 < LINE_THRESH) {
        state = ESCAPE;
        motor = REV_STR;
        rpivflag = 1;
    }

    // Rear Ground Sensors
    if(an11 < LINE_THRESH || an0 < LINE_THRESH) {
        state = SEEK;
        motor = FWD;
    }

    // Only use proximity sensors if we are not escaping the border
    if(state != ESCAPE) {


        // ******** PROXIMITY SENSORS **********
        // Front check. Only attack if both of the front sensors detect something
        if(an4 > 300) {
            state = ATTACK1;

            if(an4 > 700) {
                state = ATTACK2;
            }
        }

        // Left
        if(an5 > 275) {
            state = SEEK;
            motor = ROT_LEFT;
        }

        // RIGHT
        if(an1 > 275) {
            state = SEEK;
            motor = ROT_RIGHT;
        }

        // BACK
        if(an2 > 300) {
            state = SEEK;
            motor = FULL_PIV;
        }
    }
}


// TIMER3 interrupts for wait state (.1ms)
void __ISR(12, ipl1) timer_3_handler(void) {


    // Clear flag immediately after entering interrupt
    mT3ClearIntFlag();

    // Get the value of the digital output at RB9 pin 18
    val = PORTBbits.RB4;

    // Constantly update the the counter that detects the two starts bits
    startcounter++;

    // Since the output of the IR stays high when no buttons are pressed,
    // we detect the the first start bit goes low and make sure we are in
    // the correct state.
    //
    // Zero the counter and since we are checking for time of the next 2 bits
    // Advance the wait state to the next state
    if(waitstate == WAIT_PRE && val == 0) {
        startcounter = 0;
        waitstate = WAIT_0;
    }

    // Check we are in the state that reads low for 9ms (90 counts)
    // The value then goes high after 9ms. Check the read value immediately
    // after and if val does not read high, restart the state machine. Advance
    // to the next state otherwise and restart the next counter.
    else if(waitstate == WAIT_0 && startcounter > 90) {
        if(val == 1){
            waitstate = WAIT_1;
            startcounter = 0;
        } else {
            waitstate = WAIT_PRE;
        }
    }

    // The last start bit is high for 43 seconds. Since we are strictly checking
    // the 90 seconds in the previous state, we must give ourselves some cushion
    // so we dont over read the bit string.
    //
    // The string will read low after 43ms and the 32 bit string will start.
    // We check the state is low and advance to the last state and reset the
    // counter if satisfied. Restart the state machine otherwise.
    else if(waitstate == WAIT_1 && startcounter > 35 ) {
        if(val == 0){
            counter =0;
            waitstate = WAIT_2;
        } else {
            waitstate = WAIT_PRE;
        }
    }

    // This is the state we will be checking the 32 bit string. We have hard
    // coded the value of the power button and check against that value at
    // the end of the 32 bit read. Only go into this state when the value is
    // read high and we have satisfied the start bit check.
    //
    // We only want to update this counter when the value is read high since
    // we are checking the period. A zero will read high for 520us and a 1 will
    // read high for 1.56ms
    //
    // Set an enable flag so when the value is read low, we only store the data
    // in the array once and preserve the check. So when the value goes low,
    // first check that if the value is greater than 12ms. We do 12 to give our
    // selves cushion and not over read the value. If this check is satisfied
    // reset the counter and store a 1 char into the result array and increment
    // the array counter. If the read count is not greater than 12, the reading
    // must be a 0, thus pushing a 0 char to the result array.
    if(waitstate > 2 && val == 1) {
        counter++;
        en = 1;

    } else if(waitstate > 2 && en == 1) {
        en = 0;
        if(counter >= 12 ) {
            counter = 0;
            bitstr[i] = '1';
            i++;
        }
        else {
            counter = 0;
            bitstr[i] = '0';
            i++;
        }
    }

    // Repeat the above state 32 times and then check the read string.
    if(i > 31) {

        bitstr[i] = '\0';
        int k;
        for(k = 0;k < 32;k++) {
            if(powerstr[k] != bitstr[k]) {
                resetflag = 1;
            }
        }


        // If the the result is the correct bit string (power), invert the state
        // of the LED
        if(resetflag != 1) {
            state = SEEK;           // Start the match and seek the opponent
            motor = FWD;            // Set the direction of the motor
            CloseTimer3();          // Disable TIMER3
            perp_enable();          // Enable all the peripherals and start match
            timer_config();         // Init timers and their associated interrupts
        }

        // Reset all values/states
        memset(bitstr, 0, sizeof(bitstr));
        i = 0;
        resetflag = 0;
        waitstate = WAIT_PRE;
    }
}



// TIMER4 interrupts for motor navigation (1ms)
void __ISR(16, ipl3) timer_4_handler(void) {

    mT4ClearIntFlag();

    // Timer to count how long to reverse to and pivot for. We will reverse
    // for 300ms and then pivot for another 300ms.
    if(state == ESCAPE) {

        // Reverse the bot.
        if(motor == REV_STR) {

            revcount++;
            if(revcount > 200) {
                if(lpivflag == 1) {
                    motor = PIVOT_L;
                }
                else {
                    motor = PIVOT_R;
                }
                revcount = 0;
                lpivflag = 0;
                rpivflag = 0;
            }
        }

        // Pivoting after reverse
        else if(motor == PIVOT_L || motor == PIVOT_R) {
            pivcount++;
            if(pivcount > 200) {
                state = SEEK;
                motor = FWD;
                pivcount = 0;
            }
        }
    }
}


// ADC1 Interrupt Service Routine
void __ISR(23, ipl3) adc_handler (void) {

    // Immediately clear the interrupt flag
    mAD1ClearIntFlag();

    // DONE bit must be reset (does not do automatically?)
    AD1CON1bits.DONE = 0;

    // Since AD1CON is configured as a 16 word buffer, the 9 inputs are are
    // stored in the first 9 buffers
    an0 = ADC1BUF0;
    an1 = ADC1BUF1;
    an2 = ADC1BUF2;
    an4 = ADC1BUF3;
    an5 = ADC1BUF4;
    an9 = ADC1BUF5;
    an10 = ADC1BUF6;
    an11 = ADC1BUF7;

    // Automatic sampling bit is automatically cleared once the interrupt flag
    // is set. Must start sampling again once we leave this routine.
    AD1CON1bits.ASAM = 1;           // restart automatic sampling
}

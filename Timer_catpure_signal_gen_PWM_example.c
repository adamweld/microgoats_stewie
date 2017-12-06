/**
 * This is a very small example that shows how to use
 * === OUTPUT COMPARE and INPUT CAPTURE ===
 * The system uses hardware to generate precisely timed
 * pulses, then uses input capture tp compare the capture period
 * to the gereration period for acccuracy
 *
 * There is a command thread to set parametners
 * There is a capture time print-summary thread
 * There is a one second timer tick thread
 * 
 * -- Pin 4 is toggled by the timer2 interrupt
 * -- Pin 14 and 18 are output compare outputs
 * -- Pin 6 is input capture input -- connect this to one of the output compares
 *
 * -- Uart connections explained elsewhere
 * Modified by Bruce Land 
 * Jan 2015
 */

// i/o names 
#include <plib.h>
// set cpu freq to 64 MHz, peripeherial to 32 MHz
#pragma config FNOSC = FRCPLL, POSCMOD = HS, FPLLIDIV = DIV_2, FPLLMUL = MUL_16, FPBDIV = DIV_2, FPLLODIV = DIV_1
#pragma config FWDTEN = OFF
#pragma config FSOSCEN = OFF, JTAGEN = OFF
// frequency we're running at
#define	SYS_FREQ 64000000
// serial stuff
#include <stdio.h>
// protoThreads environment
#include "pt_cornell.h" 

// === thread structures ============================================
// thread control structs

//print lock
static struct pt_sem print_sem ;

// note that UART input and output are threads
static struct pt pt_print, pt_cmd, pt_time, pt_input, pt_output, pt_DMA_output ;

// system 1 second interval tick
int sys_time_seconds ;

//The measured period of the wave
short capture1, last_capture1=0, capture_period, min_period=32000, max_period=0 ;
//The actual period of the wave
int generate_period = 16383 ;
int pwm_on_time = 10 ;
//print state variable
int printing=0 ;

// == Timer 2 ISR =====================================================
// just toggles a pin for timeing strobe
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    // generate a trigger strobe for timing other events
    mPORTBSetBits(BIT_0);
    // change the PWM duty cycle
    if (pwm_on_time++ >= generate_period) pwm_on_time = 0;
    SetDCOC3PWM(pwm_on_time);
    // clear the timer interrupt flag
    mT2ClearIntFlag();
    mPORTBClearBits(BIT_0);
}

// == Capture 1 ISR ====================================================
// check every cpature for consistency
void __ISR(_INPUT_CAPTURE_1_VECTOR, ipl3) C1Handler(void)
{
    //mPORTBSetBits(BIT_0);
    capture1 = mIC1ReadCapture();
    capture_period = capture1 - last_capture1 ;
    // condition on last capture to avoid start up error
    if (capture_period > max_period && last_capture1>0) max_period = capture_period ;
    if (capture_period < min_period && last_capture1>0) min_period = capture_period ;
    last_capture1 = capture1 ;
    //mPORTBClearBits(BIT_0);
    // clear the timer interrupt flag
    mIC1ClearIntFlag();
}

// === Command Thread ======================================================
// The serial interface
static char cmd[16]; 
static int value;

static PT_THREAD (protothread_cmd(struct pt *pt))
{
    PT_BEGIN(pt);
      while(1) {
          
            // send the prompt via DMA to serial
            PT_SEM_WAIT(pt, &print_sem) ;
            sprintf(PT_send_buffer,"cmd>");
            // by spawning a print thread
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
            PT_SEM_SIGNAL(pt, &print_sem) ;
 
          //spawn a thread to handle terminal input
            // the input thread waits for input
            // -- BUT does NOT block other threads
            // string is returned in "PT_term_buffer"
            PT_SPAWN(pt, &pt_input, PT_GetSerialBuffer(&pt_input) );
            // returns when the thead dies
            // in this case, when <enter> is pushed
            // now parse the string
             sscanf(PT_term_buffer, "%s %d", cmd, &value);
         
             if (cmd[0]=='v' ) { printing = 1 ;} // trigger the print thread
             if (cmd[0]=='p' ) {
                 generate_period = value;
                 // update the timer period
                 WritePeriod2(generate_period);
                 // update the pulse start/stop
                 SetPulseOC2(generate_period>>2, generate_period>>1);
                 // clear the capture statistics
                last_capture1=0; min_period=32000; max_period=0 ;
             }
             if (cmd[0]=='d' ) {
                 pwm_on_time = value ;
                 SetDCOC3PWM(pwm_on_time);
             } //        
            // never exit while
      } // END WHILE(1)
  PT_END(pt);
} // thread 3

// === Period print Thread ======================================================
// just blinks 
static PT_THREAD (protothread_print(struct pt *pt))
{
    PT_BEGIN(pt);

      while(1) {

            // allow command thread to stop/start printing
            PT_WAIT_UNTIL(pt, printing);

            PT_SEM_WAIT(pt, &print_sem) ;
            // print the periods
             sprintf(PT_send_buffer,"%d %d %d %d %d \n\rcmd>",
                generate_period, capture_period, min_period, max_period, sys_time_seconds);
            // by spawning a print thread
            PT_SPAWN(pt, &pt_DMA_output, PT_DMA_PutSerialBuffer(&pt_DMA_output) );
            PT_SEM_SIGNAL(pt, &print_sem) ;

            // clear the capture statistics
            last_capture1=0; min_period=32000; max_period=0 ;

            // stop this thread
            printing = 0 ;
            
            // never exit while
      } // END WHILE(1)
  PT_END(pt);
} // thread 4

// === One second Thread ======================================================
// update a 1 second tick counter
static PT_THREAD (protothread_time(struct pt *pt))
{
    PT_BEGIN(pt);

      while(1) {
            // yield time 1 second
            PT_YIELD_TIME_msec(1000) ;
            sys_time_seconds++ ;
            // NEVER exit while
      } // END WHILE(1)

  PT_END(pt);
} // thread 4

// === Main  ======================================================

int main(void)
{
  
  // === Config timer and output compares to make pulses ========
  // set up timer2 to generate the wave period
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period);
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
  mT2ClearIntFlag(); // and clear the interrupt flag

  // set up compare3 for PWM mode
  OpenOC3(OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE , pwm_on_time, pwm_on_time); //
  // OC3 is PPS group 4, map to RPB9 (pin 18)
  PPSOutput(4, RPB9, OC3);
 // mPORTASetPinsDigitalOut(BIT_3);    //Set port as output -- not needed

  // set pulse to go high at 1/4 of the timer period and drop again at 1/2 the timer period
  OpenOC2(OC_ON | OC_TIMER2_SRC | OC_CONTINUE_PULSE, generate_period>>1, generate_period>>2);
  // OC2 is PPS group 2, map to RPB5 (pin 14)
  PPSOutput(2, RPB5, OC2);
 // mPORTBSetPinsDigitalOut(BIT_5); //Set port as output -- not needed

  // === Config timer3 free running ==========================
  // set up timer3 as a souce for input capture
  // and let it overflow for contunuous readings
  OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_1, 0xffff);

  // === set up input capture ================================
  // based on timer3
  OpenCapture1(  IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_ON );
  // turn on the interrupt so that every capture can be recorded
  ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_3 );
  INTClearFlag(INT_IC1);
  // connect PIN 24 to IC1 capture unit
  PPSInput(3, IC1, RPB2);
  mPORTBSetPinsDigitalIn(BIT_2 );    //Set port as input

  // === config the uart, DMA, vref, timer5 ISR ===========
  PT_setup();

  // === setup system wide interrupts  ====================
  INTEnableSystemMultiVectoredInt();
    
  // === set up i/o port pin ===============================
  mPORTBSetPinsDigitalOut(BIT_0 );    //Set port as output

  // === now the threads ===================================
  
  // init the threads
  PT_INIT(&pt_print);
  PT_INIT(&pt_cmd);
  PT_INIT(&pt_time);

  // init the print semaphore
  PT_SEM_INIT(&print_sem, 1); // start with ready to send

  // schedule the threads
  while(1) {
    PT_SCHEDULE(protothread_print(&pt_print));
    PT_SCHEDULE(protothread_cmd(&pt_cmd));
    PT_SCHEDULE(protothread_time(&pt_time));
  }
} // main

/*
 * Padraig Lysandrou PSL58@cornell.edu
 * Control code for 3DOF platform
 */

////////////////////////////////////
#include "config.h"
#include <math.h>
#include <plib.h>
#include <stdio.h>
#include "pt_cornell_1_2_1.h"
#include "tft_master.h"
#include "tft_gfx.h"
#include <stdlib.h>
#include "i2c_helper.h"

#define _SUPPRESS_PLIB_WARNING  // thank the goats
#define SYS_FREQ 40000000       // clock spee d
#define MICROSTEPS 32           // number of microsteps
#define TIMER2_PERIOD 15000      //1250 // 32000 hz for 300 rpm  
#define IMU_READ_PERIOD 1      // in ms
#define COMP_FILTER_G_COEF 0.98
#define COMP_FILTER_A_COEF 0.02
#define STEP_MIN 0
#define STEP_MAX 50 * MICROSTEPS


//GLOBAL VARIABLES
char buffer[60];
const static int steps360 = 200 * MICROSTEPS;
const static float deg2step = 200 * MICROSTEPS / 360;
int timer;
int outA, outB, outC; // motor output target values (steps)
int posA, posB, posC; // motor current position (steps)
float values[6], xAccel, yAccel, zAccel, xGyro, yGyro;
float old1, old2, old3, old4;
static float deg2rad = M_PI / 180.0;
static float rad2deg = 180.0 / M_PI;
static struct pt pt_driver, pt_mpu;
int init_countr = 0;
int setup_countr = 0;

float k;
float b1[3],b2[3],b3[3],p1[3],p2[3],p3[3];
float l1[3], l2[3], l3[3];
float L1_a, L2_a, L3_a;
float bRp0[3], bRp1[3], bRp2[3], T[3];
float z_set = 105 ;
float angle, theta, phi;
int a = 10, b = 10, c = 10;
float phi0 = 25.84;
float d = 50.0;
float e = 70.0;
float z0 = 32.0;
float multy = 1.0;
int attenuate = 1;


// helperfxn: for performing dot products
float dot_product(float v[], float u[]){
    float result = 0.0;
    int i;
    for (i = 0; i < 3; i++){
        result += v[i]*u[i];
    }
    return result;
}


// helper fxn: performing platform --> base 
// direction cosine matrix basis transformation
// notice this is only for phi, theta. We are constrained
// to zero in the psi dimension
void BRP(float theta, float phi){
    bRp0[0] = cos(theta*deg2rad);
    bRp0[1] = sin(theta*deg2rad)*sin(phi*deg2rad);
    bRp0[2] = cos(phi*deg2rad)*sin(theta*deg2rad);
    bRp1[0] = 0;
    bRp1[1] = cos(phi*deg2rad);
    bRp1[2] = -sin(phi*deg2rad);
    bRp2[0] = -sin(theta*deg2rad);
    bRp2[1] = cos(theta*deg2rad)*sin(phi*deg2rad);
    bRp2[2] = cos(phi*deg2rad)*cos(theta*deg2rad);
}

// helper fxn: creating the individual end-effector vectors
// note that we only have control over the z axis
// and theta, phi. therefore hard underactuated contraints
// are built in.
void create_l_vectors(){
    //effectively adding the T vector initially
    BRP(theta, phi);
    T[0] = 0.0; T[1] = 0.0; T[2] = z_set;
    l1[0] = T[0] + (dot_product(bRp0,p1)) - b1[0];
    l1[1] = T[1] + (dot_product(bRp1,p1)) - b1[1];
    l1[2] = T[2] + (dot_product(bRp2,p1)) - b1[2];
    l2[0] = T[0] + (dot_product(bRp0,p2)) - b2[0];
    l2[1] = T[1] + (dot_product(bRp1,p2)) - b2[1];
    l2[2] = T[2] + (dot_product(bRp2,p2)) - b2[2];
    l3[0] = T[0] + (dot_product(bRp0,p3)) - b3[0];
    l3[1] = T[1] + (dot_product(bRp1,p3)) - b3[1];
    l3[2] = T[2] + (dot_product(bRp2,p3)) - b3[2];
}

// helper fxn: limit lengths and convert to angles and then to steps
// first limit the lengths so platform doesn't over actuate
// also bound the output step quantity to zero.
float step_transform(float length){
    if(length > 145.0){length = 145.0;}
    if(length < 32.0){length = 32.0;}
    float intermed = ((length*length)+(d*d)-(e*e))/(2*d*length);
    if(intermed >= 1 ){intermed = 0.99;}
    angle = -rad2deg*acos(intermed) + 90.0 + phi0;
    angle = deg2step * angle;
    if(angle < 0){angle = 0;}
    return angle;
}

// Control thread: reads and filters acceleration values
// dilutes the input angle by half for less jitter
// finds angles via arctan, calculats vectors and steps
// also has loops for initialization
static PT_THREAD(protothread_mpu(struct pt *pt)) {
    PT_BEGIN(pt);
    while (1) {
        timer = PT_GET_TIME();
        readImuValues(values);
        xGyro = values[3];
        yGyro = values[4];
        xAccel = (xAccel*0.95) + (0.05*values[0]);      // get XYZ acceleration
        yAccel = (yAccel*0.95) + (0.05*values[1]);
        zAccel = (zAccel*0.95) + (0.05*values[2]);
        theta = (0.95*theta)+(multy*0.05*atan2f(xAccel, zAccel)*rad2deg); // determine angle and filter
        phi =   (0.95*phi)  +(multy*0.05*atan2f(yAccel, zAccel)*rad2deg);
        create_l_vectors(); // create the end-effector vectors
        L1_a = step_transform(sqrt((l1[0]*l1[0])+(l1[1]*l1[1])+(l1[2]*l1[2]))); //norm and 
        L2_a = step_transform(sqrt((l2[0]*l2[0])+(l2[1]*l2[1])+(l2[2]*l2[2]))); //convert to steps
        L3_a = step_transform(sqrt((l3[0]*l3[0])+(l3[1]*l3[1])+(l3[2]*l3[2])));
        
        // if first interations, calculate the proper height for the setup procedure
        if(init_countr == 0){
            theta = 0.0; phi = 0.0;
            create_l_vectors();
            setup_countr = (int)(step_transform(sqrt((l1[0]*l1[0])+(l1[1]*l1[1])+(l1[2]*l1[2]))));
        }
        
        // setup procedure: hop up to nominal height at zeroed angles
        if(init_countr < (setup_countr)){
            outA = init_countr;
            outB = init_countr;
            outC = init_countr;
            init_countr = init_countr + 10;
        }
        // otherwise just start using the calculated values from accelerometer
        else{
            outA = (int)(L1_a);
            outB = (int)(L2_a);
            outC = (int)(L3_a);
        }
        // sets frequency of this loop
        PT_YIELD_TIME_msec(IMU_READ_PERIOD - (PT_GET_TIME() - timer));
    }
    PT_END(pt);
}


// Stepper ISR
void __ISR  (_TIMER_2_VECTOR, ipl2) Timer2Handler(void) {
    mT2ClearIntFlag();
    //
    if (outA - posA > 0){      // motor behind pos
        mPORTBSetBits(BIT_4);   //set direction 
        posA++;
        mPORTBSetBits(BIT_5);   // step the thing
    } 
    else if (outA - posA < 0) { // motor in front of pos
        mPORTBClearBits(BIT_4); //set direction backwards
        posA--;
        mPORTBSetBits(BIT_5);   // step the thing
    }    
    if (outB - posB > 0) {      // motor behind pos
        mPORTBSetBits(BIT_10);  //set direction 
        posB++;
        mPORTBSetBits(BIT_7);   // step the thing
    }
    else if (outB - posB < 0) { // motor in front of pos
        mPORTBClearBits(BIT_10);//set direction backwards
        posB--;
        mPORTBSetBits(BIT_7);   // step the thing
    }
    if (outC - posC > 0) {      // motor behind pos
        mPORTASetBits(BIT_2);   //set direction 
        posC++;
        mPORTASetBits(BIT_3);   // step the thing
    }
    else if (outC - posC < 0) { // motor in front of pos
        mPORTAClearBits(BIT_2); //set direction backwards
        posC--;
        mPORTASetBits(BIT_3);   // step the thing
    }


    i2c_wait(10);
    mPORTBClearBits(BIT_5);    // clear the step lines
    mPORTBClearBits(BIT_7);
    mPORTAClearBits(BIT_3);
}

// thread for debugging 
// displays useful intermediate calculation values
static PT_THREAD(protothread_driver(struct pt *pt)){
    PT_BEGIN(pt);
    tft_setCursor(0, 0);
    tft_setTextColor(ILI9340_YELLOW);
    tft_setTextSize(2);
    tft_writeString("\'Im a dingus\' - Adam");

    int i = 32;
    while (1) {
        PT_YIELD_TIME_msec(300);
        tft_fillRoundRect(150, 120, 160, 15, 1, ILI9340_BLACK); // x,y,w,h,radius,color
        tft_setCursor(60, 120);
        sprintf(buffer, "Angle X: %.1f", theta);    // theta value from accelerometer
        tft_writeString(buffer);
        tft_fillRoundRect(150, 150, 160, 15, 1, ILI9340_BLACK); // x,y,w,h,radius,color
        tft_setCursor(60, 150);
        sprintf(buffer, "Angle Y: %.1f", phi);     // phi val from accel
        tft_writeString(buffer);
        tft_fillRoundRect(100, 180, 210, 15, 1, ILI9340_BLACK); // x,y,w,h,radius,color
        tft_setCursor(60, 180);
        sprintf(buffer, "OutA: %d", outA);          // step calculations
        tft_writeString(buffer);
        tft_fillRoundRect(100, 210, 210, 15, 1, ILI9340_BLACK); // x,y,w,h,radius,color
        tft_setCursor(60, 210);
        sprintf(buffer, "OutB: %d", outB);
        tft_writeString(buffer);
        tft_fillRoundRect(100, 240, 210, 15, 1, ILI9340_BLACK); // x,y,w,h,radius,color
        tft_setCursor(60, 240);
        sprintf(buffer, "OutC: %d", outC);
        tft_writeString(buffer);
        tft_fillRoundRect(100, 270, 210, 15, 1, ILI9340_BLACK); // x,y,w,h,radius,color
        tft_setCursor(60, 270);
        sprintf(buffer, "IC: %d", init_countr);     // initialization counter value
        tft_writeString(buffer);
    }
    PT_END(pt);
}

// main function
void main(void){
    // SET ALL INITIAL CONDITIONS AND HARD CODED VECTOR VALUES!
    outA = 0;
    outB = 0;
    outC = 0;
    posA = 0;
    posB = 0;
    posC = 0;
    xAccel = 0.0;
    yAccel = 0.0;
    zAccel = 0.0;
    theta = 0.0;
    phi = 0.0;
    k = 100.0/sin(60*deg2rad);
    b1[0] = k;                  // body origin to motor 1 shaft center X position
    b1[1] = 0.0;                // body origin to motor 1 shaft center Y position
    b1[2] = z0;                 // body origin to motor 1 shaft center Z position
    b2[0] = -k*sin(30*deg2rad); // same for motor 2
    b2[1] = k*cos(30*deg2rad);
    b2[2] = z0;
    b3[0] = -k*sin(30*deg2rad); // same for motor 3
    b3[1] = -k*cos(30*deg2rad);
    b3[2] = z0;
    p1[0] = k;                  // platform origin to effector pivot vector X position
    p1[1] = 0.0;                // platform origin to effector pivot vector Y position
    p1[2] = 0.0;                // platform origin to effector pivot vector Z position
    p2[0] = -k*sin(30*deg2rad); // same for second effector pivot point
    p2[1] = k*cos(30*deg2rad);
    p2[2] = 0.0;
    p3[0] = -k*sin(30*deg2rad); // same for third effector pivot point
    p3[1] = -k*cos(30*deg2rad);
    p3[2] = 0.0;
    if(attenuate){multy = 0.5;} // if we want to dilute the input values
    ANSELA = 0;
    ANSELB = 0;

    // === config threads ==========
    // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();

    // SETUP TIMER 2
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, TIMER2_PERIOD);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag

    // === setup system wide interrupts  ========
    INTEnableSystemMultiVectoredInt();

    // open I2C coms
    OpenI2C1(I2C_ON, 48);

    // init the display
    tft_init_hw();
    tft_begin();
    tft_fillScreen(ILI9340_BLACK);
    tft_setRotation(0); // Use tft_setRotation(1) for 320x240

    // seed random color
    srand(1);
    mPORTBSetPinsDigitalOut(BIT_5);
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetPinsDigitalOut(BIT_7);
    mPORTBSetPinsDigitalOut(BIT_10);
    mPORTASetPinsDigitalOut(BIT_2);
    mPORTASetPinsDigitalOut(BIT_3);

    // Take the MPU 6050 out of sleep mode
    char data[] = {0};
    i2c_write(0x6b, data, 1);

    // Set the gyro sensitivity to 131 lsb/(degrees/second))
    i2c_write(0x1b, data, 1);// ended up not needing this

    // Calibrate the gyroscopes (goat must be stable on flat surface)
    calibrateGyros(); // ended up not needing this

    // init the threads
    PT_INIT(&pt_driver);
    PT_INIT(&pt_mpu);
    
    // round-robin scheduler for threads
    while (1) {
        PT_SCHEDULE(protothread_driver(&pt_driver));
        PT_SCHEDULE(protothread_mpu(&pt_mpu));
    }
} // main







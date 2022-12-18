/*  Kelvin Nelson 24/07/2019
 *  
 *  Pulse Width Modulation (PWM) decoding of RC Receiver with failsafe
 *  
 *  This code contains easy to use functions to measure square wave signals on any arduiuno pro mini, nano or uno pins, excluding A6 and A7.
 *  The code is intended to be used with RC receivers, but could also be used in most other PWM measurement applications as a direct replacement for pulseIn(PIN, HIGH). 
 *  (to date it hasn't been tested at a frequency greater than 1khz or on an arduino mega)
 *  
 *  An RC signal pulse can be converted from a pulse width duration (1000-2000uS) at each input pin into an -+100% (-+1.0) output for use in a sketch.
 *  The calibration for this conversion plus a failsafe setting can be set for each channel. (fail safe tolerances 10-330Hz and 500-2500uS). 
 *  
 *  The raw data for each pin can also be extracted, i.e. time of pulse, pulse width, frame length, duty and frequency.
 *  
 *  Set-up is quick, the organisation of this file is as follows:
 *  
 *    - Overview of the code
 *    - List of functions
 *    - How to use, including example sketches
 *    - User defined variables -> specify input pins, transmitter calibration, and failsafe.
 *    - Global variables and functions
 *  
 *  OVERVIEW OF THE CODE:
 *  
 *  The code enables pin change interrupts on the selected pins by setting the appropriate registers.
 *  
 *  A voltage change on any of the selected pins will trigger one of three Interrupt Service Routines depending on which register the pin belongs to.
 *    - ISR(PCINT0_vect), ISR(PCINT1_vect) or ISR(PCINT2_vect)
 *    
 *  Within each ISR the code determines which pin has changed, and makes a note of the time before returning back to the main loop().
 *  
 *  The time intervals between pin changes are used to calculate pulse width and frame length. 
 *  
 *  Flags are set by the ISR to indicate when new pulses are received.
 *  
 *  The Flags are then used to extract and process the data collected by each ISR.
 * 
 *  Although it's not exactly the same, this code follows similar principles to those explained in this video: https://youtu.be/bENjl1KQbvo
 * 
 */
// LIST OF FUNCTIONS:
// OUTPUT TYPE    NAME OF FUNCTION           NOTES

// void           setup_pwmRead()            initialise the PWM measurement using pin change interrupts

// RC RECEIVER DECODING
// boolean        RC_avail()                 returns a HIGH when new RC data is available
// float          RC_decode(channel number)  decodes the selected RC channel into the range +-100%, and applies a failsafe.
// void           print_RCpwm()              Prints the RC channel raw data to serial port (used for calibration).

// GENERIC PWM MEASUREMENTS
// boolean        PWM_read(channel number)   returns a HIGH when a new pulse has been detected on a particular channel. 
//                                           The function saves the pulse data to variables outside the interrupt routines
//                                           and must be called just before using the rest of PWM functions.
// unsigned long  PWM_time()                 returns the time at the start of pulse 
// float          PWM()                      returns the pulse width
// float          PWM_period()               returns the time between pulses
// float          PWM_freq()                 calculates the frequency
// float          PWM_duty()                 calculates the duty

// NOTE: PWM_read(CH) and RC_decode(CH) use the same flags to detect when new data is available, meaning data could be lost if both are used on the same channel at the same time.
// SUGESTION: if you want to use PWM_read(CH) to find the frame rate of an RC channel call it before RC_decode(CH). The output from RC_decode(CH) will then default to the failsafe.

// HOW TO USE, including example sketches

#include <Arduino.h>
#include <Wire.h>

/*
 *  USER DEFINED VARIABLES (MODIFY TO SUIT YOUR APPLICATION)
 */
 
// PWM input pins, any of the following pins can be used: digital 0 - 13 or analog A0 - A5 


const int pwmPIN[]={A8,A9,A10,A11,A12,A13}; // an array to identify the PWM input pins (the array can be any length) 
                                  // first pin is channel 1, second is channel 2...etc

int RC_inputs = 0;                // The number of pins in pwmPIN that are connected to an RC receiver. Addition pins not connected to an RC receiver could be used for any other purpose i.e. detecting the echo pulse on an HC-SR04 ultrasonic distance sensor
                                  // When 0, it will automatically update to the number of pins specified in pwmPIN[] after calling setup_pwmRead().                                                
// Calibration of each RC channel:
 
// The arrays below are used to calibrate each RC channel into the range -1 to +1 so that servo direction, mixing, rates, sub trims...etc can be applied in sketch depending on application. 
// The arrays should be modified in order to calibrate the min, middle and max pulse durations to suit your transmitter (use max rates to get the best resolution). 
// FYI: the function print_PWM() will print the raw pulse width data for all the RC channels to the serial port.
// if the RC_min[], RC_mid[], RC_max[] are empty or have missing data the calibration will default to min 1000us, mid 1500us and max 2000us.

//SANWA 6CH 40MHz with corona RP6D1  
//                THR     RUD     PIT     BAL     SWITCH  SLIDER
int RC_min[6] = { 1000,    1000,   1000,    1000,    1000,   1000};
int RC_mid[6] = { 1500,   1500,   1500,   1500,   1500,   1500};
int RC_max[6] = { 2000,   2000,   2000,   2000,   2000,   2000};

// fail safe positions

float RC_failsafe[] = {0.00, 0.00, 1, 0.00, -0.25, 0.00};
   
// enter a failsafe position (in the range of -+1) for each RC channel in case radio signal is lost
// if the array is the incorrect length for the number of RC channels, the failsafe will default to neutral i.e. 0. 
// The failsafe tolerances are: 10-330Hz & 500-2500us

/*
 *    GLOBAL PWM DECODE VARIABLES
 */

const int num_ch = sizeof(pwmPIN)/sizeof(int);  // calculate the number of input pins (or channels)
volatile int PW[num_ch];                        // an array to store pulsewidth measurements
volatile boolean prev_pinState[num_ch];         // an array used to determine whether a pin has gone low-high or high-low
volatile unsigned long pciTime;                 // the time of the current pin change interrupt
volatile unsigned long pwmTimer[num_ch];        // an array to store the start time of each PWM pulse

volatile boolean pwmFlag[num_ch];               // flag whenever new data is available on each pin
volatile boolean RC_data_rdy;                   // flag when all RC receiver channels have received a new pulse
unsigned long pwmPeriod[num_ch];                 // period, mirco sec, between two pulses on each pin

byte pwmPIN_reg[num_ch];                        // each of the input pins expressed as a position on it's associated port register
byte pwmPIN_port[num_ch];                       // identify which port each input pin belongs to (0 = PORTB, 1 = PORTC, 2 = PORTD)

const int size_RC_min = sizeof(RC_min) / sizeof(int);           // measure the size of the calibration and failsafe arrays
const int size_RC_mid = sizeof(RC_mid) / sizeof(int);
const int size_RC_max = sizeof(RC_max) / sizeof(int);
const int size_RC_failsafe = sizeof(RC_failsafe) / sizeof(float);

// FUNCTION USED TO TURN ON THE INTERRUPTS ON THE RELEVANT PINS
// code from http://playground.arduino.cc/Main/PinChangeInterrupt

void pciSetup(byte pin){
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}

// FUNCTION USED TO FIND THE PIN POSITION ON EACH PORT REGISTER: helps the interrupt service routines, ISR, run faster

void pwmPIN_to_port(){
  for (int i = 0; i < num_ch; i++){

    // determine which port and therefore ISR (PCINT0_vect, PCINT1_vect or PCINT2_vect) each pwmPIN belongs to.
    pwmPIN_port[i] = 2;    // pin belongs to PCINT9_vect (PORT K) removing functionality, constricting to analog A87-A15 on mega


    // covert the pin number (i.e. pin 11 or pin A0) to the pin position in the port register. There is most likely a better way of doing this using a macro...
    // (Reading the pin state directly from the port registers speeds up the code in the ISR)
    
    if(pwmPIN[i] == A8)         pwmPIN_reg[i] = 0b00000001;
    else if(pwmPIN[i] == A9)    pwmPIN_reg[i] = 0b00000010;
    else if(pwmPIN[i] == A10)   pwmPIN_reg[i] = 0b00000100;
    else if(pwmPIN[i] == A11)   pwmPIN_reg[i] = 0b00001000;
    else if(pwmPIN[i] == A12)   pwmPIN_reg[i] = 0b00010000;
    else if(pwmPIN[i] == A13)   pwmPIN_reg[i] = 0b00100000;
    else if(pwmPIN[i] == A14)   pwmPIN_reg[i] = 0b01000000;
    else if(pwmPIN[i] == A15)   pwmPIN_reg[i] = 0b10000000;
    
  }
}

// SETUP OF PIN CHANGE INTERRUPTS

void setup_pwmRead(){
  
  for(int i = 0; i < num_ch; i++){              // run through each input pin
    pciSetup(pwmPIN[i]);                        // enable pinchange interrupt for pin
  }
  pwmPIN_to_port();                             // determines the port for each input pin
                                                // pwmPIN_to_port() also coverts the pin number in pwmPIN[] (i.e. pin 11 or pin A0) to the pin position in the port register (i.e. 0b00000001) for use in the ISR.
  
  if(RC_inputs == 0 || RC_inputs > num_ch) RC_inputs = num_ch;    // define the number of pins connected to an RC receiver.                                          
} 

// INTERRUPT SERVICE ROUTINES (ISR) USED TO READ PWM INPUT

// the PCINT0_vect (B port register) reacts to any changes on pins D8-13.
// the PCINT1_vect (C port register)          ""        ""         A0-A5.
// the PCINT2_vect (D port register)          ""        ""         D0-7.

// port registers are used to speed up if statements in ISR code:
// https://www.arduino.cc/en/Reference/PortManipulation http://tronixstuff.com/2011/10/22/tutorial-arduino-port-manipulation/
// http://harperjiangnew.blogspot.co.uk/2013/05/arduino-port-manipulation-on-mega-2560.html


// READ INTERRUPTS ON PINS D8-D13: ISR routine detects which pin has changed, and returns PWM pulse width, and pulse repetition period.

ISR(PCINT2_vect){                                                 // this function will run if a pin change is detected on portB
  pciTime = micros();                                             // Record the time of the PIN change in microseconds
  
  for (int i = 0; i < num_ch; i++){                               // run through each of the channels
    if (pwmPIN_port[i] == 2){                                     // if the current channel belongs to portB
      
      if(prev_pinState[i] == 0 && PINK & pwmPIN_reg[i]){          // and the pin state has changed from LOW to HIGH (start of pulse)
        prev_pinState[i] = 1;                                     // record pin state
        pwmPeriod[i] = pciTime - pwmTimer[i];                     // calculate the time period, micro sec, between the current and previous pulse
        pwmTimer[i] = pciTime;                                    // record the start time of the current pulse
      }
      else if (prev_pinState[i] == 1 && !(PINK & pwmPIN_reg[i])){ // or the pin state has changed from HIGH to LOW (end of pulse)
        prev_pinState[i] = 0;                                     // record pin state
        PW[i] = pciTime - pwmTimer[i];                            // calculate the duration of the current pulse
        pwmFlag[i] = HIGH;                                        // flag that new data is available
        if(i+1 == RC_inputs) RC_data_rdy = HIGH;                  
      }
    }
  }
}


/*
 *  RC OUTPUT FUNCTIONS
 */

 boolean RC_avail(){
    boolean avail = RC_data_rdy;
    RC_data_rdy = LOW;                          // reset the flag
    return avail;
    }

  float RC_decode(int CH){
  
  if(CH < 1 || CH > RC_inputs) return 0;     // if channel number is out of bounds return zero.
  
  int i = CH - 1;                     

  // determine the pulse width calibration for the RC channel. The default is 1000, 1500 and 2000us.
  
  int Min;
  if(CH <= size_RC_min) Min = RC_min[CH-1]; else Min = 1000;
  
  int Mid;
  if(CH <= size_RC_mid) Mid = RC_mid[CH-1]; else Mid = 1500;
  
  int Max;
  if(CH <= size_RC_max) Max = RC_max[CH-1]; else Max = 2000;

  float CH_output;
      
//  if(FAILSAFE(CH) == HIGH){                         // If the RC channel is outside of failsafe tolerances (10-330hz and 500-2500uS)
//      if(CH > size_RC_failsafe) CH_output = 0;      // and if no failsafe position has been defined, set output to neutral
//      else CH_output = RC_failsafe[i];              // or if defined set the failsafe position 
//  }
//  else{                                             // If the RC signal is valid
CH_output = calibrate(PW[i],Min,Mid,Max);       // calibrate the pulse width to the range -1 to 1.
//  }
  return CH_output;                                 

  // The signal is mapped from a pulsewidth into the range of -1 to +1, using the user defined calibrate() function in this code. 

  // 0 represents neutral or center stick on the transmitter
  // 1 is full displacement of a control input is one direction (i.e full left rudder)
  // -1 is full displacement of the control input in the other direction (i.e. full right rudder)
}

/*
 *  Receiver Calibration
 */

 // NEED TO SPEED UP

float calibrate(float Rx, int Min, int Mid, int Max){
   float calibrated;
   if (Rx >= Mid)
   {
    calibrated = map(Rx, Mid, Max, 0, 1000);  // map from 0% to 100% in one direction
   }
   else if (Rx == 0)
   {
    calibrated = 0;                           // neutral
   }
   else
   {
    calibrated = map(Rx, Min, Mid, -1000, 0); // map from 0% to -100% in the other direction
   }
  return calibrated * 0.001;
}

// Basic Receiver FAIL SAFE
// check for 500-2500us and 10-330Hz (same limits as pololu)

boolean FAILSAFE(int CH){

   int i = CH-1;
   boolean failsafe_flag = LOW;
        
       if(pwmFlag[i] == 1)                             // if a new pulse has been measured.
         {
            pwmFlag[i] = 0;                            // set flag to zero
      
            if(pwmPeriod[i] > 100000)                  // if time between pulses indicates a pulse rate of less than 10Hz   
            {
              failsafe_flag = HIGH;                       
            }
            else if(pwmPeriod[i] < 3000)               // or if time between pulses indicates a pulse rate greater than 330Hz   
            {
              failsafe_flag = HIGH;                             
            }

            if(PW[i] < 500 || PW[i] > 2500)           // if pulswidth is outside of the range 500-2500ms
            {
              failsafe_flag = HIGH;                        
            }   
         }
        else if (micros() - pwmTimer[i] > 100000)     // if there is no new pulswidth measurement within 100ms (10hz)
        {
          failsafe_flag = HIGH;                      
        }

    return failsafe_flag;   
}

/*
 *  Quick print function of Rx channel input
 */

void print_RCpwm(){                             // display the raw RC Channel PWM Inputs
  for (int i = 0; i < RC_inputs; i++){
    //Serial.print(" ch");Serial.print(i+1);
    Serial.print("  ");
    if(PW[i] < 1000) Serial.print(" ");
    Serial.print(PW[i]);
  }
  Serial.println("");
}

void print_decimal2percentage(float dec){
  int pc = dec*100;
  // the number and text will take up 6 charactors i.e ___3%_ or -100%_
  if (pc >= 0) Serial.print(" ");
  if (abs(pc) < 100) Serial.print(" ");
  if (abs(pc) < 10) Serial.print(" ");
  Serial.print(" ");Serial.print(pc);Serial.print("% ");
}

/*
 * GENERIC PWM FUNCTIONS
 */

unsigned long pin_time;
float pin_pwm;
float pin_period;

boolean PWM_read(int CH){
  if(CH < 1 && CH > num_ch) return false;
  int i = CH-1;
  boolean avail = pwmFlag[i];
  if (avail == HIGH){
    pwmFlag[i] = LOW;
    noInterrupts();
    pin_time = pwmTimer[i];
    pin_pwm = PW[i];
    pin_period = pwmPeriod[i];
    interrupts();
  }
  return avail;
}

unsigned long PWM_time(){return pin_time;}
float PWM_period(){return pin_period;}
float PWM(){return pin_pwm;}

float PWM_freq(){
  float freq;
  return freq = 1000000 / pin_period;  // frequency Hz
}

float PWM_duty(){
  float duty;
  duty = pin_pwm/pin_period;
  return duty;
}

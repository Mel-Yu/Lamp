//this version correct as of 4/19/22


#include <SPI.h>
unsigned long now;                        // timing variables to update data at a regular interval                  
unsigned long rc_update;
const int channels = 6;                   // specify the number of receiver channels
float RC_in[channels];     // an array to store the calibrated input from receiver 



#define PWM_SPOOL_ONE 7
#define PWM_SPOOL_TWO 6
#define PWM_SPOOL_THREE 5
#define PWM_SPOOL_FOUR 4
#define PWM_DRIVE_ONE 8
#define PWM_DRIVE_TWO 9
#define PWM_DRIVE_THREE 10
#define PWM_DRIVE_FOUR 11
#define PWM_GRIPPER 3 //added 12.07
#define DIR_SPOOL_ONE 24
#define DIR_SPOOL_TWO 30 //swapped from 26 for E spool dir because pin26 is broken
#define DIR_SPOOL_THREE 28
#define DIR_SPOOL_FOUR 22
#define DIR_DRIVE_ONE 41
#define DIR_DRIVE_TWO 43
#define DIR_DRIVE_THREE 45
#define DIR_DRIVE_FOUR 47
#define DIR_GRIPPER 34 //added 12.07
#define ENC_SPOOL_ONE 48
#define ENC_SPOOL_TWO 44
#define ENC_SPOOL_THREE 42
#define ENC_SPOOL_FOUR 46
#define LIGHT_SWITCH_TOGGLE 53 //added 12.09






float XJoy = 0;
float YJoy = 0;
float deadzone = 0.15;
float NEPower = 0.0;
float NWPower = 0.0;
float SEPower = 0.0;
float SWPower = 0.0;

float lastPos4 = 0;
float newPos4 = 0;
float vel4 =0;
float lastVel4 =0;

unsigned long lastTime;








void setup() {
    SPI.begin();
    setup_pwmRead();                      
    Serial.begin(9600);
    pinMode(PWM_SPOOL_ONE, OUTPUT);
    pinMode(PWM_SPOOL_TWO, OUTPUT);
    pinMode(PWM_SPOOL_THREE, OUTPUT);
    pinMode(PWM_SPOOL_FOUR, OUTPUT);
    pinMode(DIR_SPOOL_ONE, OUTPUT);
    pinMode(DIR_SPOOL_TWO, OUTPUT);
    pinMode(DIR_SPOOL_THREE, OUTPUT);
    pinMode(DIR_SPOOL_FOUR, OUTPUT);
    pinMode(PWM_SPOOL_ONE, OUTPUT);
    pinMode(PWM_DRIVE_TWO, OUTPUT);
    pinMode(PWM_DRIVE_THREE, OUTPUT);
    pinMode(PWM_DRIVE_FOUR, OUTPUT);
    pinMode(DIR_DRIVE_ONE, OUTPUT);
    pinMode(DIR_DRIVE_TWO, OUTPUT);
    pinMode(DIR_DRIVE_THREE, OUTPUT);
    pinMode(DIR_DRIVE_FOUR, OUTPUT);
    pinMode(PWM_GRIPPER, OUTPUT);
    pinMode(DIR_GRIPPER, OUTPUT);
    pinMode(LIGHT_SWITCH_TOGGLE, OUTPUT);
    
    stopAll();
    
    initLS7366(ENC_SPOOL_ONE);
    initLS7366(ENC_SPOOL_TWO);
    initLS7366(ENC_SPOOL_THREE);
    initLS7366(ENC_SPOOL_FOUR);
    clearEncoder(ENC_SPOOL_ONE);
    clearEncoder(ENC_SPOOL_TWO);
    clearEncoder(ENC_SPOOL_THREE);
    clearEncoder(ENC_SPOOL_FOUR);
    
}

void loop() {  
    /*Serial.print("1: ");
    Serial.print(readEncoder(ENC_SPOOL_ONE));
    Serial.print(" 2: ");
    Serial.print(readEncoder(ENC_SPOOL_TWO));
    Serial.print(" 3: ");
    Serial.print(readEncoder(ENC_SPOOL_THREE));
    Serial.print(" 4: ");
    Serial.println(readEncoder(ENC_SPOOL_FOUR));
    */
    now = millis();
    
    if(RC_avail() || now - rc_update > 25){   // if RC data is available or 25ms has passed since last update (adjust to be equal or greater than the frame rate of receiver)
      
      rc_update = now;                           
      
     //print_RCpwm();                        // uncommment to print raw data from receiver to serial
      
      for (int i = 0; i<channels; i++){       // run through each RC channel
        int CH = i+1;
        RC_in[i] = RC_decode(CH);             // decode receiver channel and apply failsafe
        Serial.print(" channel ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(RC_in[i]);// uncomment to print calibrated receiver input (+-100%) to serial       
      }
     Serial.println();
     //setMotorPower(PWM_SPOOL_ONE,DIR_SPOOL_ONE, RC_in[0]);//WEST
     //setMotorPower(PWM_SPOOL_TWO,DIR_SPOOL_TWO, -1 * RC_in[0]);//EAST
     //setMotorPower(PWM_SPOOL_THREE,DIR_SPOOL_THREE, -1 * RC_in[1]); //SOUTH
     //setMotorPower(PWM_SPOOL_FOUR,DIR_SPOOL_FOUR, RC_in[1]); //NORTH
     //Serial.println(RC_in[1]);
     /*
     if(RC_in[4]>0.5){
     if((abs(RC_in[0])>deadzone) || (abs(RC_in[1])>deadzone) ){
        drive(RC_in[1],RC_in[0]);
     } 
     else{
      stopAll();
     }
     }
     */
     

      
        if(RC_in[5] > 0.5){
          Serial.println("SPOOL CONTROL");
          if((abs(RC_in[0]))>deadzone){
          setMotorPower(PWM_SPOOL_ONE,DIR_SPOOL_ONE,RC_in[0]/5);
          }
          else{
            setMotorPower(PWM_SPOOL_ONE, DIR_SPOOL_ONE, 0);
          }
          if((abs(RC_in[1]))>deadzone){
            setMotorPower(PWM_SPOOL_TWO, DIR_SPOOL_TWO, RC_in[1]/5);
          }
          else{
            setMotorPower(PWM_SPOOL_TWO, DIR_SPOOL_TWO, 0);
          }
          if((abs(RC_in[2]))>deadzone){
          setMotorPower(PWM_SPOOL_THREE,DIR_SPOOL_THREE,RC_in[2]/5);
          }
          else{
            setMotorPower(PWM_SPOOL_THREE, DIR_SPOOL_THREE, 0);
          }
          if((abs(RC_in[3]))>deadzone){
          setMotorPower(PWM_SPOOL_FOUR, DIR_SPOOL_FOUR, RC_in[3]/5);
          }
          else{
            setMotorPower(PWM_SPOOL_FOUR, DIR_SPOOL_FOUR, 0);
          }
        }
        else if(RC_in[5] <-0.5){
          Serial.println("WHEEL & GRIPPER CONTROL");
          if ((abs(RC_in[0])>deadzone) || (abs(RC_in[1])>deadzone)){
            Serial.println("WHEEL DRIVING");
            setMotorPower(PWM_GRIPPER, DIR_GRIPPER, 0);
            drive(RC_in[0],RC_in[1]);
          }
          else if ((abs(RC_in[3]))>deadzone){
            Serial.println("GRIPPER ACTIVATED");
            drive(0,0);
//            if (RC_in[3] > deadzone){
//              analogWrite(PWM_GRIPPER, 255);
//            }
//            else if (RC_in[3] < -deadzone){
//              analogWrite(PWM_GRIPPER, -255);
//            }
            setMotorPower(PWM_GRIPPER, DIR_GRIPPER, RC_in[3]);
            
            //analogWrite(PWM_GRIPPER, 255);
          }
          else{
            setMotorPower(PWM_GRIPPER, DIR_GRIPPER, 0);
            drive(0,0);
          }
        }
        else if((abs(RC_in[4]))>0.5){
          if(RC_in[4] > 0.5){
            Serial.println("LIGHTS ON");
            digitalWrite(LIGHT_SWITCH_TOGGLE, HIGH);
          }
          else if(RC_in[4] < -0.5){
            Serial.println("LIGHTS OFF");
            digitalWrite(LIGHT_SWITCH_TOGGLE, LOW);
          }
        }
        else{
          stopAll(); 
        }

        
    }
        
      
       /*if((abs(RC_in[0])>deadzone) || (abs(RC_in[1])>deadzone) ){
        if(RC_in[5] >0.1){
          setMotorPower(PWM_SPOOL_ONE,DIR_SPOOL_ONE,RC_in[0]);
          setMotorPower(PWM_SPOOL_TWO, DIR_SPOOL_TWO, RC_in[1]);

        }
        else if(RC_in[5]<0.1){
          Serial.println("ELSE");
          setMotorPower(PWM_SPOOL_THREE,DIR_SPOOL_THREE,RC_in[0]);
          setMotorPower(PWM_SPOOL_FOUR, DIR_SPOOL_FOUR, RC_in[1]);
        }
        else{
          stopAll();
        }
        
     }
     
      else{
          stopAll();
      }
     
     }
     */
     
     

     

     
}

    



    
   
   





  

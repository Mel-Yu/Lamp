void setMotorPower(int pwmPin,int dirPin, float motorPower){
  if(motorPower < 0){
    digitalWrite(dirPin, LOW);
  }
  else{
    digitalWrite(dirPin, HIGH);
  }

  analogWrite(pwmPin, (int)255*constrain(abs(motorPower),0,1));
  
}
void stopAll(){
    stop(PWM_SPOOL_ONE);
    stop(PWM_SPOOL_TWO);
    stop(PWM_SPOOL_THREE);
    stop(PWM_SPOOL_FOUR);
    stop(PWM_DRIVE_ONE);
    stop(PWM_DRIVE_TWO);
    stop(PWM_DRIVE_THREE);
    stop(PWM_DRIVE_FOUR);
    stop(PWM_GRIPPER);
}
void stop(int pwmPin){
  analogWrite(pwmPin, 0);
}

void drive(float x, float y){ //given a desired X&Y motion, set motor powers accordingly

   float NEPower = y-x;
   float NWPower = -x-y;
   float SEPower = x+y;
   float SWPower = x-y;
  
  
  setMotorPower(PWM_DRIVE_ONE,DIR_DRIVE_ONE, SWPower/3);
  setMotorPower(PWM_DRIVE_TWO, DIR_DRIVE_TWO, NWPower/3);
  setMotorPower(PWM_DRIVE_THREE,DIR_DRIVE_THREE, NEPower/3);
  setMotorPower(PWM_DRIVE_FOUR,DIR_DRIVE_FOUR, SEPower/3);
  
}

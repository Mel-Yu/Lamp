#include <SoftwareSerial.h>

#include <Servo.h>

Servo myservo1;// create servo object to control a servo
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

SoftwareSerial BT(8,7);
String BT_input;
int s=30;

void setup() {
  Serial.begin(38400);
  BT.begin(38400);
  myservo1.attach(11);  //front right servo
  myservo2.attach(10);  //front left servo
  myservo3.attach(9);  //back right servo
  myservo4.attach(6);  //back left servo
  myservo5.attach(5); //bucket servo
}

void loop() {
  
  if (BT.available()){
    BT_input=BT.readString();
    Serial.println(BT_input);
    if (BT_input=="1")
    {
      forward();
    }
    else if (BT_input=="2")
    {
      backward();                           
    }
    else if (BT_input=="3")
    {
      turn_cw();
    }
    else if (BT_input=="4")
    {
      turn_ccw();
    }
    else if (BT_input=="5")
    {
      bucket();
    }
    else
    {
      pause();
    }
  }
}

void backward(){
  myservo1.write(90+(s-15));
  myservo2.write(90-(s-15));
  myservo3.write(90+(s-15));
  myservo4.write(90-(s-15));
}

void forward(){
  myservo1.write(90-s);
  myservo2.write(95+s);
  myservo3.write(90-s);
  myservo4.write(95+s);
}

void turn_ccw(){
  myservo1.write(130-s); 
  myservo2.write(130-s); 
  myservo3.write(130-s); 
  myservo4.write(130-s); 
}

void turn_cw(){
  myservo1.write(130+s);
  myservo2.write(130+s);
  myservo3.write(130+s);
  myservo4.write(130+s);
}

void bucket(){
  myservo5.write(45);
  myservo5.write(90);
  myservo5.write(135);
  myservo5.write(180);
  delay(2000);
  myservo5.write(135);
  myservo5.write(90);
  myservo5.write(45);
  myservo5.write(0);
  delay(2000);
}
void pause(){
  myservo1.write(90);
  myservo2.write(90);
  myservo3.write(90);
  myservo4.write(90);
}

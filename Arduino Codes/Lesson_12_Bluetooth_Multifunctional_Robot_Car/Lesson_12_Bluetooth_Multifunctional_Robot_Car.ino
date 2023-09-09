#include <Wire.h>              //including libraries of I2C
#include <IRremote.h>          //including libraries of remote control
#define RECV_PIN  12        //pin 12 of IR remoter control receiver
#include <Servo.h>
IRrecv irrecv(RECV_PIN);      //defining pin 12 of IR remoter control
Servo myservo;
decode_results res;
decode_results results;         //cache of decode of IR remoter control
#define IR_Go       0x00FF18E7 //going forward
#define IR_Back     0x00FF4AB5  //going backward
#define IR_Left     0x00FF10EF//turning left
#define IR_Right    0x00FF5AA5  //turning right
#define IR_Stop     0x00FF38C7  //stop 
//////////////////////////////////////////////////
#define SensorLeft    9   //sensor left pin of line tracking module 
#define SensorMiddle  10   //sensor middle pin of line tracking module
#define SensorRight   11  //sensor right pin of line tracking module
unsigned char SL;        //state of left sensor of line tracking module
unsigned char SM;        //state of middle sensor of line tracking module
unsigned char SR;        //state of right sensor of line tracking module
volatile int DL;
volatile int DM;
volatile int DR;
int inputPin=A0;  // ultrasonic module   ECHO to A0
int outputPin=A1;  // ultrasonic module  TRIG to A1
unsigned char bluetooth_data;       
unsigned long Key;
unsigned long Key1;
#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board
int flag=0;
int Car_state=0;             //the working state of car
int myangle;                //defining variable of angle
int pulsewidth;              //defining variable of pulse width
unsigned char DuoJiao=90;    //initialized angle of motor at 90°
void Sensor_IO_Config()     //IO initialized function of three line tracking , all setting at input
{
  pinMode(SensorLeft,INPUT);
  pinMode(SensorMiddle,INPUT);
  pinMode(SensorRight,INPUT);
  pinMode(inputPin, INPUT);      //starting receiving IR remote control signal
  pinMode(outputPin, OUTPUT);    //IO of ultrasonic module
}

void Sensor_Scan(void) //function of reading-in signal of line tracking module 
{
  SL = digitalRead(SensorLeft);
  SM = digitalRead(SensorMiddle);
  SR = digitalRead(SensorRight);
}

void M_Control_IO_config(void)
{
  pinMode(pinLB,OUTPUT); // /pin 2
  pinMode(pinLF,OUTPUT); // pin 4
  pinMode(pinRB,OUTPUT); // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(Lpwm_pin,OUTPUT);  // pin 5 (PWM) 
  pinMode(Rpwm_pin,OUTPUT);  // pin6(PWM)   
}
void Set_Speed(unsigned char pwm) //function of setting speed
{
  analogWrite(Lpwm_pin,pwm);
  analogWrite(Rpwm_pin,pwm);
}
void advance()    //  going forward
    {
     digitalWrite(pinRB,LOW);  // making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);  // making motor move towards left rear
     digitalWrite(pinLF,HIGH); 
     Car_state = 1; 
       
    }
void turnR()        //turning right(dual wheel)
    {
     digitalWrite(pinRB,LOW);  //making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,LOW);  //making motor move towards left front
     Car_state = 4;
    
    }
void turnL()         //turning left(dual wheel)
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,LOW );   //making motor move towards right front
     digitalWrite(pinLB,LOW);   //making motor move towards left rear
     digitalWrite(pinLF,HIGH);
     Car_state = 3;
     
    }    
void stopp()        //stop
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,HIGH);
     Car_state = 5;
    
    }
void back()         //back up
    {
     digitalWrite(pinRB,HIGH);  //making motor move towards right rear     
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,HIGH);  //making motor move towards left rear
     digitalWrite(pinLF,LOW);
     Car_state = 2;
         
    }
         
void Follow()
{
  flag = 0;
  while (flag == 0) 
  {
    DM=checkdistance();  //Get the current distance
    Serial.println(DM);
    analogWrite(Rpwm_pin,180);
    analogWrite(Lpwm_pin,180);
    if(DM<=10){
      Serial.println("stop");
      stopp();
      delay(300);
      DM = checkdistance();
      delay(500);
    }
    else { //There are no obstacles ahead, the car is moving forward
      Serial.println("forward");
      advance();
   }
    if (Serial.available())
      {
        bluetooth_data = Serial.read();
        if (bluetooth_data == 'S') {
          flag = 1;
        }
      }
  }
}

void Line_Tracking(void) //function of line tracking 
{
  flag = 0;
  while (flag == 0) 
  {
 Sensor_Scan();
 if (SM == HIGH)// middle sensor in black area
{
if (SL == LOW & SR == HIGH) // black on left, white on right, turn left
{
turnR();
Set_Speed(200);
}
else if (SR == LOW & SL == HIGH) // white on left, black on right, turn right
{
turnL();
Set_Speed(200);
}
else // white on both sides, going forward
{
advance();
Set_Speed(180);
}
}
else // middle sensor on white area
{
if (SL== LOW & SR == HIGH)// black on left, white on right, turn left
{
turnR();
Set_Speed(200);
}
else if (SR == LOW & SL == HIGH) // white on left, black on right, turn right
{
turnL();
Set_Speed(200);
}
else // all white, stop
{
back();
Set_Speed(150);
delay(100);
stopp() ;
Set_Speed(0);
}
}
  if (Serial.available())
   {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {
        flag = 1;
      }
   }

}


}








float checkdistance() {
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  float distance = pulseIn(A0, HIGH) / 58.00;
  delay(10);
  return distance;
}


void Detect_obstacle_distance() {
  myservo.write(160);
  for (int i = 0; i < 3; i = i + 1) {
    DL = checkdistance();
    delay(100);
  }
  myservo.write(20);
  for (int i = 0; i<3; i = i + 1) {
    DR = checkdistance();
    delay(100);
  }
}


void Ultrasonic_Obstacle_Avoidance()
{flag = 0;
  while (flag == 0) 
  {
  DM = checkdistance();
  if (DM < 30) {
    stopp();
    Set_Speed(0);
    delay(1000);
    Detect_obstacle_distance();
    if (DL < 50 || DR < 50) {
      if (DL > DR) {
        myservo.write(90);
        turnL();
        Set_Speed(200);
        delay(200);
        advance();
        Set_Speed(200);

      } else {
        myservo.write(90);
        turnR();
        Set_Speed(200);
        delay(200);
        advance();
        Set_Speed(200);

      }

    } else {
      if (random(1, 10) > 5) {
        myservo.write(90);
        turnL();
        Set_Speed(200);
        delay(200);
        advance();
        Set_Speed(200);

      } else {
        myservo.write(90);
        turnR();
        Set_Speed(200);
        delay(200);
        advance();
        Set_Speed(200);

      }

    }

  } else {
    advance();
    Set_Speed(130);

  }

  if (Serial.available())
     {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {
        flag = 1;

      }
     }
 }
}






void Infrared_Remote_Control(void)   //remote control，when pressing“#”，it quitting from the mode
{flag = 0;
  while (flag == 0) 
  {
   if(irrecv.decode(&results))  //to judge whether serial port receive data
    {
     Key = results.value;
    switch(Key)
     {
       case IR_Go:advance();Set_Speed(200);   //UP
       break;
       case IR_Back: back();Set_Speed(200);   //back
       break;
       case IR_Left:turnL();Set_Speed(200);   //Left    
       break;
       case IR_Right:turnR();Set_Speed(200); //Righ
       break;
       case IR_Stop:stopp();Set_Speed(0);   //stop
       break;
       default: 
       break;      
     } 
     irrecv.resume(); // Receive the next value
    }
    if (Serial.available())
     {
      bluetooth_data = Serial.read();
      if (bluetooth_data == 'S') {
        flag = 1;

      }
     }
     
  }
}




void setup() 
{ 
   myservo.attach(A2);
   M_Control_IO_config();     //motor controlling the initialization of IO
   Set_Speed(200);  //setting initialized speed
  
   Sensor_IO_Config();            //initializing IO of line tracking module 
   irrecv.enableIRIn();           //starting receiving IR remote control signal
   Serial.begin(9600);            //initialized serial port , using Bluetooth as serial port, setting baud 
   myservo.write(DuoJiao);
   stopp();                       //stop
   delay(1000);
   DL = 0;
   DM = 0;
   DR = 0;
} 
void loop() 
{  
  if (Serial.available())
  {
    bluetooth_data = Serial.read();
    Serial.println(bluetooth_data);
  }
  switch (bluetooth_data) {
    case 'U':
    advance();
    Set_Speed(200);
    break;
   case 'D':
    back();
    Set_Speed(200);
    break;
   case 'L':
    turnL();
    Set_Speed(200);
    break;
   case 'R':
    turnR();
    Set_Speed(200);
    break;
   case 'S':
    stopp();
    Set_Speed(0);
    break;
   case 'T':
    stopp();
    Line_Tracking();
    break;
    case 'O':
    stopp();
    Ultrasonic_Obstacle_Avoidance();
    break;
    case 'I':
    stopp();
    Infrared_Remote_Control();
    break;
    case 'G':
    stopp();
    Follow();
    break;
    default: 
    break;
  }
}

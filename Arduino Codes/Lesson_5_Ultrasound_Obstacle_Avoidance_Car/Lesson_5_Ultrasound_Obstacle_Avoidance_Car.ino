#include <Servo.h>
#define ENA  5             //pin of controlling speed---- ENA of motor driver board
#define ENB  6            //pin of controlling speed---- ENB of motor driver board
int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board
Servo myservo;
volatile int DL;
volatile int DM;
volatile int DR;
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
  for (int i = 0; i < 3; i = i + 1) {
    DR = checkdistance();
    delay(100);
  }
}
void setup(){
  Serial.begin(9600);
  myservo.attach(A2);// attaches the servo on A2 to the servo object
  pinMode(A1, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(pinLB,OUTPUT);  // pin 2
  pinMode(pinLF,OUTPUT);  // pin 4
  pinMode(pinRB,OUTPUT);  // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(ENA,OUTPUT);    // pin 5(PWM) 
  pinMode(ENB,OUTPUT);    // pin 6(PWM) 
  DL = 0;
  DM = 0;
  DR = 0;
  myservo.write(90); //Initialize the servo angle to 90 degrees
}

void loop(){
  DM = checkdistance();
  if (DM < 30) {
    stopp();
    delay(1000);
    Detect_obstacle_distance();
    if (DL < 30 || DR < 30) {
      if (DL > DR) {
          myservo.write(90);
          turnL();
          Set_Speed(200);
          delay(200);
          advance();
          Set_Speed(200);
        } else{
          myservo.write(90);
          turnR();
          Set_Speed(200);
          delay(200);
          advance();
          Set_Speed(200);
        }
      }
     if(DL < 5 || DR < 5){
        myservo.write(90);
        back();
        Set_Speed(200);
        delay(200);
        stopp();
     }
    } 
  else {
    myservo.write(90);
    advance();
    Set_Speed(200);
  }
}


void Set_Speed(unsigned char pwm) //function of setting speed
{
  analogWrite(ENA,pwm);
  analogWrite(ENB,pwm);
}
void advance()    //  going forward
    {
     digitalWrite(pinRB,LOW);  // making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,LOW);  // making motor move towards left rear
     digitalWrite(pinLF,HIGH); 
   
    }
void turnR()        //turning right(dual wheel)
    {
     digitalWrite(pinRB,HIGH);
     digitalWrite(pinRF,LOW );   //making motor move towards right front
     digitalWrite(pinLB,LOW);   //making motor move towards left rear
     digitalWrite(pinLF,HIGH);
    }
void turnL()         //turning left(dual wheel)
    {
     digitalWrite(pinRB,LOW);  //making motor move towards right rear
     digitalWrite(pinRF,HIGH);
     digitalWrite(pinLB,HIGH);
     digitalWrite(pinLF,LOW);  //making motor move towards left front
    }    
void stopp()        //stop
    {
     digitalWrite(pinRB,LOW);
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,LOW);
     digitalWrite(pinLF,LOW);
    
    }
void back()         //back up
    {
     digitalWrite(pinRB,HIGH);  //making motor move towards right rear     
     digitalWrite(pinRF,LOW);
     digitalWrite(pinLB,HIGH);  //making motor move towards left rear
     digitalWrite(pinLF,LOW);
    }

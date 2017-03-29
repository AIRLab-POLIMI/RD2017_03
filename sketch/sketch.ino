#include<Servo.h>
#define GREENLEDPIN 2
#define FORWARD HIGH
#define BACKWARD LOW
#define SERVOPIN 6
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define LEFT 0
#define RIGHT 1
#define ALIMENTATION 4
#define NORMALSPEED 175
#define DELAY45DEGREES 220
Servo oneservo;

byte pos;
byte angle=0;
byte pin_ain1 = 9; //Direction
byte pin_ain2 = 8; //Direction
byte pin_bin1 = 12; //Direction
byte pin_bin2 = 11; //Direction

byte pin_pwma = 3; //Speed
byte pin_pwmb = 5;
byte pin_stby = 10;


void setup() {
  pinMode(GREENLEDPIN, OUTPUT);
  pinMode(pin_pwma, OUTPUT);
  pinMode(pin_ain1, OUTPUT);
  pinMode(pin_ain2, OUTPUT);
  pinMode(pin_pwmb,OUTPUT);
  pinMode(pin_bin1, OUTPUT);
  pinMode(pin_bin2, OUTPUT);
  pinMode(ALIMENTATION,OUTPUT);
  digitalWrite(ALIMENTATION,HIGH);
  
  
  Serial.begin(9600);
}
int i;
bool flag=0;
void loop() {
  if (flag==0) 
  {
    delay(3000);
    flag=1;
  }
  for(i=0;i<4;i++)
  {
    move_forward(NORMALSPEED);
    delay(200);
    turn(NORMALSPEED,LEFT);
    delay(DELAY45DEGREES);
    move_servo();
  }
  for(i=0;i<4;i++)
  {
    move_forward(NORMALSPEED);
    delay(200);
    turn(NORMALSPEED,RIGHT);
    delay(DELAY45DEGREES); 
  }
}
void up_servo()
{
  
  oneservo.attach(SERVOPIN);
  oneservo.write(180);
  delay(100);
  for(i=180;i>130;i--)
  {
    oneservo.write(i);
    delay(10);
  }
}
void down_servo()
{
  oneservo.write(130);
  delay(100);
  for(i=130;i<180;i++)
  {
    oneservo.write(i);
    delay(10);
  }
  oneservo.detach();
}
void move_servo()
{
  up_servo();
  delay(300);
  down_servo();
  delay(300);
}

void turn(byte speed, bool dir)
{
  if(dir==LEFT) turn_left(speed);
  else turn_right(speed);
}
void try_all_movement()
{
 move_forward(100);
 delay(1000);
 move_backward(100);
 delay(1000);
 move_random(180);
 delay(2000);
 spin(100,CLOCKWISE);
 delay(1000);
 spin(100,COUNTER_CLOCKWISE);
 delay(1000);
 turn_left(100);
 delay(1000);
 turn_right(100);
 delay(1000);
 stop();
 delay(300);

}

void move_forward(byte speed)
{
    digitalWrite(pin_stby, HIGH);
    //set direction motor 1
    digitalWrite(pin_ain1, FORWARD);
    digitalWrite(pin_ain2, !FORWARD);
    //set direction motor 2
    digitalWrite(pin_bin1, FORWARD);
    digitalWrite(pin_bin2, !FORWARD);
    //set speed motor 1  
    analogWrite(pin_pwma, speed);
    //set speed motor 2
    analogWrite(pin_pwmb, speed); 
}
void move_backward(byte speed)
{
  digitalWrite(pin_stby, HIGH);
  //set direction motor 1
  digitalWrite(pin_ain1, BACKWARD);
  digitalWrite(pin_ain2, !BACKWARD);
  //set direction motor 2
  digitalWrite(pin_bin1, BACKWARD);
  digitalWrite(pin_bin2, !BACKWARD);
  //set speed motor 1  
  analogWrite(pin_pwma, speed);
  //set speed motor 2
  analogWrite(pin_pwmb, speed); 
}

void move_random(byte max_speed)
{
  boolean dir_motor1,dir_motor2;
  dir_motor1 = random(0,1);
  dir_motor2 = random(0,1);
  digitalWrite(pin_stby, HIGH);
  //set direction motor 1
  digitalWrite(pin_ain1, dir_motor1);
  digitalWrite(pin_ain2, !dir_motor1);
  //set direction motor 2
  digitalWrite(pin_bin1, dir_motor2);
  digitalWrite(pin_bin2, !dir_motor2);
  //set speed motor 1  
  analogWrite(pin_pwma, random(0,max_speed));
  //set speed motor 2
  analogWrite(pin_pwmb, random(0,max_speed)); 

}

void spin(byte speed, bool direction)
{
  digitalWrite(pin_stby, HIGH);
  //set direction motor 1
  digitalWrite(pin_ain1, direction);
  digitalWrite(pin_ain2, !direction);
  //set direction motor 2
  digitalWrite(pin_bin1, !direction);
  digitalWrite(pin_bin2, direction);
  //set speed motor 1  
  analogWrite(pin_pwma, speed);
  //set speed motor 2
  analogWrite(pin_pwmb, speed); 
}

void turn_left(byte speed)
{
  digitalWrite(pin_stby, HIGH);
  //set direction motor 1
  digitalWrite(pin_ain1, 1);
  digitalWrite(pin_ain2, 0);
  //set direction motor 2
  //digitalWrite(pin_bin1, !direction);
 // digitalWrite(pin_bin2, direction);
  //set speed motor 1  
  analogWrite(pin_pwma, speed);
  //set speed motor 2
  analogWrite(pin_pwmb, 0); 
}
void turn_right(byte speed)
{
  digitalWrite(pin_stby, HIGH);
  //set direction motor 1
  //digitalWrite(pin_ain1, 1);
  //digitalWrite(pin_ain2, 0);
  //set direction motor 2
  digitalWrite(pin_bin1, 1);
  digitalWrite(pin_bin2, 0);
  //set speed motor 1  
  analogWrite(pin_pwma, 0);
  //set speed motor 2
  analogWrite(pin_pwmb, speed); 
}
void stop()
{
  analogWrite(pin_pwma,0);
  analogWrite(pin_pwmb,0);
  digitalWrite(pin_stby, LOW);
}



#include<Servo.h>
#define GREENLEDPIN 2
#define FORWARD HIGH
#define BACKWARD LOW
#define SERVOPIN 6
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define LEDPIN 2
#define DISTANCEPIN 4
#define DELAYOBSTACLE 20
#define NORMALSPEED 60
#define LINESENSORPIN A0
#define LINETHRESHOLD 100
#define WHITE 0
#define BLACK 1
#define SLOWSPEED 50
#define FLAGNORMAL 0
#define FLAGCLOCKWISE 1
#define FLAGCOUNTER_CLOCKWISE 2
Servo oneservo;

byte pos;
byte angle=0;
byte pin_ain1 = 9; //Direction
byte pin_ain2 = 8; //Direction
byte pin_bin1 = 11; //Direction
byte pin_bin2 = 12; //Direction

byte pin_pwma = 3; //Speed
byte pin_pwmb = 5;
byte pin_stby = 13;

//


void setup() {
  // put your setup code here, to run once:
  pinMode(LINESENSORPIN,INPUT);
  pinMode(LEDPIN,OUTPUT);
  pinMode(DISTANCEPIN,INPUT);
  pinMode(GREENLEDPIN, OUTPUT);
  pinMode(pin_pwma, OUTPUT);
  pinMode(pin_ain1, OUTPUT);
  pinMode(pin_ain2, OUTPUT);
  pinMode(pin_pwmb,OUTPUT);
  pinMode(pin_bin1, OUTPUT);
  pinMode(pin_bin2, OUTPUT);
  oneservo.attach(SERVOPIN);
  digitalWrite(LEDPIN,HIGH);
  
  Serial.begin(9600);
}
int delay_value=0;
/*---------------------------------------------------------
 * LOOP
 ----------------------------------------------------------*/
bool dir;
int flag = 0;
unsigned long timer=0,timer2;
void loop() {
  if(obstacle()==false && colour_line() == WHITE)
  {
    timer2=millis();
    if(timer2 - timer > 4000) {
      flag=FLAGNORMAL;
      timer = timer2;
    }
    move_forward(NORMALSPEED);
  }
  else if (obstacle() == true )
  {
    switch (flag)
    {
      case FLAGNORMAL:  dir=random(CLOCKWISE,2);
                        flag= 1 + dir;
              break;
      case FLAGCLOCKWISE: dir=CLOCKWISE;
              break;
      case FLAGCOUNTER_CLOCKWISE: dir=COUNTER_CLOCKWISE;
              break;
    }
    
    while(obstacle()==true)
    {
      spin(SLOWSPEED,dir);
      delay(160);
    }
    delay(425);
  }
  else if ( colour_line() == BLACK)
  { switch (flag)
    {
      case FLAGNORMAL: dir=random(CLOCKWISE,2);
                       flag= 1 + dir;
                       break;
                       
      case FLAGCLOCKWISE: dir=CLOCKWISE;
                          break;
      case FLAGCOUNTER_CLOCKWISE: dir=COUNTER_CLOCKWISE;
                                  break;
    }
    while(colour_line()== BLACK)
    {
     spin(SLOWSPEED,dir);
      //delay(40);
    }
  }
  if(flag==0)
  
  Serial.println("normal");
  else if (flag==1)
  
  Serial.println("clockwise");
  else 
  Serial.println("counter_clockwise");
  //delay(450);
}

bool colour_line()
{
  int val=analogRead(LINESENSORPIN);
  //Serial.println(val);
  if(val>LINETHRESHOLD)
  {
    return BLACK;
    
  }
  else return WHITE;
}



bool obstacle()
{
  bool distance;
  distance = digitalRead(DISTANCEPIN);
  //delay(DELAYOBSTACLE);
  return !distance;
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
  digitalWrite(pin_stby, LOW);
}



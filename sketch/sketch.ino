//TODO LIST:
//1. Test the servo motor
//2. Let the color to be parametric

#include<Servo.h>

//pins
#define LED_PIN 2
#define SERVO_PIN 6
#define DISTANCE_PIN 4
#define LINESENSOR_PIN A0
#define AIN1_PIN 9
#define AIN2_PIN 8
#define BIN1_PIN 11
#define BIN2_PIN 12
#define PWMA_PIN 3
#define PWMB_PIN 5
#define STBY_PIN 13

//type of straigth movements
#define FORWARD HIGH
#define BACKWARD LOW

//type of spin movements
#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1

//speed
#define LOW_SPEED 50
#define HIGH_SPEED 70
#define MEDIUM_SPEED (LOW_SPEED+HIGH_SPEED)/2

//colour sensing
#define LINETHRESHOLD 100 //it is the minimum value returned from the reflectance sensor when the floor is black
#define WHITE 0
#define BLACK 1

//movement behaviour
#define FLAGNORMAL 0
#define FLAGCLOCKWISE 1
#define FLAGCOUNTER_CLOCKWISE 2

//numbers of loops that each movement takes
#define CYCLENUMBER_FORWARD 10
#define CYCLENUMBER_TURN 20
#define CYCLENUMBER_SPIN 5
#define CYCLENUMBER_BACKWARD 5

enum TypeOfMovement {
  FORWARD_MOVEMENT,
  TURN_MOVEMENT,
  SPIN_MOVEMENT,
  BACKWARD_MOVEMENT
};

Servo servo;


void setup() {
  // put your setup code here, to run once:
  pinMode(LINESENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DISTANCE_PIN, INPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(9600);
}


void loop() {

  bool dir;
  byte flag = 0;
  TypeOfMovement movement_type; //enum
  unsigned long timer = 0, timer2;
  byte i;
  byte speeds[2];
  boolean dir_spin;
  byte cyclenumber, movement_probability;

  movement_probability = random(0, 100);
  switch (movement_probability)
  {
    case 0 ... 29:
      cyclenumber = CYCLENUMBER_FORWARD;
      movement_type = FORWARD_MOVEMENT; //variable took from the enum
      break;
    case 30 ... 79:
      cyclenumber = CYCLENUMBER_TURN;
      movement_type = TURN_MOVEMENT; //variable took from the enum
      break;
    case 80 ... 96:
      cyclenumber = CYCLENUMBER_SPIN;
      movement_type = SPIN_MOVEMENT; //variable took from the enum
      break;
    case 97 ... 100:
      cyclenumber = CYCLENUMBER_BACKWARD;
      movement_type = BACKWARD_MOVEMENT; //variable took from the enum
      break;
  }
  //set speeds
  for (i = 0; i <= 1; i++)
  {
    speeds[i] = random(0, 100);
    if (speeds[i] < 10) {
      speeds[i] = 0;
    }
    else if (speeds[i] > 10 && speeds[i] < 40) {
      speeds[i] = LOW_SPEED;
    }
    else if (speeds[i] > 40 && speeds[i] < 70) {
      speeds[i] = MEDIUM_SPEED;
    }
    else {
      speeds[i] = HIGH_SPEED;
    }
  }
  //if both speeds are equal to zero the crab will stop! Thus the speeds are forced to be high
  if (speeds[0] == 0 && speeds[1] == 0) {
    speeds[0] = HIGH_SPEED;
    speeds[1] = HIGH_SPEED;
    Serial.print("speeds[0]: ");
    Serial.println(speeds[0]);
    Serial.print("speeds[1]: ");
    Serial.println(speeds[1]);
  }

  //set direction
  dir_spin = boolean(random(0, 2));

  for (i = 0; i <= cyclenumber; i++) {
    if (i==0) timer = millis();
/*
    if (obstacle() == false && colour_line() == WHITE)
    {
      timer2 = millis();
      if (timer2 - timer > 4000) {
        flag = FLAGNORMAL;
        timer = timer2;
      }

      movement (speeds[0], speeds[1], dir_spin, movement_type);
    }
    else */
    timer2=millis();
    if (obstacle() == true )
    {
      //turn ON the servo motor
      //servo.attach(SERVO_PIN);
        //Pull up the servo motor
        //servo.write(90);

      switch (flag)
      {
        case FLAGNORMAL:
          dir = random(CLOCKWISE, 2);
          flag = 1 + dir;
          break;

        case FLAGCLOCKWISE:
          dir = CLOCKWISE;
          break;

        case FLAGCOUNTER_CLOCKWISE: dir = COUNTER_CLOCKWISE;
          break;
      }

      while (obstacle() == true)
      {

        spin(LOW_SPEED, dir);
        delay(160);
      }

      //turn OFF the servo motor
      //servo.detach();

      delay(425);
    }
    else if ( colour_line() == BLACK)
    { while(millis()<(timer2+1000)){
      //turn ON the servo motor
      //servo.attach(SERVO_PIN);

      switch (flag)
      {
        case FLAGNORMAL:
          dir = random(CLOCKWISE, 2);
          flag = 1 + dir;
          break;

        case FLAGCLOCKWISE:
          dir = CLOCKWISE;
          break;
        case FLAGCOUNTER_CLOCKWISE:
          dir = COUNTER_CLOCKWISE;
          break;
      }
      while (colour_line() == BLACK)
      {
        //servo.write(90);
        spin(LOW_SPEED, dir);
      }
      move_forward(LOW_SPEED);
      //turn OFF the servo motor
      //servo.detach();
    }
    spin(LOW_SPEED, dir);
    delay(500);
    }
    movement (speeds[0], speeds[1], dir_spin, movement_type);
  }
}

bool colour_line()
{
  byte val = analogRead(LINESENSOR_PIN);
  if (val > LINETHRESHOLD)
  {
    return BLACK;
  }
  else return WHITE;
}

bool obstacle()
{
  bool distance;
  distance = digitalRead(DISTANCE_PIN);
  return !distance;
}

void movement(byte speed1, byte speed2, bool direction, TypeOfMovement type)    //TypeOfMovement from enum
{
  switch (type) {

    case (FORWARD_MOVEMENT):
      move_forward (speed1);
      break;

    case (TURN_MOVEMENT):
      turn (speed1, speed2);
      break;

    case (SPIN_MOVEMENT):
      spin (speed1, direction);
      break;

    case (BACKWARD_MOVEMENT):
      move_backward (speed1);
      break;
  }
}

void move_forward(byte speed)  //motor A is the right one, motor B is the left one
{
  digitalWrite(STBY_PIN, HIGH);
  //set direction motor 1
  digitalWrite(AIN1_PIN, FORWARD);
  digitalWrite(AIN2_PIN, !FORWARD);
  //set direction motor 2
  digitalWrite(BIN1_PIN, FORWARD);
  digitalWrite(BIN2_PIN, !FORWARD);
  //set speed motor 1
  analogWrite(PWMA_PIN, speed);
  //set speed motor 2
  analogWrite(PWMB_PIN, speed);
}

void turn(byte speed1, byte speed2) //speed1 related to the right motor(A), speed1 related to the left motor(B)
{
  digitalWrite(STBY_PIN, HIGH);
  //set direction motor 1
  digitalWrite(AIN1_PIN, 1);
  digitalWrite(AIN2_PIN, 0);
  //set direction motor 2
  digitalWrite(BIN1_PIN, 1);
  digitalWrite(BIN2_PIN, 0);
  //set speed motor 1
  analogWrite(PWMA_PIN, speed1);
  //set speed motor 2
  analogWrite(PWMB_PIN, speed2);
}

void spin(byte speed, bool direction)  //motor A is the right one, motor B is the left one
{
  digitalWrite(STBY_PIN, HIGH);
  //set direction motor 1
  digitalWrite(AIN1_PIN, direction);
  digitalWrite(AIN2_PIN, !direction);
  //set direction motor 2
  digitalWrite(BIN1_PIN, !direction);
  digitalWrite(BIN2_PIN, direction);
  //set speed motor 1
  analogWrite(PWMA_PIN, speed);
  //set speed motor 2
  analogWrite(PWMB_PIN, speed);
}

void move_backward(byte speed)  //motor A is the right one, motor B is the left one
{
  digitalWrite(STBY_PIN, HIGH);
  //set direction motor 1
  digitalWrite(AIN1_PIN, BACKWARD);
  digitalWrite(AIN2_PIN, !BACKWARD);
  //set direction motor 2
  digitalWrite(BIN1_PIN, BACKWARD);
  digitalWrite(BIN2_PIN, !BACKWARD);
  //set speed motor 1
  analogWrite(PWMA_PIN, speed);
  //set speed motor 2
  analogWrite(PWMB_PIN, speed);
}

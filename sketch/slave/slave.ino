//#include <Servo.h>
#include <IRremote.h>

//pins
#define LED_PIN 2
#define SERVO_PIN 10
#define DISTANCE_PIN 4
#define LINESENSOR_PIN A1
#define AIN1_PIN 9
#define AIN2_PIN 8
#define BIN1_PIN 11
#define BIN2_PIN 12
#define PWMA_PIN 6
#define PWMB_PIN 5
#define STBY_PIN 13
#define IR_PIN 3

//speed
#define LOW_SPEED 40
#define HIGH_SPEED 60
#define MEDIUM_SPEED (LOW_SPEED+HIGH_SPEED)/2

//colour sensing
#define LINETHRESHOLD 100 //it is the minimum value returned from the reflectance sensor when the floor is black

//numbers of loops that each movement takes
#define FORWARD_INTERVAL 1000           //time spent going forward
#define TURN_INTERVAL 1000              //time spent turning
#define SPIN_INTERVAL 500               //time spent spinning
#define BACKWARD_INTERVAL 10            //time spent going backward
#define OBSTACLE_INTERVAL 700           //time spent to spin in order to avoid obstacles
#define BLACK_LINE_SPIN_INTERVAL 2000   //time spent to spin in order to avoid the black line

//IR communication
#define IR_SENSE_INTERVAL 10000           //time frequence used to sense the infrareds
#define PEOPLE_PACKET 0xE0E0906F        //packet received everytime a person approaches
#define NO_PEOPLE_PACKET 0xE0E0906A        //packet received everytime a person goes away

//line following
#define LINE_FOLLOWING_SET_POINT 500
#define KP 0.28

enum state_type {
  MENU_REACHING,
  MENU_WALKING,
  LINE_REACHING,
  LINE_FOLLOWING
};

enum color_type {
  WHITE,
  BLACK
};

enum movement_type {
  STRAIGHT,
  TURN,
  SPIN
};

enum spin_movement_type {
  COUNTER_CLOCKWISE,
  CLOCKWISE
};

enum straight_movement_type {
  BACKWARD,
  FORWARD
};

//Servo servo;

//***********************
// INFRARED SENSOR - start
IRrecv ir_recv(IR_PIN);
decode_results results;
// INFRARED SENSOR - end
//***********************


unsigned long ir_sensing_timer;
unsigned long now;
unsigned long obstacle_timer;
unsigned long region_timer;
unsigned long movement_timer;

unsigned long movement_interval;
unsigned long region_interval;


bool need_to_start_the_ostacle_timer;

byte movement_probability;
movement_type movement; //enum
byte speeds[2];
spin_movement_type spin_direction;
straight_movement_type straight_direction;

state_type state;

bool change_behavior;

//Error used in the line following
float error_value;

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


  ir_recv.enableIRIn();


  movement_timer = millis();
  ir_sensing_timer = movement_timer;


  set_random_values();

  //servo.attach(SERVO_PIN);

}

void loop() {
  now = millis();
  /*
    //*********************************************
    // IR SENSING - start
    //*********************************************
    if (now - ir_sensing_timer > IR_SENSE_INTERVAL) {
      Serial.println("Sense:");
      ir_sensing_timer = now;
      if (ir_recv.decode(&results)) {
        if (results.value == PEOPLE_PACKET) {
          //People want to see the menu
          if (state != LINE_FOLLOWING)
            state = LINE_REACHING;
          else
            state = LINE_FOLLOWING;
          Serial.println("There's someone!");
        }
        else if (results.value == NO_PEOPLE_PACKET) {
          //No one wants to see the menu
          if (state != MENU_WALKING)
            state = MENU_REACHING;
          else
            state = MENU_WALKING;
          Serial.println("No one...");
        }
        else {
          Serial.print("Unexpected package received: ");
          Serial.println(results.value, HEX);
        }

        //The resume allow the sensor to sense another time
        ir_recv.resume();
      }
    }
    //*********************************************
    // IR SENSING - end
    //*********************************************

  */
  //SIMULATION OF PEOPLE (IR_SENSOR)
  if (now - ir_sensing_timer > IR_SENSE_INTERVAL) {
    ir_sensing_timer = now;
    if (change_behavior) {
      
      if (state != MENU_WALKING)
        state = MENU_REACHING;
      else
        state = MENU_WALKING;

      change_behavior = false;
    }
    else {
      
      if (state != LINE_FOLLOWING)
        state = LINE_REACHING;
      else
        state = LINE_FOLLOWING;

      change_behavior = true;
    }
  }

  //Just for testing
  //state = MENU_REACHING;


  //the state of the robot must be modified only in this switch
  switch (state) {

    case MENU_REACHING:
      Serial.println("MENU_REACHING");
      if (reach_the_menu())
        state = MENU_WALKING;

      break;

    case MENU_WALKING:
      Serial.println("MENU_WALKING");
      menu_walking();
      break;

    case LINE_REACHING:
      Serial.println("LINE_REACHING");
      if (reach_the_line())
        state = LINE_FOLLOWING;
      break;

    case LINE_FOLLOWING:
      Serial.println("LINE_FOLLOWING");
      follow_the_line();
      break;
  }

  move(speeds[0], speeds[1], straight_direction, spin_direction, movement);
  //servo.write(90);
  //servo.write(180);
}

//It returns true if the the robot has moved for a specific amount of time (i.e. interval).
bool need_to_change_movement() {
  bool need_to_change;

  switch (movement) {
    case STRAIGHT:
      need_to_change = (now - movement_timer > FORWARD_INTERVAL);
      break;

    case TURN:
      need_to_change = (now - movement_timer > TURN_INTERVAL);
      break;

    case SPIN:
      need_to_change = (now - movement_timer > SPIN_INTERVAL);
      break;
  }

  return need_to_change;
}

//It sets the movement type (straight, spin, turn) and the relative parameters.
void set_random_values() {
  movement_probability = random(0, 100);
  switch (movement_probability)
  {
    case 0 ... 29:
      movement_interval = FORWARD_INTERVAL;
      movement = STRAIGHT; //variable took from the enum
      straight_direction = FORWARD;
      break;
    case 30 ... 100:
      movement_interval = TURN_INTERVAL;
      movement = TURN; //variable took from the enum
      break;
  }

  //set the two speeds
  for (int i = 0; i <= 1; i++)
  {
    speeds[i] = random(0, 100);
    if (speeds[i] < LOW_SPEED) {
      speeds[i] = LOW_SPEED;
    }
    else if (speeds[i] > LOW_SPEED && speeds[i] < HIGH_SPEED) {
      speeds[i] = MEDIUM_SPEED;
    }
    else {
      speeds[i] = HIGH_SPEED;
    }
  }

  //set direction
  spin_direction = random(0, 2);
}


//It moves the robot according to the parameters.
void move(byte speed1, byte speed2, straight_movement_type straight_direction, spin_movement_type spin_direction, movement_type type)
{
  switch (type) {

    case STRAIGHT:
      move_straight (speed1, straight_direction);
      break;

    case TURN:
      turn (speed1, speed2);
      break;

    case SPIN:
      spin (speed1, spin_direction);
      break;
  }
}

//It moves the robot in order to let it go straight (forward/backward according to the parameter).
//The robot starts moving after this method till you stop it.
//Motor A is the right one, motor B is the left one.
void move_straight(byte speed, bool direction)
{
  digitalWrite(STBY_PIN, HIGH);
  //set direction motor 1
  digitalWrite(AIN1_PIN, direction);
  digitalWrite(AIN2_PIN, !direction);
  //set direction motor 2
  digitalWrite(BIN1_PIN, direction);
  digitalWrite(BIN2_PIN, !direction);
  //set speed motor 1
  analogWrite(PWMA_PIN, speed);
  //set speed motor 2
  analogWrite(PWMB_PIN, speed);
}

//It moves the robot in order to let it turn (clockwise/counter-clockwise according to the parameters).
//The robot starts moving after this method till you stop it.
void turn(byte right_speed, byte left_speed)
{
  digitalWrite(STBY_PIN, HIGH);
  //set direction motor 1
  digitalWrite(AIN1_PIN, 1);
  digitalWrite(AIN2_PIN, 0);
  //set direction motor 2
  digitalWrite(BIN1_PIN, 1);
  digitalWrite(BIN2_PIN, 0);
  //set speed motor 1
  analogWrite(PWMA_PIN, right_speed);
  //set speed motor 2
  analogWrite(PWMB_PIN, left_speed);
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

bool obstacle()
{
  //The sensor returns a 1 if an obstacle is between 2cm and 10cm, 0 otherwise.
  return !(digitalRead(DISTANCE_PIN));
}

color_type get_colour()
{
  byte val = analogRead(LINESENSOR_PIN);
  if (val > LINETHRESHOLD)
  {
    return BLACK;
  }
  else return WHITE;
}

void show_movement_values() {
  switch (movement) {
    case STRAIGHT:
      if (straight_direction == FORWARD)
        Serial.println("FORWARDING");
      else
        Serial.println("BACKWARDING");
      Serial.print("speed: ");
      Serial.println(speeds[0]);
      break;

    case TURN:
      Serial.println("TURNING");
      Serial.print("speed1: ");
      Serial.println(speeds[0]);
      Serial.print("speed2: ");
      Serial.println(speeds[1]);
      break;

    case SPIN:
      if (spin_direction == CLOCKWISE)
        Serial.println("SPINNING CLOCKWISE");
      else
        Serial.println("SPINNING COUNTER-CLOCKWISE");
      Serial.print("speed: ");
      Serial.println(speeds[0]);
      break;
  }
}

//It returns true if the robot has reached the menu, otherwise false
bool reach_the_menu() {
  bool menu_reached;

  if (get_colour() == BLACK) {
    menu_reached = false;

    //set spin movement
    movement = SPIN;
    spin_direction = COUNTER_CLOCKWISE;
    speeds[0] = MEDIUM_SPEED;
    speeds[1] = MEDIUM_SPEED;

  }
  else {
    menu_reached = true;
    Serial.println("Menu reached!");
  }
  return menu_reached;
}

//It returns true if the robot has reached the black line, otherwise false
bool reach_the_line() {
  bool line_reached;

  if (get_colour() == WHITE) {
    line_reached = false;

    //set forward movement
    movement = STRAIGHT;
    straight_direction = FORWARD;
    speeds[0] = HIGH_SPEED;
    speeds[1] = HIGH_SPEED;

  }
  else {
    line_reached = true;
    Serial.println("Line reached!");
  }
  return line_reached;
}

void follow_the_line() {
  cal_pid();
  pid_turn();
}

//It calculates position[set point] depending on KP
void cal_pid() {
  error_value = KP * (analogRead(LINESENSOR_PIN) - LINE_FOLLOWING_SET_POINT);
}

//It computes the error to be corrected and sets the motors speeds
void pid_turn() {
  if (error_value < -1) {
    error_value = -150;
  }
  if (error_value > 0) {
    error_value = 150;
  }

  // If error_value is less than -1 calculate right turn speed values, otherwise calculate left turn values
  if (error_value < -1) {
    speeds[0] = error_value;
    speeds[1] = 0;
  }
  else {
    speeds[0] = 0;
    speeds[1] = (-1) * error_value;
  }
  movement = TURN;
}

void menu_walking() {
  if (need_to_change_movement()) {
    movement_timer = now;
    set_random_values();
    show_movement_values();
  }

  //**********************************************************************************************************************************************
  //It checks if there is an OBSTACLE in front of it. If it occours, it lets the robot spinning 90 degrees clockwise/counter-clockwise (randomly).
  //**********************************************************************************************************************************************
  if (obstacle()) {
    if (need_to_start_the_ostacle_timer) {
      Serial.println("Obstacle!!!!");
      //it is the first time I have encountered an obstacle, I have to start the timer
      obstacle_timer = now;
      spin_direction = random(0, 2);
      need_to_start_the_ostacle_timer = false;
    }
  }
  if (now - obstacle_timer < OBSTACLE_INTERVAL) {
    //obstacle avoiding...
    movement = SPIN;
  }
  else
    //obstacle avoided, I have to reset the timer
    need_to_start_the_ostacle_timer  = true;
  //**********************************************************************************************************************************************


  //**********************************************************************************************************************************************
  //It checks if there is a BLACK LINE under the robot. If it occours, it lets the robot spinning 90 degrees clockwise/counter-clockwise (randomly).
  //**********************************************************************************************************************************************
  if (now - region_timer < BLACK_LINE_SPIN_INTERVAL)
    //region avoiding...
    movement = SPIN;
  else if (get_colour() == BLACK) {
    Serial.println("Region to avoid!!!!");
    //it is the first time I have encountered the colour to be avoided, I have to start the timer
    region_timer = now;
    spin_direction = random(0, 2);
  }
  //**********************************************************************************************************************************************

}

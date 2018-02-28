#include <Servo.h>
#include <IRremote.h>

//pins
#define LED_PIN 2
#define SERVO_PIN 10
#define DISTANCE_PIN 4
#define LINESENSOR_PIN A3
#define AIN1_PIN 9
#define AIN2_PIN 8
#define BIN1_PIN 11
#define BIN2_PIN 12
#define PWMA_PIN 6
#define PWMB_PIN 5
#define STBY_PIN 13
#define IR_PIN 3

//speed
#define LOW_SPEED 60 //(blue)
#define HIGH_SPEED 70 //(blue)
//#define LOW_SPEED 50 //(blank)
//#define HIGH_SPEED 70 //(blank)
#define EXTREME_SPEED 100
#define MEDIUM_SPEED (LOW_SPEED+HIGH_SPEED)/2

//colour sensing
#define LINETHRESHOLD 500
//it is the minimum value returned from the reflectance sensor when the floor is black

//numbers of loops that each movement takes
#define FORWARD_INTERVAL 500           //time spent going forward
#define TURN_INTERVAL 500              //time spent turning
#define SPIN_INTERVAL 2000              //time spent spinning//(blue)
//#define SPIN_INTERVAL 500              //time spent spinning//(blank)
#define BACKWARD_INTERVAL 10            //time spent going backward
#define OBSTACLE_INTERVAL 800           //time spent to spin in order to avoid obstacles (blue)
//#define OBSTACLE_INTERVAL 800           //time spent to spin in order to avoid obstacles (blank)
#define BLACK_LINE_SPIN_INTERVAL 150    //time spent to spin in order to avoid the black line (blue)
//#define BLACK_LINE_SPIN_INTERVAL 200    //time spent to spin in order to avoid the black line (blank)
#define POPDOWN_INTERVAL 300            //time spent to popdown in the shell 
#define HIDING_INTERVAL 2000             //time spent hiding in the shell
#define POPUP_INTERVAL 3000              //time spent to popup the shell

//IR communication
#define IR_SENSE_INTERVAL 200           //time frequence used to sense the infrareds
#define PEOPLE_PACKET 0xE0E0906F        //packet received everytime a person approaches
#define NO_PEOPLE_PACKET 0xE0E0906A        //packet received everytime a person goes away
#define PEOPLE_PRESENCE_TIME 10000

//spin sensing
#define SPIN_SENSE_INTERVAL 6000           //time frequence used to sense spinning
//line following
#define LINE_FOLLOWING_SET_POINT 500

servo pos
#define POS_MIN 45//(blue)
#define POS_MAX 60//(blue)
//#define POS_MIN 25//(blank)
//#define POS_MAX 40//(blank)
bool line_crossed;
byte steps_on_menu;

enum state_type {
  MENU_WALKING,
  MENU_REACHING,
  SCARING,
  LINE_REACHING,
  LINE_FOLLOWING
};

enum scaring_state_type {
  POPDOWN,
  HIDING,
  POPUP
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

enum steering_type {
  TO_THE_LEFT,
  TO_THE_RIGHT
};

enum spin_movement_type {
  COUNTER_CLOCKWISE,
  CLOCKWISE
};

enum straight_movement_type {
  STRAIGHT_BACKWARD,
  STRAIGHT_FORWARD
};

enum turn_movement_type {
  TURN_BACKWARD,
  TURN_FORWARD
};

Servo servo;

//***********************
// INFRARED SENSOR - start
IRrecv ir_recv(IR_PIN);
decode_results results;
// INFRARED SENSOR - end
//***********************


unsigned long ir_sensing_timer;
unsigned long spin_sensing_timer;
unsigned long ir_people_presence_timer;
unsigned long now;
unsigned long obstacle_timer;
unsigned long region_timer;
unsigned long movement_timer;
unsigned long popdown_timer;
unsigned long hiding_timer;
unsigned long popup_timer;

unsigned long movement_interval;
unsigned long region_interval;

bool need_to_start_the_ostacle_timer;
bool need_to_spin;
byte movement_probability;
movement_type movement;
movement_type chosen_movement;
steering_type type_of_steering;
float speeds[2];
byte servo_pos;
spin_movement_type spin_direction, chosen_spin_direction;
turn_movement_type turn_direction;
straight_movement_type straight_direction;

byte coin;

state_type state;
state_type previous_state;
scaring_state_type scaring_state;

bool there_is_someone;

//Error used in the line following
float error_value;


//menu reaching variables
byte steps_beyond_the_line;
byte steps_on_the_line;
color_type colour;
bool start_to_count;

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
  spin_sensing_timer = movement_timer;


  set_random_values();

  servo.attach(SERVO_PIN);

  for (int i = POS_MIN; i <= POS_MAX; i += 1) {
    // in steps of 1 degree
    servo_pos = i;
    servo.write(servo_pos);
    delay(30);
  }

}

void loop() {
  now = millis();

  //*********************************************
  // IR SENSING - start
  //*********************************************
  if (now - ir_sensing_timer > IR_SENSE_INTERVAL) {
    Serial.println("Sense:");
    ir_sensing_timer = now;

    if (ir_recv.decode(&results)) {
      Serial.println("PACKET:");
      Serial.println(HEX, results.value);
      if (results.value == PEOPLE_PACKET) {
        ir_people_presence_timer = millis();
        if (state ==  MENU_WALKING) {
          previous_state = MENU_WALKING;
          state = LINE_REACHING;
        }
        Serial.println("There's someone!");
      }
      else if (results.value == NO_PEOPLE_PACKET) {
        //No one wants to see the menu
        ir_people_presence_timer = 0;
        Serial.println("No one...");
        if (state == LINE_FOLLOWING) {
          previous_state = LINE_FOLLOWING;
          state = MENU_REACHING;
        }
      }
    }
    else {
      //No one wants to see the menu
      if (millis() -  ir_people_presence_timer > PEOPLE_PRESENCE_TIME)
      {
        ir_people_presence_timer = 0;
        Serial.println("No one...");
        if (state == LINE_FOLLOWING) {
          previous_state = LINE_FOLLOWING;
          state = MENU_REACHING;
        }
      }
    }
    //The resume allow the sensor to sense another time
    ir_recv.resume();
  }
  //*********************************************
  // IR SENSING - end
  //*********************************************

  /*
    //SIMULATION OF PEOPLE (IR_SENSOR)
    if (now - ir_sensing_timer > IR_SENSE_INTERVAL) {
    ir_sensing_timer = now;
    if (there_is_someone) {
      there_is_someone = false;
      Serial.println("HE'S GONE AWAY...!");
    }
    else {
      there_is_someone = true;
      Serial.println("THERE'S SOMEONE!");
    }
    }
    if (there_is_someone) {
    if (state ==  MENU_WALKING) {
      previous_state = MENU_WALKING;
      Serial.println("AIUUUUUTO!!!!");
      state = SCARING;
      scaring_state = POPDOWN;
      popdown_timer = now;
    }
    }
    else {
    if (state == LINE_FOLLOWING) {
      previous_state = LINE_FOLLOWING;
      state = MENU_REACHING;
    }
    }

    //Just for testing
    //state = MENU_WALKING;
  */

  //the state of the robot must be modified only in this switch
  switch (state) {

    case MENU_REACHING:
      Serial.println("MENU_REACHING");

      if (start_to_count) {
        if (reach_the_menu()) {
          steps_on_menu++;
          Serial.print("START TO COUNT - STEP #");
          Serial.println(steps_on_menu);
        }
      } else {
        //init the menu reaching variables
        start_to_count = true;
        steps_on_menu = 0;
      }

      if (steps_on_menu == 3) {
        //reset the line reaching variables
        start_to_count = false;
        steps_on_menu = 0;

        if (previous_state == SCARING)
          state = LINE_FOLLOWING;
        else
          state = MENU_WALKING;

        previous_state = MENU_REACHING;
      }
      break;

    case MENU_WALKING:
      Serial.println("MENU_WALKING");
      menu_walking();
      break;

    case SCARING:

      switch (scaring_state) {
        case POPDOWN: //the hermit closes the eyes, stops walking and the shell goes down
          digitalWrite(LED_PIN, LOW);
          Serial.println("SCARING - POPDOWN");
          speeds[0] = 0;
          speeds[1] = 0;
          if (now - popdown_timer < POPDOWN_INTERVAL) {
            if (servo_pos > POS_MIN)
              servo_pos = servo_pos - 15;
            Serial.print("servo_pos = ");
            Serial.println(servo_pos);
            servo.write(servo_pos);
            delay(15);
          }
          else {
            scaring_state = HIDING;
            hiding_timer = now;
          }
          break;

        case HIDING: //the hermit remains still and inside its shell
          Serial.println("SCARING - HIDING");
          speeds[0] = 0;
          speeds[1] = 0;
          if (now - hiding_timer > HIDING_INTERVAL) {
            scaring_state = POPUP;
            popup_timer = now;
          }

          break;

        case POPUP:
          Serial.println("SCARING - POPUP");
          if (now - popup_timer < POPUP_INTERVAL) {
            if (servo_pos < POS_MAX)
              servo_pos = servo_pos + 3;
            Serial.print("servo_pos = ");
            Serial.println(servo_pos);
            servo.write(servo_pos);
            delay(15);
          }
          else {
            //two blinking eyes and then the hermit starts reching the line
            digitalWrite(LED_PIN, HIGH);
            delay(250);
            digitalWrite(LED_PIN, LOW);
            delay(100);
            digitalWrite(LED_PIN, HIGH);
            delay(250);
            digitalWrite(LED_PIN, LOW);
            delay(100);
            digitalWrite(LED_PIN, HIGH);
            delay(2000);
            previous_state=SCARING;
            state = MENU_REACHING;
          }
      }

      break;

    case LINE_REACHING:
      Serial.println("LINE_REACHING");
      if (reach_the_line(STRAIGHT)) {
        previous_state = LINE_REACHING;
        state = SCARING;
        scaring_state = POPDOWN;
        popdown_timer = now;
      }
      break;

    case LINE_FOLLOWING:
      Serial.println("LINE_FOLLOWING");
      follow_the_line();
      break;
  }


  if (now - obstacle_timer < OBSTACLE_INTERVAL) {
    Serial.println("Obstacle avoiding...");
    //obstacle avoiding...
    if (state == LINE_FOLLOWING) {
      speeds[0] = 0;
      speeds[1] = 0;
    } else {
      movement = SPIN;
      spin_direction = chosen_spin_direction;
      speeds[0] = LOW_SPEED;
      speeds[1] = LOW_SPEED;
    }
    digitalWrite(LED_PIN, HIGH);
  }
  else if (obstacle() == true) {
    Serial.println("Obstacle!!!!");
    //it is the first time I have encountered the colour to be avoided, I have to start the timer
    obstacle_timer = now;
    spin_sensing_timer = now;
    need_to_spin = false;
    speeds[0] = 0;
    speeds[1] = 0;
    chosen_spin_direction = (random(0, 2) == 0) ? CLOCKWISE : COUNTER_CLOCKWISE;//(blue)
    //chosen_spin_direction = COUNTER_CLOCKWISE;//(blank)
    digitalWrite(LED_PIN, LOW);
  }

  show_movement_values();
  move(speeds[0], speeds[1], straight_direction, spin_direction, turn_direction, movement);
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
  byte random_movement_speed;
  movement_probability = random(0, 100);
  switch (movement_probability)
  {
    case 0 ... 49:
      movement_interval = FORWARD_INTERVAL;
      movement = STRAIGHT; //variable took from the enum
      straight_direction = STRAIGHT_FORWARD;

      //set the two speeds
      speeds[0] = random(0, 100);
      if (speeds[0] < LOW_SPEED) {
        speeds[0] = LOW_SPEED;
        speeds[1] = LOW_SPEED;
      }
      else if (speeds[0] > LOW_SPEED && speeds[0] < HIGH_SPEED) {
        speeds[0] = MEDIUM_SPEED;
        speeds[1] = MEDIUM_SPEED;
      }
      else {
        speeds[0] = HIGH_SPEED;
        speeds[1] = HIGH_SPEED;
      }

      break;

    case 50 ... 100:
      movement_interval = TURN_INTERVAL;
      movement = TURN; //variable took from the enum
      turn_direction = TURN_FORWARD;
      //set the two speeds
      random_movement_speed = random(0, 100);
      if (random_movement_speed < LOW_SPEED) {
        speeds[0] = LOW_SPEED;
        speeds[1] = MEDIUM_SPEED;
      }
      else if (random_movement_speed >= LOW_SPEED && random_movement_speed < MEDIUM_SPEED) {
        speeds[0] = LOW_SPEED;
        speeds[1] = HIGH_SPEED;
      }
      else if (random_movement_speed >= MEDIUM_SPEED && random_movement_speed < HIGH_SPEED) {
        speeds[0] = HIGH_SPEED;
        speeds[1] = LOW_SPEED;
      }
      else {
        speeds[0] = MEDIUM_SPEED;
        speeds[1] = LOW_SPEED;
      }

  }
}


//It moves the robot according to the parameters.
void move(byte speed1, byte speed2, straight_movement_type straight_direction, spin_movement_type spin_direction, turn_movement_type turn_direction, movement_type type)
{
  switch (type) {

    case STRAIGHT:
      move_straight (speed1, straight_direction);
      break;

    case TURN:
      turn (speed1, speed2, turn_direction);
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
void turn(byte right_speed, byte left_speed, bool direction)
{
  digitalWrite(STBY_PIN, HIGH);
  //set direction motor 1
  digitalWrite(AIN1_PIN, direction);
  digitalWrite(AIN2_PIN, !direction);
  //set direction motor 2
  digitalWrite(BIN1_PIN, direction);
  digitalWrite(BIN2_PIN, !direction);
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
  float val = analogRead(LINESENSOR_PIN);
  Serial.print("LINE SENSOR: ");
  Serial.println(val);
  if (val > LINETHRESHOLD)
  {
    return BLACK;
  }
  else return WHITE;
}

void show_movement_values() {
  switch (movement) {
    case STRAIGHT:
      if (straight_direction == STRAIGHT_FORWARD)
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

  //set spin movement
  movement = SPIN;
  spin_direction = CLOCKWISE;//(blue)
  //spin_direction = COUNTER_CLOCKWISE;//(blank)
  speeds[0] = LOW_SPEED;
  speeds[1] = 0;

  if (get_colour() == WHITE) {
    menu_reached = true;
    spin_sensing_timer = now;
    need_to_spin = false;
  }
  else
    menu_reached = false;

  return menu_reached;
}


//It returns true if the robot has reached the black line, otherwise false
bool reach_the_line(movement_type type_of_movement) {
  bool line_reached;

  if (get_colour() == WHITE) {
    line_reached = false;

    if (type_of_movement == STRAIGHT) {
      //set forward movement
      movement = STRAIGHT;
      straight_direction = STRAIGHT_FORWARD;
      speeds[0] = HIGH_SPEED;
      speeds[1] = HIGH_SPEED;
    } else {
      //set spinning movement
      movement = SPIN;
      spin_direction = CLOCKWISE;//(blue)
      //spin_direction = COUNTER_CLOCKWISE;//(blank)
      speeds[0] = LOW_SPEED;
      speeds[1] = LOW_SPEED;
    }
  }
  else {
    line_reached = true;
    Serial.println("Line reached!");
    if (movement == SPIN) {
      speeds[0] = 0;
      speeds[1] = 0;
    }
  }
  return line_reached;
}

void follow_the_line() {

  if (get_colour() == BLACK) {
    speeds[0] = 0;//(blue)
    speeds[1] = HIGH_SPEED;//(blue)
    //speeds[0] = HIGH_SPEED;//(blank)
    //speeds[1] = 0;//(blank)
    type_of_steering = TO_THE_LEFT;
    Serial.println("steering left...");
  }
  else {
    speeds[0] = MEDIUM_SPEED;//(blue)
    speeds[1] = 0;//(blue)
    //speeds[0] = 0;//(blank)
    //speeds[1] = MEDIUM_SPEED;//(blank)
    type_of_steering = TO_THE_RIGHT;
    Serial.println("steering right...");

  }
  movement = TURN;
  turn_direction = TURN_FORWARD;

}

void menu_walking() {

  if (need_to_change_movement()) {
    movement_timer = now;
    set_random_values();
    show_movement_values();
  }

  //****************************************************************************************************************
  //It checks if there is a BLACK LINE under the robot. If it occours, it lets the robot spinning counter-clockwise.
  //****************************************************************************************************************
  if (now - region_timer < BLACK_LINE_SPIN_INTERVAL) {
    //region avoiding...
    movement = SPIN;
    spin_direction = CLOCKWISE;//(blue)
    //spin_direction = COUNTER_CLOCKWISE;//(blank)
    speeds[0] = LOW_SPEED;
    speeds[1] = LOW_SPEED;
  }
  else if (get_colour() == BLACK) {
    Serial.println("Region to avoid!!!!");
    //it is the first time I have encountered the colour to be avoided, I have to start the timer
    region_timer = now;
    spin_sensing_timer = now;
    need_to_spin = false;
  }

  //*********************************************
  // SPIN SENSING - start
  //*********************************************
  if (now - spin_sensing_timer > SPIN_SENSE_INTERVAL) {
    Serial.println("Sense spinning:");
    spin_sensing_timer = now;
    need_to_spin = true;
  }
  if (need_to_spin) {
    if (now - spin_sensing_timer < SPIN_INTERVAL) {
      movement = SPIN;
      spin_direction = CLOCKWISE;//(blue)
      //spin_direction = COUNTER_CLOCKWISE;//(blank)
      speeds[0] = LOW_SPEED;
      speeds[1] = LOW_SPEED;
    } else {
      need_to_spin = false;
    }
  }
  //*********************************************
  // SPIN SENSING - end
  //*********************************************

}

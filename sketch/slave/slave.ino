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
#define LOW_SPEED 40
#define HIGH_SPEED 60
#define EXTREME_SPEED 100
#define MEDIUM_SPEED (LOW_SPEED+HIGH_SPEED)/2

//colour sensing
#define LINETHRESHOLD 500
//it is the minimum value returned from the reflectance sensor when the floor is black

//numbers of loops that each movement takes
#define FORWARD_INTERVAL 1000           //time spent going forward
#define TURN_INTERVAL 1000              //time spent turning
#define SPIN_INTERVAL 1000               //time spent spinning
#define BACKWARD_INTERVAL 10            //time spent going backward
#define OBSTACLE_INTERVAL 700           //time spent to spin in order to avoid obstacles
#define BLACK_LINE_SPIN_INTERVAL 500    //time spent to spin in order to avoid the black line

//IR communication
#define IR_SENSE_INTERVAL 20000           //time frequence used to sense the infrareds
#define PEOPLE_PACKET 0xE0E0906F        //packet received everytime a person approaches
#define NO_PEOPLE_PACKET 0xE0E0906A        //packet received everytime a person goes away

//line following
#define LINE_FOLLOWING_SET_POINT 500

bool line_crossed;
byte steps_on_menu;

enum state_type {
  MENU_WALKING,
  MENU_REACHING,
  SCARING,
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
unsigned long now;
unsigned long obstacle_timer;
unsigned long region_timer;
unsigned long movement_timer;

unsigned long movement_interval;
unsigned long region_interval;

bool need_to_start_the_ostacle_timer;

byte movement_probability;
movement_type movement;
steering_type type_of_steering;
float speeds[2];
spin_movement_type spin_direction, chosen_spin_direction;
turn_movement_type turn_direction;
straight_movement_type straight_direction;

state_type state;
state_type previous_state;

bool there_is_someone;

//Error used in the line following
float error_value;


//menu reaching variables
byte steps_beyond_the_line;
byte steps_on_the_line;
color_type colour;
bool start_to_count;

int pos = 0;    // variable to store the servo position


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

  servo.attach(SERVO_PIN);

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
        Serial.println("PACKET:");
        Serial.println(HEX, results.value);
        if (results.value == PEOPLE_PACKET) {
          if (state ==  MENU_WALKING){
            previous_state = MENU_WALKING;
            state = SCARING;
          }
          Serial.println("There's someone!");
        }
        else if (results.value == NO_PEOPLE_PACKET) {
          //No one wants to see the menu
          Serial.println("No one...");
          if (state == LINE_FOLLOWING){
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
  */
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
      state = SCARING;
    }
  }
  else {
    if (state == LINE_FOLLOWING) {
      previous_state = LINE_FOLLOWING;
      state = MENU_REACHING;
    }
  }

  //Just for testing
  //state = LINE_FOLLOWING;


  //the state of the robot must be modified only in this switch
  switch (state) {

    case MENU_REACHING:
      Serial.println("MENU_REACHING");
      if (reach_the_menu() == false) {
        //init the menu reaching variables
        start_to_count = true;
        steps_beyond_the_line = 0;
      }
      if (start_to_count) {
        if (reach_the_menu()) {
          steps_beyond_the_line++;
          Serial.print("START TO COUNT - STEP #");
          Serial.println(steps_beyond_the_line);
        }
      }
      if (steps_beyond_the_line == 3) {
        //reset the line reaching variables
        start_to_count = false;
        steps_beyond_the_line = 0;

        previous_state = MENU_REACHING;
        if (previous_state == LINE_REACHING)
          state = LINE_FOLLOWING;
        else
          state = MENU_WALKING;
      }
      break;

    case MENU_WALKING:
      Serial.println("MENU_WALKING");
      menu_walking();
      break;

    case SCARING:
      Serial.println("SCARING");
      shell_popup();
      previous_state = SCARING;
      state = LINE_REACHING;
      break;
      
    case LINE_REACHING:
      Serial.println("LINE_REACHING");
      if (reach_the_line(STRAIGHT)) {
        previous_state = LINE_REACHING;
        state = MENU_REACHING;
      }
      break;

    case LINE_FOLLOWING:
      Serial.println("LINE_FOLLOWING");
      follow_the_line();
      break;
  }

  Serial.print("RIGHT: ");
  Serial.println(speeds[0]);
  Serial.print("LEFT: ");
  Serial.println(speeds[1]);
  move(speeds[0], speeds[1], straight_direction, spin_direction, turn_direction, movement);
  /*  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      servo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  */
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

    case 30 ... 100:
      movement_interval = TURN_INTERVAL;
      movement = TURN; //variable took from the enum
      turn_direction = TURN_FORWARD;
      //set the two speeds
      speeds[0] = random(0, 100);
      if (speeds[0] < LOW_SPEED) {
        speeds[0] = LOW_SPEED;
        speeds[1] = MEDIUM_SPEED;
      }
      else if (speeds[0] > LOW_SPEED && speeds[0] < HIGH_SPEED) {
        speeds[0] = LOW_SPEED;
        speeds[1] = HIGH_SPEED;
      }
      else {
        speeds[0] = MEDIUM_SPEED;
        speeds[1] = HIGH_SPEED;
      }
      break;
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
  movement = TURN;
  turn_direction = TURN_FORWARD;
  speeds[0] = LOW_SPEED;
  speeds[1] = 0;

  //I'm steering TO_THE_RIGHT, so I have to cross the black line which is on my left
  if (get_colour() == BLACK) {
    line_crossed = true;
    steps_on_menu = 0;
  }


  if (line_crossed) {
    if (get_colour() == WHITE)
      menu_reached = true;
    else
      menu_reached = false;
  }

  return menu_reached;
}

void shell_popup(){
  //something similar to sweep
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
      speeds[0] = LOW_SPEED;
      speeds[1] = LOW_SPEED;
    } else {
      //set spinning movement
      movement = SPIN;
      spin_direction = COUNTER_CLOCKWISE;
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
    speeds[0] = LOW_SPEED;
    speeds[1] = 0;
    type_of_steering = TO_THE_LEFT;
    Serial.println("steering left...");
  }
  else {
    speeds[0] = 0;
    speeds[1] = LOW_SPEED;
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
    spin_direction = chosen_spin_direction;
    //movement = TURN;
    turn_direction = TURN_BACKWARD;
    speeds[0] = LOW_SPEED;
    speeds[1] = HIGH_SPEED;
  }
  else if (get_colour() == BLACK) {
    Serial.println("Region to avoid!!!!");
    //it is the first time I have encountered the colour to be avoided, I have to start the timer
    region_timer = now;
    //chosen_spin_direction = (random(0, 2) == 0) ? CLOCKWISE : COUNTER_CLOCKWISE;
    chosen_spin_direction = COUNTER_CLOCKWISE;
    speeds[0] = 0;
    speeds[1] = 0;
  }

  /*
    if (get_colour() == BLACK) {
      if (start_to_count) {
        steps_on_the_line++;
        Serial.print("START TO COUNT - STEP ON THE LINE #");
        Serial.println(steps_on_the_line);
      }
      else {
        //init the menu reaching variables
        start_to_count = true;
        steps_on_the_line = 0;
      }
    } else {
      start_to_count = false;
      steps_on_the_line = 0;
    }

    if (steps_on_the_line == 2) {
      movement = STRAIGHT;
      straight_direction = STRAIGHT_BACKWARD;


      //movement = SPIN;
      //spin_direction = random(0, 2);

      //reset the line reaching variables
      start_to_count = false;
      steps_on_the_line = 0;

    }

    //**********************************************************************************************************************************************
  */
  /*
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
  */
}

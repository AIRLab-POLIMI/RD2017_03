#include <SoftwareSerial.h>
#include <DFPlayer_Mini_Mp3.h>
#include <IRremote.h>
#include <NewPing.h>
#include <Servo.h>

//PIN SONAR 1
#define TRIGGER_PIN1 6
#define ECHO_PIN1 7
//PIN SONAR 2
#define TRIGGER_PIN2 10
#define ECHO_PIN2 11
//PIN SONAR 3
#define TRIGGER_PIN3 12
#define ECHO_PIN3 13
//THRESHOLD(IN cm) TO DEFINE IF A PERSON IS CLOSE TO THE OYSTER
#define PRESENCE_THRESHOLD 100
//MAX DISTANCE IN cm TO PING
#define MAX_DISTANCE 100

/*-----------------------------------------
 * SERVO
 ------------------------------------------*/

//PIN BUBBLE GUN SERVO
#define BUBBLEGUN_SERVO_PIN 9

//TIME BETWEEN THE COMMAND OF A MOVEMENT TO A SERVO AND ITS DETACH
#define DETACH_TIME 300
//HOW MUCH TIME THE BUBBLE GUN WILL BE TRIGGERED
#define TRIG_TIME 1000
//TRIG / UNTRIG POSITION OF THE BUBBLE GUN'S SERVO
#define UN_TRIG_POSITION 140
#define TRIG_POSITION 75
/*---------------------------------------
 * END SERVO
 ----------------------------------------*/
//PIN DFPLAYER
#define RX_PIN 4
#define TX_PIN 2

//DEFINE THE DIMENSION OF THE TABLE FOR THE SONAR READING OPERATION
#define NUMBEROFMISSREAD 3

//POSSIBLE STATE OF THE HISTORY PRESENCE TABLE
#define STRONGFALSE 0
#define AMBIGUOUS 1
#define STRONGTRUE 2
//TIME THAT MUST PASS BETWEEN AN UNTRIG OF THE BUBBLE GUN AND THE FOLLOWING TRIG WITHOUT ANY STRONGTRUE IN MEANTIME
#define STRONGFALSE_TIME 5000
//TIME THAT HAVE TO PASS AFTER A STRONGTRUE WITHOUT ANY STRONGFALSE IN ORDER TO SAY THAT SOMEONE IS IN FRONT OF THE OYSTER
#define PEOPLE_SEEN_TIME 250

/*
 * IR COMMUNICATION
 */
//TIME THAT DEFINE FOR HOW MUCH TIME I WILL SEND A PEOPLE PACKET WITHOUT MODIFYING THE STATE
#define UNCHECKED_SENDING_PEOPLE_TIME 5000
//TIME THAT DEFINE FOR HOW MUCH I WILL SEND THE NO PEOPLE PACKET
#define SENDING_NO_PEOPLE_TIME 250

#define PEOPLE_PACKET 0xE0E0906F
#define NO_PEOPLE_PACKET 0xE0E0906A
//DIMENSION IN BIT OF THE PACKET SENT
#define PACKET_DIMENSION 32
#define DELAY_BETWEEN_IR_SENDING 30


#define MIN_VOLUME 15
#define MAX_VOLUME 30
#define DEFAULT_SONG 1
#define PEOPLE_INVITATION 2
#define NUMBER_OF_TRACK_INVITATION 4

int i = 0;
//SONAR READ VALUE
unsigned int cm_sonar1, cm_sonar2, cm_sonar3;
/*-------------------------------------------------------------------------
   TIMER
  -------------------------------------------------------------------------*/
unsigned long people_seen_timer, sending_people_timer, sending_no_people_timer, sonar_waiting_timer = 0;
unsigned long ir_timer = 0, strongfalse_timer = 0, bubble_gun_timer = 0, detach_bubble_gun_timer = 0;

/*-----------------------------------------------------------------------------
   FLAG RELATED TO TIMER
  -----------------------------------------------------------------------------*/
bool timer_flag = false, send_people_flag = false, send_no_people_flag = false;
bool strongfalse_flag = false, bubble_gun_trig = false;
bool detach_bubble_gun_flag = false;

//TABLE HISTORY OF PEOPLE PRESENCE
bool people_presence_history [NUMBEROFMISSREAD] = {false, false};

//Used to store the actual state of presence table : STRONGTRUE, STRONGFALSE, AMBIGUOUS (WHEN A READ SEE PRESENCE AND THE FOLLOWING/PREVIOUS DOES NOT)
int presence_table_state = 0;
enum state_type {
  BUBBLE_GUN_TRIGGERED_STATE,
  NO_PEOPLE_STATE,
  PEOPLE_SEEN_STATE,
  PEOPLE_PRESENCE_STATE
};

//ACTUAL STATE VARIABLE
state_type actual_state = NO_PEOPLE_STATE;

//first trigger pin, second echo pin
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);

//----------RX arduino ---> TX dfPlayer
SoftwareSerial mySerial(RX_PIN, TX_PIN); // RX, TX
byte dfplayer_volume = 30;

//sender object, pin 3

IRsend ir_sender;


//Servo variable:
Servo trigger_bubble_gun;


void setup() {
  //mp3 serial
  dfplayer_volume = 30;
  mySerial.begin (9600);
  mp3_set_serial (mySerial);
  Serial.begin(115200);
  //mp3 set volume
  mp3_set_volume (dfplayer_volume);
  mp3_play (DEFAULT_SONG);
  un_trig_bubble_gun();

  //for test:
  //actual_state = PEOPLE_PRESENCE_STATE;
  //people_presence_history[NUMBEROFMISSREAD-1] = true;
}


void loop() {
  sonars_perceiving();
  update_people_presence_history();
  Serial.println (cm_sonar1);
  Serial.println (cm_sonar2);
  Serial.println (cm_sonar3);
  presence_table_state = check_presence_table();
  switch (presence_table_state)
  {
    case STRONGTRUE : Serial.println("STRONGTRUE"); break;
    case STRONGFALSE : Serial.println("STRONGFALSE"); break;
    case AMBIGUOUS : Serial.println("AMBIGUOUS"); break;
  }

  switch (actual_state)
  {
    case NO_PEOPLE_STATE:
      Serial.println("NO_PEOPLE_STATE");
      no_people_state_action();
      break;
    case BUBBLE_GUN_TRIGGERED_STATE:      
      Serial.println("BUBBLE_GUN_TRIGGERED_STATE");
      bubble_gun_triggered_action();
      break;
    case PEOPLE_SEEN_STATE:      
      Serial.println("PEOPLE_SEEN_STATE");
      people_seen_action();
      break;
    case PEOPLE_PRESENCE_STATE:
      Serial.println("PEOPLE_PRESENCE_STATE");
      people_presence_action();
      break;
    default:
      break;

  }
  if (send_no_people_flag == true)
  {
    // send no people packet
    send_no_people_packet();
    
    if ( expired_timer(sending_no_people_timer , SENDING_NO_PEOPLE_TIME))
    {
      Serial.println("Stop no people packet");
      send_no_people_flag = false;
    }
  }
  
  if (detach_bubble_gun_flag)
  {
    if (expired_timer(detach_bubble_gun_timer , DETACH_TIME))
    {
      trigger_bubble_gun.detach();
      detach_bubble_gun_flag=false;
    }
  }
}
/*-------------------------------------------------
   NO PEOPLE STATE ACTION
  --------------------------------------------------*/

void no_people_state_action()
{
  switch (presence_table_state)
  {
    case AMBIGUOUS:
    case STRONGFALSE:
      if (strongfalse_flag == false && presence_table_state == STRONGFALSE)
      {
        strongfalse_flag = true;
        strongfalse_timer = millis();
      }
      else if ( strongfalse_flag == true && expired_timer(strongfalse_timer, STRONGFALSE_TIME))
      {
        strongfalse_flag = false;
        trig_bubble_gun();
        actual_state = BUBBLE_GUN_TRIGGERED_STATE;
        bubble_gun_timer = millis();
      }
      break;
    case STRONGTRUE:
      if (strongfalse_flag == true) strongfalse_flag = false;
        people_seen_timer = millis();
        actual_state = PEOPLE_SEEN_STATE;
      break;
    default:
      break;
  }
}
/*-----------------------------------------
 * BUBBLE GUN TRIGGERED ACTION 
 ------------------------------------------*/
 
void bubble_gun_triggered_action()
{
  switch(presence_table_state)
  {
    case AMBIGUOUS:
    case STRONGFALSE:
      if(expired_timer(bubble_gun_timer, TRIG_TIME))
      {
        Serial.println("DISABLE BUBBLEGUN");
        un_trig_bubble_gun();
        actual_state = NO_PEOPLE_STATE;
      }
      break;
     case STRONGTRUE:
      un_trig_bubble_gun();
      actual_state = PEOPLE_SEEN_STATE;
      break;
  }
}
/*-------------------------------------------------------
 * PEOPLE SEEN ACTION
 --------------------------------------------------------*/
byte played_track = PEOPLE_INVITATION;
void people_seen_action()
{
  switch (presence_table_state)
  {
    case STRONGFALSE:
      actual_state = NO_PEOPLE_STATE;
      break;
    case AMBIGUOUS:
    case STRONGTRUE:
      if (expired_timer(people_seen_timer, PEOPLE_SEEN_TIME))
      {
        send_no_people_flag = false;
        if(played_track >= (PEOPLE_INVITATION + NUMBER_OF_TRACK_INVITATION))
          played_track = PEOPLE_INVITATION;
        mp3_play (played_track);
        mp3_single_loop (true);
        played_track++;
        Serial.println("Playing " + played_track);
        actual_state = PEOPLE_PRESENCE_STATE;        
        sending_people_timer = millis();
      }
      break;
    default:
      break;


  }
}
/*-------------------------------------------------
 * PEOPLE PRESENCE ACTION
 --------------------------------------------------*/
void people_presence_action()
{
  if (expired_timer(sending_people_timer,UNCHECKED_SENDING_PEOPLE_TIME))
  {
    switch(presence_table_state)
    {
        case STRONGTRUE:
          sending_people_timer = millis();
          break;
        case AMBIGUOUS:
          send_people_packet();
          break;
        case STRONGFALSE:
          send_no_people_flag = true;
          sending_no_people_timer = millis();
          mp3_play (DEFAULT_SONG);
          mp3_single_loop (true);
          actual_state = NO_PEOPLE_STATE;
          break;
    }
  }
  else
  {
    send_people_packet();
  }
}
// GIVEN A TIMER AND AN INTERVAL, RETURN IF THE TIME IS PASSED(TRUE) OR NOT(FALSE)

bool expired_timer(unsigned long timer, unsigned int interval)
{
  return (millis() - timer > interval);
}

//UPDATE THE PEOPLE PRESENCE HISTORY USING THE NEW VALUE FROM THE SONAR

void update_people_presence_history()
{
  //SHIFTING OPERATION
  for(int i = 1; i < NUMBEROFMISSREAD; i++) people_presence_history[i-1] = people_presence_history[i];
  
  if (presence_perceived(cm_sonar1) || presence_perceived(cm_sonar2) || presence_perceived(cm_sonar3))
  {
    people_presence_history [NUMBEROFMISSREAD - 1] = true;
  }
  else
  {
    people_presence_history[NUMBEROFMISSREAD - 1] = false;
  }
}

/* RETURN A VALUE BASED ON THE VALUES ON THE PEOPLE PRESENCE HISTORY BY FOLLOWING THIS RULES:
  ALL TRUE     ------> STRONGTRUE
  ALL FALSE    ------> STRONGFALSE
  MIXED CASE   ------> AMBIGUOUS
*/
int check_presence_table()
{ 
  bool first_table_value = people_presence_history[0];
  for(int i = 1; i <NUMBEROFMISSREAD; i++)
  {
    if (people_presence_history[i] != first_table_value) return AMBIGUOUS;
  }
  if(first_table_value) return STRONGTRUE;
  else return STRONGFALSE;
}

//ACTIVATE THE BUBBLE GUN BY MOVING THE SERVO TO A CERTAIN POSITION

void trig_bubble_gun()
{
  trigger_bubble_gun.attach(BUBBLEGUN_SERVO_PIN);
  trigger_bubble_gun.write(TRIG_POSITION);
  //detach_bubble_gun_flag = true;
  //detach_bubble_gun_timer = millis();
}

//DEACTIVATE THE BUBBLE GUN BY MOVING THE SERVO TO A CERTAIN POSITION

void un_trig_bubble_gun()
{
  trigger_bubble_gun.attach(BUBBLEGUN_SERVO_PIN);
  trigger_bubble_gun.write(UN_TRIG_POSITION);
  detach_bubble_gun_flag = true;
  detach_bubble_gun_timer = millis();
}

//PERCEIVE DISTANCE FROM THE SONAR

void sonars_perceiving()
{
  cm_sonar1 = sonar1.ping_cm();
  cm_sonar2 = sonar2.ping_cm();
  cm_sonar3 = sonar3.ping_cm();  
}


bool presence_perceived( unsigned int distance)
{
  if (distance > 0 && distance < PRESENCE_THRESHOLD) return true;
  else return false;

}

// SEND THE PACKET TO THE HERMIT TELLING THAT THE PERSON/PEOPLE IN FRONT OF THE OYSTER WENT AWAY

void send_no_people_packet()
{
  
  if (expired_timer(ir_timer,DELAY_BETWEEN_IR_SENDING))
  {
    Serial.println("No People Packet");
    ir_sender.sendSony(NO_PEOPLE_PACKET, PACKET_DIMENSION);
    ir_timer = millis();
  }
}

//SEND THE PACKET TO THE HERMIT TELLING THAT THERE IS SOMEONE IN FRONT OF THE OYSTER

void send_people_packet()
{
  if (expired_timer(ir_timer,DELAY_BETWEEN_IR_SENDING))
  {
    Serial.println("People Packet");
    ir_sender.sendSony(PEOPLE_PACKET, PACKET_DIMENSION);
    ir_timer = millis();
  }
}



#include <IRremote.h>
#include <NewPing.h>

//Sonar pin
#define TRIGGER_PIN1 6
#define ECHO_PIN1 7
#define TRIGGER_PIN2 8 
#define ECHO_PIN2 11
#define TRIGGER_PIN3 12
#define ECHO_PIN3 13

#define MAX_DISTANCE 300 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 50
#define NUMBEROFMISSREAD 2
#define PRESENCE_THRESHOLD 50
#define STRONGTRUE 2
#define AMBIGUOUS 1 
#define STRONGFALSE 0
#define TIME_THRESHOLD 1000
#define UNCHECKED_SENDING_PEOPLE_TIME 3000
#define SENDING_NO_PEOPLE_TIME 1000
#define IR_PEOPLE_PACKET
#define IR_NO_PEOPLE_PACKET
#define SONAR_FURTHER_READING_DELAY 50
#define PEOPLE_PACKET 0xE0E0906F
#define NO_PEOPLE_PACKET 0xE0E0906A
#define DELAY_BETWEEN_IR_SENDING 30


int i = 0;
unsigned int cm_sonar1, cm_sonar2, cm_sonar3;
unsigned long sonar_timer, sending_people_timer, sending_no_people_timer, sonar_further_reading, sonar_waiting_timer = 0;
unsigned long sender_timer = 0;
bool timer_flag = false, send_people_flag = false, send_no_people_flag = false;
bool people_presence_history [NUMBEROFMISSREAD] = {false, false};


//first trigger pin, second echo pin
NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE);

//sender object, pin 3

IRsend ir_sender;

void setup() {
  Serial.begin(115200);
}

void loop() {
    cm_sonar1 = sonar1.ping_cm();
    cm_sonar2 = sonar2.ping_cm();
    cm_sonar3 = sonar3.ping_cm();
    sonar_waiting_timer = millis();
    update_people_presence_history();
    Serial.println (cm_sonar1);
    Serial.println (cm_sonar2);
    Serial.println (cm_sonar3);
    switch (check_presence_table())
    {
      case STRONGTRUE : Serial.println("STRONGTRUE"); break;
      case STRONGFALSE : Serial.println("STRONGFALSE"); break;
      case AMBIGUOUS : Serial.println("AMBIGUOUS"); break;
    }
    if (send_people_flag == true)
    {
      //send packet
      send_people_packet();
      Serial.println("Sending Packet");
      //checking if time is ended
      if ( millis() - sending_people_timer > UNCHECKED_SENDING_PEOPLE_TIME)
      {
        if (check_presence_table () == STRONGTRUE || check_presence_table() == AMBIGUOUS)
        {
          sending_people_timer = millis();
        }
        else
        {
          send_people_flag = false;
          send_no_people_flag = true;
          sending_no_people_timer = millis ();
          Serial.println("First no people packet sent");
          //send no people packet
          send_no_people_packet();
        }
      }

    }

    else if (check_presence_table() == STRONGTRUE || check_presence_table() == AMBIGUOUS)
    {

      if (timer_flag == false && check_presence_table() == STRONGTRUE)
      {

        Serial.println("Start presence Timer");
        sonar_timer = millis();
        timer_flag = true;
      }
      else if ( timer_flag == true)
      {
        if (millis() - sonar_timer >= TIME_THRESHOLD)
        {
          timer_flag = false;
          send_no_people_flag = false;
          send_people_flag = true;
          send_people_packet();
          //send packet
          Serial.println("First packet Sent");
          //initialize timer
          sending_people_timer = millis();
        }
      }
    }
    else if ( check_presence_table() == STRONGFALSE && timer_flag == true)
    {
      Serial.println("Stop Sonar Timer");
      timer_flag = false;
    }
    else if (send_no_people_flag == true)
    {
      Serial.println("No people packet");
      // send no people packet
      send_no_people_packet();
      if ( millis () - sending_no_people_timer > SENDING_NO_PEOPLE_TIME)
      {
        Serial.println("Stop no people packet");
        send_no_people_flag = false;
      }
    }
  //delay(300);
}



void update_people_presence_history()
{
  people_presence_history[0] = people_presence_history[1]; //shifting, in case of a greater number of allowed missread, will be substituted with a proper function
  if (presence_perceived(cm_sonar1) || presence_perceived(cm_sonar2) || presence_perceived(cm_sonar3))
  {
    people_presence_history [NUMBEROFMISSREAD - 1] = true;
  }
  else
  {
    people_presence_history[NUMBEROFMISSREAD - 1] = false;
  }
}



int check_presence_table()
{
  if (people_presence_history[0] == true && people_presence_history[1] == true) return STRONGTRUE;
  else if (people_presence_history[0] == false && people_presence_history[1] == false) return STRONGFALSE;
  else return AMBIGUOUS;
}



bool presence_perceived( unsigned int distance)
{
  if (distance > 0 && distance < PRESENCE_THRESHOLD) return true;
  else return false;

}

void send_no_people_packet()
{
  if(millis() - sender_timer >= DELAY_BETWEEN_IR_SENDING)
  {
      ir_sender.sendSony(NO_PEOPLE_PACKET, 32);
      sender_timer = millis();
  }
}

void send_people_packet()
{
  if(millis() - sender_timer >= DELAY_BETWEEN_IR_SENDING)
  {
    ir_sender.sendSony(PEOPLE_PACKET, 32);
    sender_timer = millis ();
  }  
}


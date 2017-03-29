#include <IRremote.h>

#define LEDPIN 2

int RECV_PIN = 7;

IRrecv irrecv(RECV_PIN);

decode_results results;
void setup() {
  // put your setup code here, to run once:
  pinMode(LEDPIN,OUTPUT);
   irrecv.enableIRIn();
}

void loop() {
  if (irrecv.decode(&results)) {
    irrecv.resume(); // Receive the next value
    digitalWrite(LEDPIN,HIGH);
  }
  else
  digitalWrite(LEDPIN,LOW);

}

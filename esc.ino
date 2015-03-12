// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.

#define MAX 2000
#define SVP 9
#define LED 13
#define MIN 960
#define DEL 5000
#define ADD 20
#include <Servo.h> 
//wait 35 seconds
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = MIN;    // variable to store the servo position
 
void setup() 
{ 
  myservo.attach(SVP);  // attaches the servo on pin 9 to the servo object
  myservo.writeMicroseconds(pos);
  pinMode(LED, OUTPUT);
}
 
 
void loop() 
{ 
  digitalWrite(LED, HIGH);
  pos +=ADD;
  myservo.writeMicroseconds(pos);
  delay(DEL);
  if(pos == MAX){
    pos = MIN;
  }
  digitalWrite(LED, LOW);
  delay(DEL);
}

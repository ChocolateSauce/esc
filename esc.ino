#define MAX 1120
#define MIN 1050
#define CHG 1
#define SVP 6
#define LPR 4
#define LPG 5
#define BON 2
#define BTO 3
#define SEN A0
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 13
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

Servo myservo;  // create servo object to control a servo 
                
volatile uint8_t running;
 
volatile int pos;  

volatile uint8_t l_state;

volatile unsigned long t_zero;
volatile unsigned long t_one;
float spd = 0;
 
void setup() 
{ 
  
  running = 0;
  t_zero = 0;
  
  init_display();
  init_esc();
  
  pinMode(LPR,OUTPUT);
  pinMode(LPG,OUTPUT);
  pinMode(SEN,INPUT);
  pinMode(BON,INPUT);
  pinMode(BTO,INPUT);
  
  digitalWrite(LPR,LOW);
  digitalWrite(LPG,LOW);
  
  init_pci();
  attachInterrupt(1, up_isr, FALLING);
  attachInterrupt(0, down_isr, FALLING);
  
  l_state = digitalRead(SEN);  
  
}
 
void loop() 
{ 
  spd = (1.0f/(((t_one - t_zero)*2.0f)/1000000.0f))*60.0f;
  if (!running)
  { 
    digitalWrite(LPR,HIGH);
    digitalWrite(LPG,LOW);
  }
  else
  {
    digitalWrite(LPG,HIGH);
    digitalWrite(LPR,LOW);
  }
  
  myservo.writeMicroseconds(pos);
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(pos);
  display.println("us");
  /*if(l_state)
    display.println("latched");
  else
    display.println("unlatched");*/
  display.print(spd);
  display.display();

}

void init_esc(){
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("Initializing");
  display.display();
  display.setTextSize(2);
  
  myservo.attach(SVP);
  myservo.writeMicroseconds(980);
  delay(5000);
  pos = MIN;
  t_one = micros();
}

void init_display(){
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
}

void init_pci(){
  PCICR |= (1 << PCIE1); //enable pc interrupts
  PCMSK1 |= (1 << PCINT8); //enable intterupts for A0
}

void up_isr(){
 if (!running)
   running = 1;
  
 if (pos != MAX)
   pos += CHG; 
}

void down_isr(){
  
  if (pos != MIN){
    pos -= CHG;   
  }
  if (pos == MIN){
    running = 0;
  }
}

ISR( PCINT1_vect ){
  l_state = digitalRead(SEN);
  t_zero = t_one;
  t_one = micros();
}

#define MAX 1150
#define MIN 1060
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
#define SETPT 605.0f
#define KD 0.000f
#define KP 0.003f //0.006//0.007f
#define KI 0.00000f//0.000033/0.00003f
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

Servo myservo;  // create servo object to control pwm 
                
volatile uint8_t running;
volatile uint8_t fudgemode;
 
/*volatile*/ int pos;  

volatile uint8_t l_state;

volatile unsigned long t_zero;
volatile unsigned long t_one;

float spd = 0;
//float prev_spd = 0;
volatile float prev_error = 0;
volatile float error_sum  = 0;

void computePID(float current_spd, float setpoint, int command);

void setup() 
{ 
  fudgemode = 0;
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
  spd = 0;  
  
}
 
void loop() 
{ 
  //spd = (1.0f/(((t_one - t_zero)*2.0f)/1000000.0f))*60.0f;
  spd = (1.0f/(((t_one - t_zero))/1000000.0f))*60.0f;
  //if (prev_spd != 0)
  //  spd = (spd+prev_spd)/2;
   
  if (!running)
  { 
    digitalWrite(LPR,HIGH);
    digitalWrite(LPG,LOW);
    pos = MIN;
  }
  else
  {
    digitalWrite(LPG,HIGH);
    digitalWrite(LPR,LOW);
    
    if(fudgemode && (spd < SETPT) ){
      pos++;
      fudgemode = 0;
    }
    else if (fudgemode && (spd >= SETPT) ){
      pos--;
      fudgemode = 0;
    }
    else
      computePID(spd, SETPT, pos);
    
  }
  
  myservo.writeMicroseconds(pos);
  
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(pos);
  display.println("us");
  if(l_state)
    display.println("latched");
  else
    display.println("unlatched");
  if(fudgemode)
    display.println("m");
  display.print(spd);
  display.display();
  
  //prev_spd = spd;

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
 else
   fudgemode = 1;
  
 //if (pos != MAX)
 //  pos += CHG; 
}

void down_isr(){
  
  //if (pos != MIN){
  //  pos -= CHG;   
  //}
  //if (pos == MIN){
    running = 0;
    fudgemode = 0;
    prev_error = 0;
    error_sum  = 0;
    spd = 0;
  //}
}

ISR( PCINT1_vect ){
  l_state = digitalRead(SEN);
  if (l_state){
  t_zero = t_one;
  t_one = micros();
  }
}

void computePID(float current_spd, float setpoint, int command){
   float error = 0;
   float pidTerm = 0;
   
   error = setpoint - current_spd;
   
   pidTerm = (KP*error) + KD*(error - prev_error) + KI*error_sum;
   
   pos = constrain( command + (int) pidTerm, MIN, MAX);
   
   
   prev_error = error;
   error_sum += error;
  
}

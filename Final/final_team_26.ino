#include <WiFi.h> 
#include <WiFiUdp.h>
#include "vive510.h"
#include <esp_now.h>
#define RGBLED 18 // for ESP32S2 Devkit pin 18, for M5 stamp=2
#define SIGNALPIN1 4 // pin receiving signal from Vive circuit
#define SIGNALPIN2 5 // pin receiving signal from Vive circuit
#define UDPPORT 2510 // For GTA 2022C game 
#define STUDENTIP 154 // choose a teammembers assigned IP number
#define teamNumber 26
#define FREQ 1 // in Hz
#define pina 12
#define pinb 13
#define pinc 14
#define trigPinfr 7
#define echoPinfr 6
#define trigPinfl 38
#define echoPinfl 37
#define trigPinbr 4
#define echoPinbr 5
#define trigPinbl 41
#define echoPinbl 40
#define trigPinfront 11
#define echoPinfront 10
#define IN1 8
#define IN2 3

// Define pins for Motor Driver
#define PWM_PIN1 21
#define PWM_PIN2 33
#define PWM_PIN3 19
#define PWM_PIN4 20
#define analog0 0
#define analog4 4
#define analog1 1
#define analog2 2

//Define constants
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701

// Other variables used
long durationfr;
float distanceCmfr;
float distanceInchfr;
long durationfl;
float distanceCmfl;
float distanceInchfl;
long durationbr;
float distanceCmbr;
float distanceInchbr;
long durationbl;
float distanceCmbl;
float distanceInchbl;
long duration;
float distanceCm;
float distanceInch;
float distancemmfront;
float d=80, res_bit=11, f=20, biit=2047,l,k=20,da = 75,t;
int a1,b1,c1,a,b,c,e,d1,d2;
float side_distance_1,side_distance_2,side_distance_3,side_distance_4,distance,threshold=300;

//WiFiUDP UDPServer;
Vive510 vive1(SIGNALPIN1);
Vive510 vive2(SIGNALPIN2);

// Define the staffcomm
esp_now_peer_info_t staffcomm = {
  .peer_addr = {0x84,0xF7,0x03,0xA9,0x04,0x78}, 
  .channel = 1,             // channel can be 1 to 14, channel 0 means current channel.
  .encrypt = false,
};

// Send XY coordinates using ESP-NOW
void sendxy(int x1a, int y1a) {
  char message[13];
  int teamNum = 26;
  sprintf(message,"%2d:%4d,%4d",teamNum,x1a,y1a);
  esp_now_send(staffcomm.peer_addr, (uint8_t *)message,13);
  Serial.println(message);
}

// Get the distance from Ultrasonic Sensor
void Ultrasound(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distanceCm = duration * SOUND_SPEED/2;
  distanceInch = distanceCm * CM_TO_INCH;
}

// Return the range of robot from wall
void ultrasound_four(){
  // Ultrasound Front Right
  Ultrasound(trigPinfr,echoPinfr);
  distanceCmfr = distanceCm;
  distanceInchfr = distanceInch;

  // Ultrasound Front Left
  Ultrasound(trigPinfl,echoPinfl);
  distanceCmfl = distanceCm;
  distanceInchfl = distanceInch;

  // Ultrasound Back Left
  Ultrasound(trigPinbl,echoPinbl);
  distanceCmbl = distanceCm;
  distanceInchbl = distanceInch;

  // Ultrasound Back Right
  Ultrasound(trigPinbr,echoPinbr);
  distanceCmbr = distanceCm;
  distanceInchbr = distanceInch;
}

// Subroutine for turning left
void turn_left(){
  ledcWrite(analog0, 0);
  ledcWrite(analog4, biit*d/100);
  ledcWrite(analog1, 0);
  ledcWrite(analog2, biit*d/100);
}

// Subroutine for turning right
void turn_right(){
  ledcWrite(analog0, biit*d/100);
  ledcWrite(analog4, 0);
  ledcWrite(analog1, biit*d/100);
  ledcWrite(analog2, 0);
}

// Subroutine for going straight
void go_straight(){
  ledcWrite(analog0, biit*d/100);
  ledcWrite(analog4, 0);
  ledcWrite(analog1, 0);
  ledcWrite(analog2, biit*d/100);
}

// Subroutine for turning ninety degree
void turn_right_ninety(){
  ledcWrite(analog0, biit*da/100);
  ledcWrite(analog4, 0);
  ledcWrite(analog1, 0);
  ledcWrite(analog2, 0);
}

// Subroutine for turning right slowly
void turn_right_slow(){
  ledcWrite(analog0, biit*k/100);
  ledcWrite(analog4, 0);
  ledcWrite(analog1, biit*k/100);
  ledcWrite(analog2, 0);
}

static long int ms = millis();

// Subroutine for wall following
void wall_following(){
  Ultrasound(trigPinfront, echoPinfront);  // Get distance from front ultrasonic sensor
  distancemmfront = distanceCm*10;
  
  if (distancemmfront>350){// Get distance of the robot from the side wall
     ultrasound_four();

      if (distanceCmfl>(12.5+5) && distanceCmbl<(11.5+5)){
        turn_left();
        delay(20);
        go_straight();
      }
      else if(distanceCmfl<(12.5+5) && distanceCmbl>(11.5+5)){
        turn_right();
        delay(15);
        go_straight();
      }
      else if(distanceCmfl<(12.5+5) && distanceCmbl>(11.5+5)){
        turn_right();
        delay(15);
        go_straight();
      }
      else if (distanceCmfl>(12.5+5) && distanceCmbl>(12.5+5) && distanceCmfl<60 && distanceCmbl<60 ){
        turn_left();
        delay(15);
        go_straight();
      }
      else if (distanceCmfl<(11.5+5) && distanceCmbl<(11.5+5)){
        turn_right();
        delay(15);
        go_straight();
      }
      else if(distanceCmfl<(11.5+5) && distanceCmbl>(11.5+5)){
        turn_right();
        delay(15);
        go_straight();
      }
      else{
        go_straight();
      }
    }
  else{
    turn_right_ninety();
    t = (1.723*60*1000)/da;
  }
}

// Subroutine for pushing police car
void police_car(){
  Ultrasound(trigPinfront, echoPinfront); // Get distance from front ultrasonic sensor
  distancemmfront = distanceCm*10;
  
  if (distancemmfront>0.0001){// Get distance of the robot from the side wall
     ultrasound_four();
      if (distanceCmfl>(40.5+5) && distanceCmbl<(39.5+5)){
        turn_left();
        delay(30);
        go_straight();
      }
      else if(distanceCmfl<(40.5+5) && distanceCmbl>(39.5+5)){
        turn_right();
        delay(30);
        go_straight();
      }
      else if(distanceCmfl<(40.5+5) && distanceCmbl>(39.5+5)){
        turn_right();
        delay(30);
        go_straight();
      }
      else if (distanceCmfl>(40.5+5) && distanceCmbl>(40.5+5) && distanceCmfl<600 && distanceCmbl<600 ){
        turn_left();
        delay(30);
        go_straight();
      }
      else if (distanceCmfl<(39.5+5) && distanceCmbl<(39.5+5)){
        turn_right();
        delay(30);
        go_straight();
      }
      else if(distanceCmfl<(39.5+5) && distanceCmbl>(39.5+5)){
        turn_right();
        delay(30);
        go_straight();
      }
      else{
        go_straight();
      }
    }
  else{
    turn_right_ninety();
    t = (1.723*60*1000)/da;
    delay(t/1.2);
  }
}

void IR_Circuit(){
  a = pulseIn(IN1,HIGH);
  b = pulseIn(IN1,LOW);
  c = pulseIn(IN2,HIGH);
  e = pulseIn(IN2,LOW);

  // Convert the time detected into frequency
  if((a+b)==0){
      d1=0;
    }
    else{
      d1=1000000*(1/(a+b));
    }
  if((c+e)==0){
      d2=0;
    }
    else{
      d2=1000000*(1/(c+e));
    }

   // Conditions for turning the robot in the direction of the received frequency
   // 23Hz frequency detected by right transistor.
   if (d1>15 && d1<50){
      if(d2<15 || d2>50){// 23Hz frequency not detected by left transistor.
          turn_right(); 
        }
      else{
          go_straight();  // 23Hz frequency detected by both transistors
        }
    }

  // 23Hz frequency detected by left transistor.
  else if(d2>15 && d2<50){
      if(d1<15 || d1>50){// 23Hz frequency not detected by right transistor.
          turn_left();
        }
      else{
          go_straight(); // 23Hz frequency detected by both transistors.
        }
    }

  // 700Hz frequency detected by right transistor.
  else if (d1>650 && d1<750){
      if(d2<650 || d2>750){// 700Hz frequency not detected by left transistor.
          turn_right();
        }
      else{
          go_straight();   // 700Hz frequency detected by both transistors.
        }
    }


  // 700Hz frequency detected by left transistor.
  else if(d2>650 && d2<750){
      if(d1<650 || d1>750){// 700Hz frequency not detected by right transistor.
          turn_left();
        }
      else{
          go_straight();   // 700Hz frequency detected by both transistors.
        }
    }


  // When desired frequency is not detected slowly turn in one direction (right) till we detect any frequency
  else{
    if(((d1<15 && d1>50) &&  (d2<15 && d2>50)) || ((d1<650 && d1>750) && (d2<650 && d2>750))){ 
        turn_right_slow();
      }
    else{
        go_straight();
      }
    }
}

void setup() {
  Serial.begin(115200);
 //Ultrasound
  pinMode(trigPinfr, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinfr, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinfl, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinfl, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinbr, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinbr, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinbl, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinbl, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinfront, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinfront, INPUT); // Sets the echoPin as an Input
  pinMode(pina, INPUT);
  pinMode(pinb, INPUT);
  //Motor
  pinMode(PWM_PIN1,OUTPUT);
  pinMode(PWM_PIN2,OUTPUT);
  pinMode(PWM_PIN3,OUTPUT);
  pinMode(PWM_PIN4,OUTPUT);

  ledcSetup(analog0, f, res_bit);
  ledcAttachPin(PWM_PIN1, analog0);
  ledcSetup(analog4, f, res_bit);
  ledcAttachPin(PWM_PIN2, analog4);
  ledcSetup(analog1, f, res_bit);
  ledcAttachPin(PWM_PIN3, analog1);
  ledcSetup(analog2, f, res_bit);
  ledcAttachPin(PWM_PIN4, analog2);

  WiFi.mode(WIFI_STA);
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  esp_now_init();
  esp_now_add_peer(&staffcomm );

  vive1.begin();
}

void loop(){
  // Read the state of switch
  a1 = digitalRead(pina);
  b1 = digitalRead(pinb);
  c1 = digitalRead(pinc);
  if(a1==HIGH){
    police_car(); 
  //  Send location using VIVE
  static long int ms = millis();
  static uint16_t x1,y1,x2,y2;

  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
        neopixelWrite(RGBLED,255,255,255);  // full white
        sendxy(x1,y1);
  }  
  if (vive1.status() == VIVE_RECEIVING) {
    x1 = vive1.xCoord();
    y1 = vive1.yCoord();
    neopixelWrite(RGBLED,0,x1/200,y1/200);  // blue to greenish
  }
  else {
    x1=0;
    y1=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
      }
    }
  }

  
  if(b1==HIGH){
    wall_following();  
  //  Send location using VIVE
  static long int ms = millis();
  static uint16_t x1,y1,x2,y2;

  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
        neopixelWrite(RGBLED,255,255,255);  // full white
        sendxy(x1,y1);
  }
  if (vive1.status() == VIVE_RECEIVING) {
    x1 = vive1.xCoord();
    y1 = vive1.yCoord();
    neopixelWrite(RGBLED,0,x1/200,y1/200);  // blue to greenish
  }
  else {
    x1=0;
    y1=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
      }
    } 
  }

  
  if(c1==HIGH){
    IR_Circuit();  
  //  Send location using VIVE
  static long int ms = millis();
  static uint16_t x1,y1,x2,y2;

  if (millis()-ms>1000/FREQ) {
    ms = millis();
    if (WiFi.status()==WL_CONNECTED)
        neopixelWrite(RGBLED,255,255,255);  // full white
        sendxy(x1,y1);
  }
  if (vive1.status() == VIVE_RECEIVING) {
    x1 = vive1.xCoord();
    y1 = vive1.yCoord();
    neopixelWrite(RGBLED,0,x1/200,y1/200);  // blue to greenish
  }
  else {
    x1=0;
    y1=0; 
    switch (vive1.sync(5)) {
      break;
      case VIVE_SYNC_ONLY: // missing sweep pulses (signal weak)
        neopixelWrite(RGBLED,64,32,0);  // yellowish
      break;
      default:
      case VIVE_NO_SIGNAL: // nothing detected     
        neopixelWrite(RGBLED,128,0,0);  // red
      }
    } 
  }
}

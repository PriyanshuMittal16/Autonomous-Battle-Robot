

//Motor
#include <WiFi.h> 
//#include "html510.h"
//#include "slider.h"

//Ultrasound Pins
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


//TOF Pins
//#define IRQ_PIN 2
//#define XSHUT_PIN 3
//#define SCL 11
//#define SDA 10

//Motor Pins
#define PWM_PIN1 21
#define PWM_PIN2 33
#define PWM_PIN3 19
#define PWM_PIN4 20
#define analog0 0
#define analog4 4
#define analog1 1
#define analog2 2

//Other Constants
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701
//Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

//Initialize Variables
//Ultrasound
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

//Motor
float d=80, res_bit=11, f=20, biit=2047,l,k,da = 75,t;

//Code Constants
float side_distance_1,side_distance_2,side_distance_3,side_distance_4,distance,threshold=300;

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
}

void loop() {

  
  Ultrasound(trigPinfront, echoPinfront); // Get distance from front ultrasonic sensor
  distancemmfront = distanceCm*10;
  
  if (distancemmfront>350){// Get distance of the robot from the side wall
     ultrasound_four();
     // Conditions for the robot to be in a threshold distance range from the wall
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
    delay(500);
  }
}

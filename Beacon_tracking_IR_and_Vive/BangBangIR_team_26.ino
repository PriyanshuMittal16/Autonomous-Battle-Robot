// Input pins from IR Circuit
#define IN1 8
#define IN2 3

// Pins to motor driver
#define analog0 0
#define analog4 4
#define analog1 1
#define analog2 2
#define PWM_PIN1 21
#define PWM_PIN2 33
#define PWM_PIN3 19
#define PWM_PIN4 20

// Declaring other used variables
float d=20, res_bit=11, f=20, biit=2047, k=20;
float d1,d2, a, b, c, e;

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

// Subroutine for turning right slowly
void turn_right_slow(){
  ledcWrite(analog0, biit*k/100);
  ledcWrite(analog4, 0);
  ledcWrite(analog1, biit*k/100);
  ledcWrite(analog2, 0);
}

// Subroutine for going straight
void go_straight(){
  ledcWrite(analog0, biit*d/100);
  ledcWrite(analog4, 0);
  ledcWrite(analog1, 0);
  ledcWrite(analog2, biit*d/100);
}

void setup(){
  Serial.begin(115200);
  pinMode(IN1,INPUT);
  pinMode(IN2,INPUT);
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

  // Read the high time and low time of the detected pulse by the phototransistor using PulseIn function
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
      if(d2<15 && d2>50){// 23Hz frequency not detected by left transistor.
          turn_right(); 
        }
      else{
          go_straight();  // 23Hz frequency detected by both transistors
        }
    }


  // 23Hz frequency detected by left transistor.
  else if(d2>15 && d2<50){
      if(d1<15 && d1>50){// 23Hz frequency not detected by right transistor.
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

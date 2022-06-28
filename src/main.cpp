#include <Arduino.h>
#include <stdio.h>
#include <StateMachine.h>
#include <Bounce2.h>
#include <string>
#include <FlashStorage.h>
#include "main.h"

#define F_R 0
#define R_L 1
#define sensorInt 6
#define sensorExt 7
#define speed 8
#define joystick A1
#define buttonMA 4 //manual or auto mode
#define buttonTC 5 //tension or center mode 
#define DEBUG 0


StateMachine machine = StateMachine();

//Variables
int sign = 0;
int nCentrage = 0;
int delta;
int moveDuration_ms;

unsigned long last_center = 0;
unsigned long last_out = 0;
unsigned long last_outLoop = 0;
unsigned long time2out = 0;
unsigned long time2center = 0;
unsigned long endOfMove = 0;
unsigned int bounceTime = 5;
unsigned int bounceTimeSensors = 100;

bool lastButton = false;

//FlashStorage
FlashStorage(delta_storage, int);
FlashStorage(moveDuration_ms_storage, int);

//States
State* S0 = machine.addState(&state0); //manual mode
State* S1 = machine.addState(&state1); //belt centered
State* S2 = machine.addState(&state2); //belt need to be centered

//Debounce
Bounce bounceMA = Bounce();
Bounce bounceTC = Bounce();
Bounce bounceInt = Bounce();
Bounce bounceExt = Bounce();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if(DEBUG){
    
    while(!Serial.available()){
      Serial.println("Press any key to start");
      yield();
    }
  }
  pinMode(F_R, OUTPUT);
  pinMode(R_L, OUTPUT);
  pinMode(speed, INPUT_PULLDOWN);

  stop();

  S0->addTransition(&transitionS0S1,S1); // S0 transition to S1
  S1->addTransition(&transitionS1S0,S0); // S1 transition to S0
  S1->addTransition(&transitionS1S2,S2); // S1 transition to S2
  S2->addTransition(&transitionS2S0,S0); // S2 transition to S0
  S2->addTransition(&transitionS2S1,S1); // S2 transition to S1

  machine.transitionTo(S0);

  bounceMA.attach(buttonMA, INPUT_PULLDOWN);
  bounceMA.interval(bounceTime);
  bounceTC.attach(buttonTC, INPUT_PULLDOWN);
  bounceTC.interval(bounceTime);
  bounceInt.attach(sensorInt, INPUT);
  bounceInt.interval(bounceTimeSensors);
  bounceExt.attach(sensorExt, INPUT);
  bounceExt.interval(bounceTimeSensors);

  if (!delta_storage.read() && !moveDuration_ms_storage.read()){ //s'il n'y a rien en mémoire, alors on utilise les valeurs par défaut et on les enregistre
    delta = int(UINT8_MAX / 4);
    moveDuration_ms = 500;
    delta_storage.write(delta);
    moveDuration_ms_storage.write(moveDuration_ms);
  }
  else{ //sinon on charge les valeurs stockées
    delta = delta_storage.read();
    moveDuration_ms = moveDuration_ms_storage.read();
  }
}

void loop() {
  // put your main code here, to run repeatedly
  bounceMA.update();
  bounceTC.update();
  bounceInt.update();
  bounceExt.update();

  /*if (digitalRead(buttonMA) != lastButton){
    lastButton = digitalRead(buttonMA);
    Serial.println("OK");
  }
  if (bounceMA.changed()){
    Serial.println(bounceMA.read());
  }*/

  machine.run();
  //delay(5);

  if(Serial.available() > 0){
    String data = Serial.readStringUntil(0x0D);
    switch (data[0]){
      case 'T':
        moveDuration_ms = atoi((data.substring(1)).c_str()); //convert the input (string) into an integer
        Serial.println("Vous avez config moveDuration = "+String(moveDuration_ms));
        moveDuration_ms_storage.write(moveDuration_ms);
        break;
      case 'D':
        delta = atoi((data.substring(1)).c_str()); //convert the input (string) into an integer
        Serial.println("Vous avez config delta = "+String(delta));
        if (delta > 127){
          delta = 127;
          Serial.println("La valeur indiquee est > 127, delta = 127");
        }
        delta_storage.write(delta);
        break;
      default:
        Serial.println("Pour config moveDuration a la valeur X tapez 'TX'");
        Serial.println("Pour config delta a la valeur X tapez 'DX'");
    }
  }
}

//----------STATES------------

void state0(){
  if (machine.executeOnce){
    //to do : reset variables de centrage
    last_out = last_center = 0;
    time2center = time2out = 0;
    nCentrage = 0;
    Serial.println("State 0 : MANUAL");
    stop();
  }
  int consigne = joystickRead();
  if (bounceMA.read()){
    if (bounceTC.read()){ //tension
      analogWrite(R_L, 127);
      analogWrite(F_R, consigne);
      //Serial.println("state0: Tension = "+String(consigne));
    }
    else if (!bounceTC.read()){ //centrage
      analogWrite(F_R, 127);
      analogWrite(R_L, consigne);
      //Serial.println("state0: Centrage = "+String(consigne));
    }
  }
}

void state1(){
  if (machine.executeOnce){
    Serial.println("State 1 : CENTERED");
    last_center = millis();
    if (last_out > 0){
      time2center = millis()-last_out;
    }
    move(-sign, delta, moveDuration_ms*nCentrage,"[state1] cancel"); // = cancel center
    if (time2out > 0 && time2center > 0){
      float ratio = 1.0*time2center/time2out;
      Serial.println("Ratio="+String(ratio)+", time2out="+String(time2out)+", time2center="+String(time2center));
      move(sign, delta, int(ratio*moveDuration_ms),"[state1] adjust"); // = adjust position
    }
  }
  if (millis() >= endOfMove){
    stop();
  }
}

void state2(){
  if (machine.executeOnce){
    nCentrage = 0;
    Serial.println("State 2 : NOT CENTERED...");
    last_out = last_outLoop = millis();
    time2out = millis()-last_center;
    if (!bounceInt.read() && !bounceExt.read()){
      sign = 1;
    }
    if (bounceInt.read() && bounceExt.read()){
      sign = -1;
    }
    move(sign, delta, moveDuration_ms, "state2: first");
    nCentrage++;
  }
  unsigned long now = millis();
  if (now-last_outLoop > 60000){
    nCentrage++;
    move(sign, delta, moveDuration_ms, "state2: centrage "+String(nCentrage));
    last_outLoop = millis();
  }
  if (millis() >= endOfMove){
    stop();
  }
}

//--------TRANSITIONS---------

bool transitionS0S1(){
  bool r = false;
  if (!bounceMA.read()){
    r = true;
    stop();
  }
  return r;
}

bool transitionS1S0(){
  bool r = false;
  if (bounceMA.read()){
    r = true;
    stop();
  }
  return r;
}

bool transitionS1S2(){
  bool r = false;
  if (digitalRead(speed) && (!bounceInt.read() || bounceExt.read())){
    r = true;
    stop();
  }
  return r;
}

bool transitionS2S0(){
  bool r = false;
  if (bounceMA.read()){
    r = true;
    stop();
  }
  return r;
}

bool transitionS2S1(){
  bool r = false;
  if (!digitalRead(speed) || (bounceInt.read() && !bounceExt.read())){
    r = true;
    stop();
  }
  return r;
}

//---------FUNCTIONS----------

int joystickRead(){
  int value = 0;
  int iter = 10;
  int r;
  int temp;
  analogReadResolution(12);
  for (int i = 0; i < iter; i++){
      value += analogRead(joystick);
  }
  value /= iter;
  if (DEBUG){
    int min_joy = pow(2,12)/2; 
    int max_joy = pow(2,12)/2;
    if (value > max_joy){
      Serial.println("Max = "+String(value));
      max_joy = value;
    }
    if (value < min_joy){
      Serial.println("Min = "+String(value));
      min_joy = value;
    }
  }
  //value = int(value/36)*36;
  temp = map(value, 287, 3787, 0, 255);
  if (temp > 121 && temp < 133) r=127;
  else r = temp;
  return r;
}

void move(int sign_, int delta_, unsigned int time_, String contexte=""){
  int consigne = 127+sign_*delta_;
  Serial.println("Move "+contexte+": sign " +String(sign_)+", consigne "+String(consigne)+", duration "+String(time_));
  analogWrite(R_L, consigne);
  endOfMove = time_+millis();
}

void stop(){
  analogWrite(R_L, 127);
  analogWrite(F_R, 127);
}
#include <NewPing.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>

#define TRIGGER_PIN  8
#define ECHO_PIN     9
#define MAX_DISTANCE 255 // Maximum distance we want to measure (in centimeters).
#define MIN_DISTANCE 5
#define BUTTON_PIN   7
#define LED_PIN      11

int address = 0, //eeprom address to read/write
    distance, //distance gathered from ping
    prevDistance, //previous distance to compare changes with
    saveDistance, //saved distance to compare to
    idleTime = 30, //time before going idle
    buttonState, //for holding current state of the button
    prevButton, //previous button state to compare if change
    sonarPing = 20, //how many pings for sonar(higher=longer/better)
    delayCount = 29, //how long to delay between sonar query for distance
    learnDelay = 5, //how long to hold button for in seconds
    learnTime = 10, //time to spend learning in seconds
    degOfError = 10, //average degree of error when trying to learn to prevent too bad of values from throwing off data set
    learnAvg = 0, 
    brightness = 0; //brightness scale (0-255) for controlling LED
unsigned long time; //hold value of time for compare
bool dim = false;
String logs = "";

enum STATE {
  RUN,
  IDLE,
  LEARN
};

enum STATE currentState = RUN;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  //load from eeprom here
  saveDistance = EEPROM.read(address);
  time = millis();
  if(Serial){
    Serial.print(millis());
    Serial.print(": ");
    Serial.print("Learned Distance: ");
    Serial.println(saveDistance);
  }
}

void loop() {
  buttonState = digitalRead(BUTTON_PIN);
  if(buttonState != HIGH){
    delay(delayCount);                    // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    distance = sonar.ping_median(sonarPing);
    distance = sonar.convert_cm(distance);
  }
  switch(currentState){
    case RUN:
      //running state
      if(buttonState == HIGH){
        //button pressed
        int learn = millis() - time;
        if(prevButton == LOW){
          time = millis();
          digitalWrite(LED_PIN, LOW);
        }else{
          logs = logs + " | Button Down : " + String(learn);
          delay(1);
        }
        if(abs(learn) >= (learnDelay * 1000)){
          currentState = LEARN;
          digitalWrite(LED_PIN, HIGH);
          logs = logs + " | Moving Run > Learn";
          delay(500);
          time = millis();
        }
      }else{
        //check distance
        if(prevButton == HIGH){
          digitalWrite(LED_PIN, HIGH);
          time = millis();
        }
        if(distance != 0){
          logs = logs + " | Distance: " + String(distance) + "cm";
          if(distance <= saveDistance){
            //do stuff if within range
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            logs = logs + " | Within Range!";
          }else{
            digitalWrite(LED_PIN, HIGH);
          }
        }
        //check if nothing is happening and move to idle, 
        if(prevDistance >= (distance - 1) && prevDistance <= (distance + 1)){
          int idle = millis() - time;
          logs = logs + " | Idle Count: " + String(idle);
          if(abs(idle) >= idleTime * 1000){
            currentState = IDLE;
            logs = logs + " | Moving Run > Idle";
          }
          delay(1);
        }else{
          time = millis();
        }
      }
      break;
    case LEARN:
      //learning state
      if(buttonState == LOW){
        if(abs(millis() - time) <= (learnTime * 1000)){
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          if(abs(prevDistance - distance) <= degOfError){
            learnAvg += distance;
            if(Serial){
              logs = logs + " | Learning Distance: " + String(distance) + "cm";
            }
          }
        }else{
          //save to eeprom here
          learnAvg = learnAvg / learnTime;
          if(learnAvg > MAX_DISTANCE){
            learnAvg = MAX_DISTANCE;
          }else if(learnAvg < MIN_DISTANCE){
            learnAvg = MIN_DISTANCE;
          }
          EEPROM.update(address, learnAvg);
          saveDistance = learnAvg;
          learnAvg = 0;
          currentState = RUN;
          digitalWrite(LED_PIN, HIGH);
          if(Serial){
            logs = logs + " | Learned Distance (" + String(saveDistance) + ") Moving Learn > Run";
          }
        }
      }else{
        if(Serial){
          currentState = RUN;
          logs = logs + " | Canceled! Moving Learn > Run";
        }
      }
      break;
    case IDLE:
      //idle state
      //disable any lights
      logs = logs + " | Idling!";
      do{
        analogWrite(LED_PIN, brightness);
        delay(6);
        if(dim){
          brightness -= 2;
        }else{
          brightness += 2;
        }
        if(brightness >= 255){
          brightness = 255;
          dim = true;
        }else if(brightness <= 0){
          brightness = 0;
          dim = false;
          digitalWrite(LED_PIN, LOW);
        }
        if(buttonState == HIGH){
          brightness = 0;
        }
      }while(brightness > 0);
      if(buttonState == HIGH){
        digitalWrite(LED_PIN, LOW);
        currentState = RUN;
        time = millis();
        logs = logs + " | Moving Idle > Run";
      }
      if(prevDistance <= (distance - 2) || prevDistance >= (distance + 2)){
        digitalWrite(LED_PIN, HIGH);
        currentState = RUN;
        time = millis();
        logs = logs + " | Moving Idle > Run";
      }
      break;
    default:
      currentState = RUN;
  }
  prevDistance = distance;
  prevButton = buttonState;
  if(Serial){
    Serial.print(millis());
    Serial.println(logs);
    logs = "";
  }
}

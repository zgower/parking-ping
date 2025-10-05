#include <NewPing.h>
#include <Arduino.h>
#include <EEPROM.h>

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
    idleCount = 0, //counter for moving to idle state
    buttonCount = 0, //counter for how long button is held down
    buttonState, //for holding current state of the button
    prevButton, //previous button state to compare if change
    sonarPing = 20, //how many pings
    maxCount = 30, //how many queries for learn
    delayCount = 29, //how long to delay between query
    learnDelay = 1500, //how long to hold button for 
    degOfError = 10,
    learnAvg = 0, //
    brightness = 0; //brightness scale (0-255) for controlling LED

bool dim = false;

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
  if(Serial){
    Serial.print(millis());
    Serial.print(": ");
    Serial.print("Learned Distance: ");
    Serial.println(saveDistance);
    learnDelay = learnDelay / 2;
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
        //start count
        if(prevButton == LOW){
          digitalWrite(LED_PIN, LOW);
        }else{
          buttonCount++;
          idleCount = 0;
          if(Serial){
            Serial.print(millis());
            Serial.print(": ");
            Serial.print("Button Down: ");
            Serial.println(buttonCount);
          }
          delay(5);
        }
        if(buttonCount >= learnDelay){
          buttonCount = 0;
          idleCount = 0;
          currentState = LEARN;
          digitalWrite(LED_PIN, HIGH);
          if(Serial){
            Serial.print(millis());
            Serial.print(": ");
            Serial.println("Moving to Learn from Run");
          }
          delay(1000);
        }
      }else{
        //check distance
        if(prevButton == HIGH){
          buttonCount = 0;
          digitalWrite(LED_PIN, HIGH);
        }
        if(distance != 0){
          if(distance <= saveDistance){
            //do stuff if within range
            if(Serial){
              Serial.print(millis());
              Serial.print(": ");
              Serial.print("Distance: ");
              Serial.print(distance);
              Serial.print("cm | ");
              Serial.println("Within Range!");
            }
          }else{
            if(Serial){
              Serial.print(millis());
              Serial.print(": ");
              Serial.print("Distance: ");
              Serial.print(distance);
              Serial.println("cm");
            }
          }
        }
        //check if nothing is happening and move to idle, 
        if(prevDistance >= (distance - 1) && prevDistance <= (distance + 1)){
          idleCount++;
          if(Serial){
            Serial.print(millis());
            Serial.print(": ");
            Serial.print("Idle Counting: ");
            Serial.println(idleCount);
          }
          delay(5);
        }else{
          idleCount = 0;
        }
        if(idleCount >= 100){
          idleCount = 0;
          currentState = IDLE;
          if(Serial){
            Serial.print(millis());
            Serial.print(": ");
            Serial.println("Moving to Idle from Run");
          }
        }
      }
      break;
    case LEARN:
      //learning state
      if(buttonState == LOW){
        if(buttonCount <= maxCount){
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          if(abs(prevDistance - distance) <= degOfError){
            learnAvg += distance;
            if(Serial){
              Serial.print(millis());
              Serial.print(": ");
              Serial.print("Learn Distance: ");
              Serial.print(distance);
              Serial.println("cm");
            }
          }
          buttonCount++;
        }else{
          //save to eeprom here
          learnAvg = learnAvg / maxCount;
          if(learnAvg > MAX_DISTANCE){
            learnAvg = MAX_DISTANCE;
          }else if(learnAvg < MIN_DISTANCE){
            learnAvg = MIN_DISTANCE;
          }
          EEPROM.update(address, learnAvg);
          saveDistance = learnAvg;
          learnAvg = 0;
          buttonCount = 0;
          currentState = RUN;
          digitalWrite(LED_PIN, HIGH);
          if(Serial){
            Serial.print(millis());
            Serial.print(": ");
            Serial.print("Learned Distance: ");
            Serial.println(saveDistance);
            Serial.print(millis());
            Serial.print(": ");
            Serial.println("Moving to Run from Learn");
          }
        }
      }else{
        if(Serial){
          currentState = RUN;
          digitalWrite(LED_PIN, HIGH);
          Serial.println("Cancel Learn");
          Serial.println("Moving to Run from Learn");
        }
      }
      break;
    case IDLE:
      //idle state
      //disable any lights
      int bright = 2;
      do{
        analogWrite(LED_PIN, brightness);
        delay(6);
        if(dim){
          brightness -= bright;
        }else{
          brightness += bright;
        }
        if(brightness >= 255){
          brightness = 255;
          dim = true;
        }else if(brightness <= 0){
          brightness = 0;
          dim = false;
          digitalWrite(LED_PIN, LOW);
        }
      }while(brightness > 0);
      if(buttonState == HIGH){
        digitalWrite(LED_PIN, LOW);
        currentState = RUN;
        if(Serial){
          Serial.print(millis());
          Serial.print(": ");
          Serial.println("Moving to Run from Idle");
        }
      }
      if(prevDistance <= (distance - 2) || prevDistance >= (distance + 2)){
        digitalWrite(LED_PIN, HIGH);
        currentState = RUN;
        if(Serial){
          Serial.print(millis());
          Serial.print(": ");
          Serial.println("Moving to Run from Idle");
        }
      }
      if(Serial){
        Serial.print(millis());
        Serial.print(": ");
        Serial.println("Idling");
      }
      break;
    default:
      currentState = RUN;
  }
  prevDistance = distance;
  prevButton = buttonState;
}

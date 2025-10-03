#include <NewPing.h>
#include <Arduino.h>
#include <EEPROM.h>

#define TRIGGER_PIN  8
#define ECHO_PIN     9
#define MAX_DISTANCE 255 // Maximum distance we want to measure (in centimeters).
#define BUTTON_PIN   7
#define LED_PIN      11

int address = 0, //eeprom address to read/write
    distance, 
    lastDistance,
    saveDistance, //saved distance to compare to
    minDistance = 5, //minimum distance from device
    distanceCount = 0,
    buttonState, 
    prevButton, 
    sonarPing = 20,
    btnCount = 0, //how long button is down
    maxCount = 30, //how many queries for learn
    delayCount = 29, //how long to delay between query
    learnDelay = 1500, //how long to hold button for 
    degOfError = 10,
    learnAvg = 0,
    brightness = 0;

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
  Serial.print(millis());
  Serial.print(": ");
  Serial.print("Learned Distance: ");
  Serial.println(saveDistance);
}

void loop() {
  buttonState = digitalRead(BUTTON_PIN);
  if(buttonState != HIGH){
    delay(delayCount);                    // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
    distance = sonar.ping_median(sonarPing);
    distance = sonar.convert_cm(distance);
  }
  switch(currentState){
    case 0:
      //running state
      if(buttonState == HIGH){
        //start count
        if(prevButton == LOW){
          digitalWrite(LED_PIN, LOW);
        }else{
          btnCount++;
          delay(5);
        }
        if(btnCount >= learnDelay){
          btnCount = 0;
          digitalWrite(LED_PIN, HIGH);
          Serial.print(millis());
          Serial.print(": ");
          Serial.println("Moving to Learn from Run");
          currentState = LEARN;
          delay(1000);
        }
      }else{
        //check distance
        if(prevButton == HIGH){
          btnCount = 0;
          digitalWrite(LED_PIN, HIGH);
        }
        if(distance != 0){
          if(distance <= saveDistance){
            //do stuff if within range
            Serial.println("Within Range!");
          }
        }
        if(lastDistance >= (distance - 1) && lastDistance <= (distance + 1)){
          Serial.print("Same Same ");
          Serial.print(distanceCount);
          Serial.print(" : ");
          Serial.print(distance);
          Serial.println("cm");
          distanceCount++;
          delay(5);
        }else{
          distanceCount = 0;
        }
        if(distanceCount >= 100){
          distanceCount = 0;
          Serial.print(millis());
          Serial.print(": ");
          Serial.println("Moving to Idle from Run");
          currentState = IDLE;
        }
      }
      break;
    case 1:
      //idle state
      //disable any lights
      int bright = 1;
      do{
        analogWrite(LED_PIN, brightness);
        delay(8);
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
      
      if(lastDistance <= (distance - 1) || lastDistance >= (distance + 1)){
        digitalWrite(LED_PIN, HIGH);
        Serial.print(millis());
        Serial.print(": ");
        Serial.println("Moving to Run from Idle");
        currentState = RUN;
      }
      break;
    case 2:
      //learning state
      if(prevButton != HIGH && buttonState != HIGH){
        if(btnCount <= maxCount){
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
          if(abs(lastDistance - distance) <= degOfError){
            learnAvg += distance;
          }
          btnCount++;
        }else{
          btnCount = 0;
          //save to eeprom here
          learnAvg = learnAvg / maxCount;
          if(learnAvg > 255){
            learnAvg = 255;
          }else if(learnAvg < minDistance){
            learnAvg = minDistance;
          }
          EEPROM.update(address, learnAvg);
          saveDistance = learnAvg;
          learnAvg = 0;
          digitalWrite(LED_PIN, HIGH);
          Serial.print("Learned Distance: ");
          Serial.println(saveDistance);
          Serial.print(millis());
          Serial.print(": ");
          Serial.println("Moving to Run from Learn");
          currentState = RUN;
        }
      }
      break;
    default:
      currentState = IDLE;
  }
  lastDistance = distance;
  prevButton = buttonState;
}

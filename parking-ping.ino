/*
Title: parking-ping
Author: zgower
*/

#include <NewPing.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include <protothreads.h>
#include <FastLED.h>

#define BUTTON_PIN    15 
#define TRIGGER_PIN   5
#define ECHO_PIN      6
#define STATUS_PIN    4
#define RST_PIN       11
#define LED_PIN       14
#define NUM_LED       138 //total 138

const int max_distance = 600; //maximum distance for sensor(cm)
const int min_distance = 5; //minimum distance for range detection(cm)
CRGB leds[NUM_LED];

const int address = 0, //address in eeprom to store distance information
          median = 50, //extra distance to issue getting close warning
          savedDistance = EEPROM.read(address), //saved distance, set during setup and updated when peforming LEARN
          medianDistance = savedDistance + median, //median distance, aprox 50cm+savedDistance
          doeLearn = 5, //Degree Of Error for when learning, prevent wild values outside of range from getting added
          doeIdle = 1; //Degree Of Error for when checking if nothing is happening, smaller values will make it trigger more often, too high and it will not trigger enough
                       //when idle, will add one(1) to value to increase any diviation that may occur
const unsigned int gotoLearn = 5, //time it will take to hold button to goto learn
                   gotoIdle = 45, //time it will take to goto idle
                   timeLearn = 15; //time it will spend learning distance before save
NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);
volatile unsigned int distance; //distance learned by HC-SR04 ultrasonic sensor
unsigned int prevDistance; //previous distance to track movement and direction
unsigned long prevTime, flashMillis; //time used for counting
int runCount = 0, //counts how many times entered run state
    idleCount = 0, //counts how many times entered idle state
    learnCount = 0, //counts how many times entered learn state
    buttonState, //current state of button
    prevButtonState, //previous state of button
    countLearn = 0, //counts how many times something gets added to avgLearn
    avgLearn = 0; //adds all distance values during LEARN and is divided by countLearn to get average
String logs = ""; //string to hold all log information and display at end
bool rst = false;
unsigned int* pdistance = &distance;
enum STATE {
  RUN, //checks distance
  IDLE, //lower actions, meant to disable any lights
  LEARN //for learning new saved distance
};
enum RANGE {
  GREEN, 
  YELLOW,
  RED,
  DANGER
};
enum STATE currentState; //stores current state
enum STATE previousState;
enum RANGE currentRange; //stores current range state
enum RANGE previousRange;

void logger(String msg){
  logs += (Serial) ? msg + " " : "";
}

pt ptSonar;
int sonarThread(struct pt* pt){
  PT_BEGIN(pt);
  for(;;){
    FastLED.clear(true); // this causes strobing effect everytime sensor is ran. Can be removed if power for sensor and LED is seperated
    delay(50);
    distance = sonar.ping_median();
    distance = sonar.convert_cm(distance);
    PT_YIELD(pt);
  }
  PT_END(pt);
}

pt ptButton;
int buttonThread(struct pt* pt){
  PT_BEGIN(pt);
  for(;;){
    prevButtonState = buttonState;
    buttonState = digitalRead(BUTTON_PIN);
    PT_YIELD(pt);
  }
  PT_END(pt);
}

pt ptStatus;
int statusThread(struct pt* pt){
  PT_BEGIN(pt);
  for(;;){
    switch(currentState){
      case RUN:
        if(buttonState == HIGH){
          digitalWrite(STATUS_PIN, LOW);
        }else{
          digitalWrite(STATUS_PIN, HIGH);
        }
        break;
      case LEARN:
        if(buttonState == LOW){
          digitalWrite(STATUS_PIN, !digitalRead(STATUS_PIN));
        }
        break;
      case IDLE:
        digitalWrite(STATUS_PIN, LOW);
        break;
      default:
        //digitalWrite(STATUS_PIN, HIGH);
        break;
    }
    PT_YIELD(pt);
  }
  PT_END(pt);
}

pt ptLED;
int ledThread(struct pt* pt){
  PT_BEGIN(pt);
  for(;;){
    CRGB color = CRGB::White;
    if(currentState == RUN){
      switch(currentRange){
        case DANGER:
          //lights are red flashing
          if(abs(millis() - flashMillis) >= 500){
            flashMillis = millis();
            if(color == CRGB::Red){
              color == CRGB(255, 255, 255);
            }else{
              color == CRGB::Red;
            }
          }
          break;
        case RED:
          //lights are red
          color = CRGB::Red;
          break;
        case GREEN:
          //lights are green
          color = CRGB::Green;
          break;
        case YELLOW:
          //lights are yellow/orange
          color = CRGB::Yellow;
          break;
        
        
        default:
          //lights are off
          break;
      }
      fill_solid(leds, NUM_LED, color);
      FastLED.show();
    }else{
      //turn off LED
      FastLED.clear();
    }
    PT_YIELD(pt);
  }
  PT_END(pt);
}

void reset(){
  Serial.println("RESETING");
  delay(1000);
  digitalWrite(RST_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(RST_PIN, HIGH);
}



void setup() {
  Serial.begin(9600);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LED);
  FastLED.clear();
  PT_INIT(&ptSonar);
  PT_INIT(&ptButton);
  PT_INIT(&ptStatus);
  PT_INIT(&ptLED);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  pinMode(RST_PIN, OUTPUT);
  prevTime = millis();
  flashMillis = millis();
  logger("| Saved Distance (" + String(savedDistance) + ") | Moving RUN");
  runCount++;
  currentState = RUN;
}

void loop() {
  PT_SCHEDULE(sonarThread(&ptSonar));
  PT_SCHEDULE(buttonThread(&ptButton));
  PT_SCHEDULE(statusThread(&ptStatus));
  PT_SCHEDULE(ledThread(&ptLED));
  unsigned int d = *pdistance;
  switch(currentState){
    case RUN:
      if(buttonState == HIGH){
        if(prevButtonState == LOW){
          prevTime = millis();
        }
        unsigned long learn = abs(millis() - prevTime);
        if(learn >= ((long)gotoLearn * 1000)){
          learnCount++;
          currentState = LEARN;
        }else{
          logger("| Button Held (" + String((int)(learn * 0.001)) + ")");
        }
      }else{
        //detect range
        if(prevButtonState == HIGH){
          prevTime = millis();
        }
        if(d != 0){
          if(d > medianDistance){
            currentRange = RED;
          }else if(d > savedDistance){
            currentRange = YELLOW;
          }else {
            currentRange = GREEN;
            if(d <= min_distance){
              currentRange = DANGER;
            }
          }
        }else{
          logger("| No Range");
        }
        //detect IDLE
        if(prevDistance >= (d - doeIdle) && prevDistance <= (d + doeIdle)){
          unsigned long idle = (millis() - prevTime); 
          logger("| Idle Count (" + String((int)(idle * 0.001)) + ")");
          if(idle >= ((long)gotoIdle * 1000)){
            idleCount++;
            currentRange = -1;
            currentState = IDLE;
          }
        }else{
          prevTime = millis();
        }
      }
      break;
    case LEARN:
      if(buttonState == LOW && prevButtonState == HIGH){
        prevTime = millis();
      }else if(buttonState == LOW && prevButtonState == LOW){
        unsigned long learn = abs(millis() - prevTime);
        if(learn <= ((long)timeLearn*1000)){
          if(abs(prevDistance - d) <= doeLearn){
            avgLearn += d;
            countLearn++;
            logger("| Learning (" + String(avgLearn / countLearn) + ")");
          }
        }else{
          avgLearn = avgLearn / countLearn;
          if(avgLearn > max_distance){
            avgLearn = max_distance;
          }else if(avgLearn < min_distance){
            avgLearn = min_distance;
          }
          if(savedDistance != avgLearn){
            EEPROM.update(address, avgLearn);
            logger("| Learned Distance (" + String(savedDistance) + ")");
            logger("| Reseting");
            rst = true;
          }
          avgLearn = 0;
          countLearn = 0;
          runCount++;
          currentState = RUN;
        }
      }else if(buttonState == HIGH && prevButtonState == LOW){
        runCount++;
        currentState = RUN;
      }
      break;
    case IDLE:
      bool exitIdle = false;
      if(prevDistance < (d - (doeIdle+1)) || prevDistance > (d + (doeIdle+1))){
        exitIdle = true;
      }
      if(buttonState == HIGH && prevButtonState == LOW){
        exitIdle = true;
      }
      if(exitIdle){
        runCount++;
        currentState = RUN;
        break;
      }
      break;
    default:
      currentState = RUN;
  }
  if(Serial){
    Serial.print(millis());
    Serial.print("| State: ");
    switch(currentState){
      case RUN:
        Serial.print("running  (" + String(runCount) +") ");
        break;
      case IDLE:
        Serial.print("idling   (" + String(idleCount) + ") ");
        break;
      case LEARN:
        Serial.print("learning (" + String(learnCount) + ") ");
        break;
      default:
        Serial.print("no state");
    }
    Serial.print("| Range: ");
    switch(currentRange){
      case GREEN:
        Serial.print("within range ");
        break;
      case YELLOW:
        Serial.print("almost range ");
        break;
      case RED:
        Serial.print("out of range ");
        break;
      case DANGER:
        Serial.print("move back now");
        break;
      default:
        Serial.print("no range");
    }
    Serial.print("(" + String(d) + "/" + String(savedDistance) + "cm) ");
    if(previousState != currentState){
      Serial.print("| State Change " + String(previousState) + " > " + String(currentState) + " ");
    }
    if(previousRange != currentRange && currentState != IDLE){
      Serial.print("| Range Change " + String(previousRange) + " > " + String(currentRange) + " ");
    }
    Serial.println(logs);
  }
  if(rst){
    reset();
  }
  prevDistance = d;
  logs = "";
  prevTime = (currentState == IDLE) ? millis() : prevTime;
  previousState = currentState;
  previousRange = currentRange;
  delay(1);
}

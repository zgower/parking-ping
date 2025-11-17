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
          median = 65, //extra distance to issue getting close warning
          sonarDelay = 29, //dealy for how often to run the ultrasonic sensor
          sonarResolution = 6, //ultrasonic resolution, how often it sends out a ping
          savedDistance = EEPROM.read(address), //saved distance, set during setup and updated when peforming LEARN
          doeLearn = 5, //Degree Of Error for when learning, prevent wild values outside of range from getting added
          doeIdle = 1; //Degree Of Error for when checking if nothing is happening, smaller values will make it trigger more often, too high and it will not trigger enough
                       //when idle, will add one(1) to value to increase any diviation that may occur
const unsigned int gotoLearn = 5, //time it will take to hold button to goto learn
                   gotoIdle = 45, //time it will take to goto idle
                   timeLearn = 15; //time it will spend learning distance before save
NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);
volatile unsigned int distance; //distance learned by HC-SR04 ultrasonic sensor
unsigned int prevDistance; //previous distance to track movement and direction
unsigned long prevTime, sonarMillis, dangerMillis; //time used for counting
int runCount = 0, //counts how many times entered run state
    idleCount = 0, //counts how many times entered idle state
    learnCount = 0, //counts how many times entered learn state
    buttonState, //current state of button
    prevButtonState, //previous state of button
    countLearn = 0, //counts how many times something gets added to avgLearn
    avgLearn = 0; //adds all distance values during LEARN and is divided by countLearn to get average
bool rst = false,
     danger = true;
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
enum RANGE currentRange; //stores current range state

pt ptSonar;
int sonarThread(struct pt* pt){
  PT_BEGIN(pt);
  for(;;){
    long someDelay = sonarDelay;
    if(currentState == IDLE){
      someDelay = sonarDelay * 10;
    }
    if((millis() - sonarMillis) >= someDelay){
      sonarMillis = millis();
      distance = sonar.ping_median(sonarResolution);
      distance = sonar.convert_cm(distance);
    }
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
        digitalWrite(STATUS_PIN, LOW);
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
            if(danger){
              color = CRGB::White;
            }else{
              color = CRGB::Red;
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
          color = CRGB::Orange;
          break;
      }
      fill_solid(leds, NUM_LED, color);
      FastLED.show();
    }else{
      //turn off LED
      FastLED.clear(true);
    }
    PT_YIELD(pt);
  }
  PT_END(pt);
}

void reset(){
  Serial.print("RESETING");
  delay(1000);
  digitalWrite(RST_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(RST_PIN, HIGH);
  while(true){
    Serial.print(".");
  }
}

String logger(String* log, String msg){
  *log += (Serial) ? msg + " " : "";
  return *log;
}

void setup() {
  Serial.begin(9600);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LED);
  FastLED.clear(true);
  PT_INIT(&ptSonar);
  PT_INIT(&ptButton);
  PT_INIT(&ptStatus);
  PT_INIT(&ptLED);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  pinMode(RST_PIN, OUTPUT);
  prevTime = millis();
  sonarMillis = millis();
  dangerMillis = millis();
  Serial.println(String(millis()) + " | Saved Distance (" + String(savedDistance) + ")");
  runCount++;
  currentState = RUN;
}

void loop() {
  unsigned long delta = millis();
  enum STATE previousState = currentState;
  enum RANGE previousRange = currentRange;
  PT_SCHEDULE(sonarThread(&ptSonar));
  PT_SCHEDULE(buttonThread(&ptButton));
  PT_SCHEDULE(statusThread(&ptStatus));
  PT_SCHEDULE(ledThread(&ptLED));
  unsigned int d = *pdistance;
  String log = "";
  logger(&log, "| State:");
  switch(currentState){
    case RUN:
      logger(&log, "running (" + String(runCount) +")");
      if(buttonState == HIGH){
        if(prevButtonState == LOW){
          prevTime = millis();
        }
        unsigned long learn = (millis() - prevTime);
        if(learn >= ((long)gotoLearn * 1000)){
          learnCount++;
          currentState = LEARN;
        }else{
          logger(&log, "| Button Held (" + String((int)(learn * 0.001)) + ")");
        }
      }else{
        //detect range
        if(prevButtonState == HIGH){
          prevTime = millis();
        }
        logger(&log, "| Range:");
        if(d != 0){
          if(d > (savedDistance + median)){
            currentRange = RED;
            logger(&log, "out of range");
          }else if(d > savedDistance){
            currentRange = YELLOW;
            logger(&log, "almost range");
          }else {
            currentRange = GREEN;
            if(d < min_distance){
              if((millis() - dangerMillis) >= 500){
                dangerMillis = millis();
                danger = !danger;
              }
              currentRange = DANGER;
              logger(&log, "move back now");
            }else{
              logger(&log, "within range");
            }
          }
          logger(&log, "(" + String(d) + "/" + String(savedDistance) + "cm)");
        }else{
          currentRange = RED;
          logger(&log, "| No Range");
        }
        //detect IDLE
        if((prevDistance >= (d - doeIdle) && prevDistance <= (d + doeIdle)) || d == 0){
          unsigned long idle = (millis() - prevTime); 
          logger(&log, "| Idle Count (" + String((int)(idle * 0.001)) + ")");
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
      logger(&log, "learning (" + String(learnCount) + ")");
      if(buttonState == LOW && prevButtonState == HIGH){
        prevTime = millis();
      }else if(buttonState == LOW && prevButtonState == LOW){
        unsigned long learn = (millis() - prevTime);
        if(learn <= ((long)timeLearn*1000)){
          if(abs(prevDistance - d) <= doeLearn){
            avgLearn += d;
            countLearn++;
            logger(&log, "| Learning (" + String(avgLearn / countLearn) + ")");
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
            logger(&log, "| Learned Distance (" + String(savedDistance) + ")");
            logger(&log, "| Reseting");
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
      logger(&log, "idling (" + String(idleCount) + ")");
      bool exitIdle = false;
      if(prevDistance < (d - (doeIdle+1)) || prevDistance > (d + (doeIdle+1))){
        exitIdle = true;
      }
      if(buttonState == HIGH && prevButtonState == LOW){
        exitIdle = true;
      }
      if(exitIdle && d != 0){
        runCount++;
        currentState = RUN;
        break;
      }
      break;
    default:
      logger(&log, "no state");
      currentState = RUN;
  }
  if(previousState != currentState){
    logger(&log, "| State Change " + String(previousState) + " > " + String(currentState));
  }
  if(previousRange != currentRange && currentState != IDLE){
    logger(&log, "| Range Change " + String(previousRange) + " > " + String(currentRange));
  }
  prevDistance = d;
  prevTime = (currentState == IDLE) ? millis() : prevTime;
  delta = millis() - delta;
  if(Serial){
    Serial.print(String(millis()) + " (" + String(delta) + ")");
    Serial.println(log);
  }
  if(rst){
    reset();
  }
}

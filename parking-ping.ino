#include <NewPing.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include <protothreads.h>

#define BUTTON_PIN    8
#define TRIGGER_PIN   9
#define ECHO_PIN      10
#define LED_PIN       11

const int max_distance = 600; //maximum distance for sensor(cm)
const int min_distance = 5; //minimum distance for range detection(cm)

const int address = 0, //address in eeprom to store distance information
          median = 50, //extra distance to issue getting close warning
          doeLearn = 5, //Degree Of Error for when learning, prevent wild values outside of range from getting added
          doeIdle = 1; //Degree Of Error for when checking if nothing is happening, smaller values will make it trigger more often, too high and it will not trigger enough
                       //when idle, will add one(1) to value to increase any diviation that may occur
const unsigned int gotoLearn = 5, //time it will take to hold button to goto learn
                   gotoIdle = 45, //time it will take to goto idle
                   timeLearn = 15; //time it will spend learning distance before save
NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);
volatile unsigned int distance; //distance learned by HC-SR04 ultrasonic sensor
unsigned int prevDistance; //previous distance to track movement and direction
unsigned long prevTime; //time used for counting
int savedDistance, //saved distance, set during setup and updated when peforming LEARN
    medianDistance, //median distance, aprox 50cm+savedDistance
    runCount = 0, //counts how many times entered run state
    idleCount = 0, //counts how many times entered idle state
    learnCount = 0, //counts how many times entered learn state
    buttonState, //current state of button
    prevButtonState, //previous state of button
    countLearn = 0, //counts how many times something gets added to avgLearn
    avgLearn = 0; //adds all distance values during LEARN and is divided by countLearn to get average
String logs = ""; //string to hold all log information and display at end
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
    delay(29);
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
          digitalWrite(LED_PIN, LOW);
        }else{
          digitalWrite(LED_PIN, HIGH);
        }
        break;
      case LEARN:
        if(buttonState == LOW){
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
        break;
      case IDLE:
        digitalWrite(LED_PIN, LOW);
        break;
      default:
        //digitalWrite(LED_PIN, HIGH);
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
    if(currentState == RUN){
      switch(currentRange){
        case GREEN:
          //lights are green
          break;
        case YELLOW:
          //lights are yellow/orange
          break;
        case RED:
          //lights are red
          break;
        case DANGER:
          //lights are red flashing
          break;
        default:
          //lights are off
          break;
      }
    }else{
      //turn off LED
    }
    PT_YIELD(pt);
  }
  PT_END(pt);
}

void setup() {
  Serial.begin(9600);
  PT_INIT(&ptSonar);
  PT_INIT(&ptButton);
  PT_INIT(&ptStatus);
  PT_INIT(&ptLED);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  prevTime = millis();
  savedDistance = EEPROM.read(address);
  medianDistance = savedDistance+median;
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
  previousState = currentState;
  previousRange = currentRange;
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
          EEPROM.update(address, avgLearn);
          savedDistance = avgLearn;
          medianDistance = savedDistance + median;
          avgLearn = 0;
          countLearn = 0;
          logger("| Learned Distance (" + String(savedDistance) + ")");
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
  prevDistance = d;
  logs = "";
  prevTime = (currentState == IDLE) ? millis() : prevTime;
  delay(1);
}

#include <NewPing.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <string.h>
#include <protothreads.h>

#define BUTTON_PIN    8
#define TRIGGER_PIN   9
#define ECHO_PIN      10
#define LED_PIN       11

const int max_distance = 600;
const int min_distance = 5;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);

volatile unsigned int distance;
unsigned int prevDistance;
unsigned int* pdistance = &distance;
unsigned long prevTime, 
              timeLearn = 15,
              timeIdle = 45;
int address = 0,
    brightness = 0,
    runCount = 0,
    idleCount = 0,
    learnCount = 0,
    savedDistance,
    medianDistance,
    buttonState,
    prevButtonState,
    gotoLearn = 5,
    countLearn = 0,
    avgLearn = 0,
    doeLearn = 5,
    doeIdle = 1,
    fadeAmount = 1;
String logs = "";
bool detectIdle = false;
enum STATE {
  RUN,
  IDLE,
  LEARN
};
enum RANGE {
  GREEN,
  YELLOW,
  RED
};
enum STATE currentState;
enum RANGE currentRange;

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
      case IDLE:
        analogWrite(LED_PIN, brightness);
        break;
      case LEARN:
        if(buttonState == LOW){
          digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        }
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
          //lights are yellow/orange flashing
          break;
        case RED:
          //lights are red
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
  medianDistance = savedDistance + 50;
  logs += (Serial) ? "| Saved Distance (" + String(savedDistance) + ") | Moving RUN" : "";
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
        if(abs(millis() - prevTime) >= (gotoLearn * 1000)){
          logs += (Serial) ? "| Moving RUN > LEARN (" + String(learnCount) + ") " : "";
          learnCount++;
          currentState = LEARN;
        }else{
          logs += (Serial) ? "| Button (" + String(abs(millis() - prevTime)) + ") " : "";
        }
      }else{
        //detect range
        if(prevButtonState == HIGH){
          prevTime = millis();
        }
        if(d != 0){
          if(d > medianDistance){
            currentRange = RED;
            logs += (Serial) ? "| Out of range! " : "";
          }else if(d > savedDistance){
            currentRange = YELLOW;
            logs += (Serial) ? "| Almost in range! " : "";
          }else{
            currentRange = GREEN;
            logs += (Serial) ? "| Within Range! " : "";
          }
        }else{
          logs += (Serial) ? "| No Range " : "";
        }
        //detect IDLE
        if(prevDistance >= (d - doeIdle) && prevDistance <= (d + doeIdle)){
          unsigned long idle = millis() - prevTime; 
          logs += (Serial) ? "| Idle Count (" + String(idle) + ") " : "";
          if(idle >= timeIdle*1000){
            logs += (Serial) ? "| Moving RUN > IDLE (" + String(idleCount) + ") " : "";
            idleCount++;
            currentRange = NULL;
            currentState = IDLE;
          }
        }else{
          prevTime = millis();
        }
      }
      break;
    case IDLE:
      bool exitIdle = false;
      float delta = abs(millis() - prevTime) * 0.001;
      brightness += fadeAmount;
      logs += (Serial) ? "| I'm an idle(" + String(brightness) + ") " : "";
      if(prevDistance <= (d - (doeIdle+1)) || prevDistance >= (d + (doeIdle+1))){
        exitIdle = true;
      }
      if(buttonState == HIGH && prevButtonState == LOW){
        exitIdle = true;
      }
      if(exitIdle){
        logs += (Serial) ? " | Moving Idle > Run (" + String(runCount) + ") " : "";
        runCount++;
        currentState = RUN;
      }
      if(brightness <= 0 || brightness >= 255){
        fadeAmount = -fadeAmount;
      }
      break;
    case LEARN:
      if(buttonState == LOW && prevButtonState == HIGH){
        prevTime = millis();
      }else if(buttonState == LOW && prevButtonState == LOW){
        if(abs(millis() - prevTime) <= (timeLearn * 1000)){
          if(abs(prevDistance - d) <= doeLearn){
            avgLearn += d;
            countLearn++;
            logs += (Serial) ? "| Learning (" + String(avgLearn / countLearn) + ") " : "";
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
          medianDistance = savedDistance + 50;
          avgLearn = 0;
          countLearn = 0;
          logs += (Serial) ? "| Learned Distance (" + String(savedDistance) + ")| Moving LEARN > RUN (" + String(runCount) + ") " : "";
          runCount++;
          currentState = RUN;
        }
      }else if(buttonState == HIGH && prevButtonState == LOW){
        logs += (Serial) ? "| Button/Moving LEARN > RUN (" + String(runCount) + ") " : "";
        runCount++;
        currentState = RUN;
      }
      break;
    default:
      currentState = RUN;
  }
  if(Serial){
    Serial.print(millis());
    Serial.print(" | r/i/l=" + String(runCount) + "/" + String(idleCount) + "/" + String(learnCount) + " ");
    Serial.print("| Distance: " + String(d) + "/" + String(savedDistance) + "cm ");
    Serial.println(logs);
  }
  prevDistance = d;
  logs = "";
  prevTime = (currentState == IDLE) ? millis() : prevTime;
  delay(1);
}

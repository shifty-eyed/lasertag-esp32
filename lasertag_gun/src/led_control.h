#pragma once

#include <Arduino.h>
#include "definitions.h"

#include "freertos/task.h"

static void colorLedOn(uint8_t pin, bool val) {
  #ifdef VEST
  digitalWrite(pin, val ? HIGH : LOW);
  #else
  digitalWrite(pin, val ? LOW : HIGH); //gun
  #endif
}

static void colorLedsOff() {
  colorLedOn(STATUS_LED_GREEN_LO, false);
  colorLedOn(STATUS_LED_BLUE_LO, false);
  colorLedOn(STATUS_LED_RED_LO, false);
}

void loadControlCycle(bool isOnline, int8_t playerTeam, int8_t playerState) {
  while (1) {
    colorLedsOff();
    if (!isOnline) {
      digitalWrite(STATUS_LED_RED, HIGH);
      digitalWrite(STATUS_LED_ONBOARD, LOW);
      vTaskDelay(500);
      digitalWrite(STATUS_LED_RED, LOW);
      digitalWrite(STATUS_LED_ONBOARD, HIGH);
      vTaskDelay(500);
    } else {
      switch (playerTeam) {
        case TEAM_RED: 
          colorLedOn(STATUS_LED_RED_LO, true);
          break;
        case TEAM_BLUE:
          colorLedOn(STATUS_LED_BLUE_LO, true);
          break;
        case TEAM_GREEN:
          colorLedOn(STATUS_LED_GREEN_LO, true);
          break;
        case TEAM_YELLOW:
          colorLedOn(STATUS_LED_RED_LO, true);
          colorLedOn(STATUS_LED_GREEN_LO, true);
          break;
        case TEAM_MAGENTA:
          colorLedOn(STATUS_LED_RED_LO, true);
          colorLedOn(STATUS_LED_BLUE_LO, true);
          break;
        case TEAM_CYAN:
          colorLedOn(STATUS_LED_BLUE_LO, true);
          colorLedOn(STATUS_LED_GREEN_LO, true);
          break;
      }
      if (playerState == PLATER_STATE_PLAY) {
        vTaskDelay(1000);
      } else {
        vTaskDelay(20);
        colorLedsOff();
        vTaskDelay(2000);
      }
    }
  }
}

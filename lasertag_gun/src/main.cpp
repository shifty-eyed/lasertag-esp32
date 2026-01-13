#include <Arduino.h>
#include <IRremote.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "definitions.h"
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
TaskHandle_t udpSendTaskHandle = NULL;
TaskHandle_t udpReceiveTaskHandle = NULL;

static volatile uint32_t pingsSentWithNoResponse = 5;
static volatile int8_t playerId = 0;
static volatile int8_t playerTeam = TEAM_GREEN;
static volatile int8_t playerState = PLATER_STATE_IDLE;
static volatile int8_t bulletsLeft = 0;

static bool isOnline() {
  return pingsSentWithNoResponse < 3;
}

void colorLedsOff() {
  digitalWrite(STATUS_LED_GREEN_LO, HIGH);
  digitalWrite(STATUS_LED_BLUE_LO, HIGH);
  digitalWrite(STATUS_LED_RED_LO, HIGH);
}

void taskStatusLed(void *pvParameters) {
  while (1) {
    colorLedsOff();
    if (!isOnline()) {
      digitalWrite(STATUS_LED_RED, HIGH);
      digitalWrite(STATUS_LED_ONBOARD, LOW);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(STATUS_LED_RED, LOW);
      digitalWrite(STATUS_LED_ONBOARD, HIGH);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    } else {
      switch (playerTeam) {
        case TEAM_RED: 
          digitalWrite(STATUS_LED_RED_LO, LOW);
          break;
        case TEAM_BLUE:
          digitalWrite(STATUS_LED_BLUE_LO, LOW);
          break;
        case TEAM_GREEN:
          digitalWrite(STATUS_LED_GREEN_LO, LOW);
          break;
        case TEAM_YELLOW:
          digitalWrite(STATUS_LED_RED_LO, LOW);
          digitalWrite(STATUS_LED_GREEN_LO, LOW);
          break;
        case TEAM_MAGENTA:
          digitalWrite(STATUS_LED_RED_LO, LOW);
          digitalWrite(STATUS_LED_BLUE_LO, LOW);
          break;
        case TEAM_CYAN:
          digitalWrite(STATUS_LED_BLUE_LO, LOW);
          digitalWrite(STATUS_LED_GREEN_LO, LOW);
          break;
      }
      if (playerState == PLATER_STATE_PLAY) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      } else {
        vTaskDelay(20 / portTICK_PERIOD_MS);
        colorLedsOff();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
      }
    }
  }
}

void taskHeartbeat(void* pvParameters);
void taskBTReceiver(void* pvParameters);
void taskIRReceiver(void* pvParameters);
void sendBTMessage(int8_t messageType, int8_t counterpartPlayerId);

void setup() {
  Serial.begin(9600);
  IrSender.begin(IR_LED_PIN);
  IrReceiver.begin(IR_RECEIVER_PIN);

  pinMode(FIRE_PIN, INPUT_PULLUP);
  pinMode(RELOAD_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_RED, OUTPUT);
  pinMode(STATUS_LED_RED_LO, OUTPUT);
  pinMode(STATUS_LED_BLUE_LO, OUTPUT);
  pinMode(STATUS_LED_GREEN_LO, OUTPUT);
  pinMode(STATUS_LED_ONBOARD, OUTPUT);

  xTaskCreate(taskStatusLed, "taskStatusLed", 2048, NULL, 1, NULL);

  if (!SerialBT.begin(deviceName)) {
      Serial.println("An error occurred initializing Bluetooth");
  } else {
      while (!SerialBT.connected(2000)) {
        Serial.println("Connecting BT..");
        vTaskDelay(2000 / portTICK_PERIOD_MS);
      }
      Serial.println("BT connected.");
  }
  
  xTaskCreate(taskHeartbeat, "taskHeartbeat", 2048, NULL, 1, &udpSendTaskHandle);
  xTaskCreate(taskBTReceiver, "taskBTReceiver", 2048, NULL, 1, &udpReceiveTaskHandle);
  xTaskCreate(taskIRReceiver, "taskIRReceiver", 2048, NULL, 1, &udpReceiveTaskHandle);
}

void loop() {

  int fireButtonState = digitalRead(FIRE_PIN);
  int reloadButtonState = digitalRead(RELOAD_PIN);

  if (fireButtonState == LOW && isOnline()) {
    if (bulletsLeft > 0 && playerId != 0 && playerState == PLATER_STATE_PLAY) {
      IrSender.sendSony(IR_ADDRESS_GUN, playerId, 1, SIRCS_12_PROTOCOL);
    }
    sendBTMessage(MSG_TYPE_GUN_SHOT, 0);
    vTaskDelay(GUN_FIRE_INTERVAL / portTICK_PERIOD_MS);
  }

  if (reloadButtonState == LOW) {
    Serial.println("Reload");
    sendBTMessage(MSG_TYPE_GUN_RELOAD, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}


void taskIRReceiver(void *pvParameters) {
  uint32_t playersHitRecentHit[MAX_PLAYERS];
  for (int i = 0; i < MAX_PLAYERS; i++) {
    playersHitRecentHit[i] = 0;
  }

  while (1) {
    if (IrReceiver.decode()) {
      uint8_t address = (uint8_t)IrReceiver.decodedIRData.address;
      uint8_t command = (uint8_t)IrReceiver.decodedIRData.command;
      //Serial.println("IR received. Address: " + String(address) + ", Command: " + String(command));
      if (address == IR_ADDRESS_GUN) {
        uint8_t hitByPlayer = command;
        if (hitByPlayer != playerId 
          && (millis() - playersHitRecentHit[hitByPlayer] > GUN_FIRE_INTERVAL)) {
          playersHitRecentHit[hitByPlayer] = millis();
          Serial.println("Hit by player: " + String(hitByPlayer));
          sendBTMessage(MSG_TYPE_VEST_HIT, hitByPlayer);
        } else {
          Serial.println("Hit by myself");
        }
      } else if (address == IR_ADDRESS_RESPAWN && playerState == PLATER_STATE_DEAD) {
        uint8_t respawnPointId = command;
        Serial.println("Respawn");
        sendBTMessage(MSG_TYPE_RESPAWN, respawnPointId);
      } else if (address == IR_ADDRESS_AMMO && playerState == PLATER_STATE_PLAY) {
        Serial.println("Got Ammo");
        sendBTMessage(MSG_TYPE_GOT_AMMO, command);
      } else if (address == IR_ADDRESS_HEALTH && playerState == PLATER_STATE_PLAY) {
        Serial.println("Got Health");
        sendBTMessage(MSG_TYPE_GOT_HEALTH, command);
      } else if (address == IR_ADDRESS_FLAG && playerState == PLATER_STATE_PLAY) {
        Serial.println("Flag");
        sendBTMessage(MSG_TYPE_FLAG, command);
      }
      IrReceiver.resume();
    }
    vTaskDelay(30 / portTICK_PERIOD_MS);
  }
}

void sendBTMessage(int8_t messageType, int8_t counterpartPlayerId) {
  SerialBT.write(messageType);
  SerialBT.write(counterpartPlayerId);
  SerialBT.write(STOP_BYTE);
  SerialBT.flush();

  if (messageType != MSG_TYPE_PING) {
    Serial.println("Sent BT message: " + String(messageType));
  }
}

void taskHeartbeat(void* pvParameters) {
  while (1) {
    if (pingsSentWithNoResponse < 10) {
      sendBTMessage(MSG_TYPE_PING, 0);
      pingsSentWithNoResponse ++;
    }
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void taskBTReceiver(void* pvParameters) {
  char incomingPacket[16];
  while (1) {
    if (SerialBT.available()) {
      int len = SerialBT.readBytesUntil(STOP_BYTE, incomingPacket, sizeof(incomingPacket));
      if (len == 0) {
        Serial.println("Received Empty message");
        continue;
      }
      pingsSentWithNoResponse = 0;

      int8_t type = incomingPacket[0];
      if (type == MSG_TYPE_PING) {
        Serial.printf("Received Ping ACK: %d\n", incomingPacket[0]);
      } else if (type == MSG_TYPE_IN_PLAYER_STATE) {
        if (len < 3) {
          Serial.println("Invalid player state message length");
          continue;
        }
        playerId = incomingPacket[1];
        playerTeam = incomingPacket[2];
        playerState = incomingPacket[3];
        bulletsLeft = incomingPacket[4];
        Serial.printf("Received Player Info: playerId=%d, team=%d, state=%d, bullets=%d\n", 
          playerId, playerTeam, playerState, bulletsLeft);
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

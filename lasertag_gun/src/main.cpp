#include <Arduino.h>
#include <IRremote.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "definitions.h"

WiFiUDP udp;
IPAddress serverIp = discoveryServerIp;
TaskHandle_t udpSendTaskHandle = NULL;
TaskHandle_t udpReceiveTaskHandle = NULL;

volatile uint32_t lastPingTime = 999;
volatile int8_t playerTeam = TEAM_GREEN;
volatile int8_t playerState = PLATER_STATE_IDLE;
volatile int8_t bulletsLeft = 0;

bool isOnline() {
  return lastPingTime < 5;
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
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(STATUS_LED_RED, LOW);
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

void connectToWiFi();
void taskUdpHeartbeat(void* pvParameters);
void taskUdpReceiver(void* pvParameters);
void taskIRReceiver(void* pvParameters);
void sendUdpMessage(int8_t messageType, int8_t counterpartPlayerId);

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

  Serial.println("Ready");

  xTaskCreate(taskStatusLed, "taskStatusLed", 2048, NULL, 1, NULL);

  connectToWiFi();
  udp.begin(localUdpPort);
  Serial.println("UDP started on port: " + String(localUdpPort));
  xTaskCreate(taskUdpHeartbeat, "taskUdpHeartbeat", 2048, NULL, 1, &udpSendTaskHandle);
  xTaskCreate(taskUdpReceiver, "taskUdpReceiver", 2048, NULL, 1, &udpReceiveTaskHandle);
  xTaskCreate(taskIRReceiver, "taskIRReceiver", 2048, NULL, 1, &udpReceiveTaskHandle);
}

void loop() {

  int fireButtonState = digitalRead(FIRE_PIN);
  int reloadButtonState = digitalRead(RELOAD_PIN);

  if (fireButtonState == LOW && isOnline()) {
    if (bulletsLeft > 0) {
      IrSender.sendSony(COMMON_IR_ADDRESS, PLAYER_ID, 1, SIRCS_12_PROTOCOL);
    }
    sendUdpMessage(MSG_TYPE_GUN_SHOT, 0);
    vTaskDelay(GUN_FIRE_INTERVAL / portTICK_PERIOD_MS);
  }

  if (reloadButtonState == LOW) {
    Serial.println("Reload");
    sendUdpMessage(MSG_TYPE_GUN_RELOAD, 0);
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
      if (IrReceiver.decodedIRData.address == COMMON_IR_ADDRESS) {
        uint8_t hitByPlayer = (uint8_t)IrReceiver.decodedIRData.command;
        if (hitByPlayer != PLAYER_ID 
          && (millis() - playersHitRecentHit[hitByPlayer] > GUN_FIRE_INTERVAL)) {
          playersHitRecentHit[hitByPlayer] = millis();
          Serial.println("Hit by player: " + String(hitByPlayer));
          sendUdpMessage(MSG_TYPE_VEST_HIT, hitByPlayer);
        } else {
          Serial.println("Hit by myself");
        }
      }
      IrReceiver.resume();
      
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    Serial.println("Connecting...");
  }

  Serial.println("Connected to WiFi.");
  Serial.print("Local IP Address: ");
  Serial.println(WiFi.localIP());
}

void sendUdpMessage(int8_t messageType, int8_t counterpartPlayerId) {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  udp.beginPacket(serverIp, udpPort);
  udp.write(messageType);
  udp.write(PLAYER_ID);
  udp.write(counterpartPlayerId);
  udp.endPacket();
  if (messageType != MSG_TYPE_PING) {
    Serial.println("Sent UDP message: " + String(messageType));
  }
}

void taskUdpHeartbeat(void* pvParameters) {
  while (1) {
    sendUdpMessage(MSG_TYPE_PING, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    lastPingTime ++;
  }
  vTaskDelete(NULL);
}

void taskUdpReceiver(void* pvParameters) {
  char incomingPacket[16];
  while (1) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      serverIp = udp.remoteIP();
      int len = udp.read(incomingPacket, sizeof(incomingPacket));
      if (len == 0) {
        Serial.println("Received Empty message");
        continue;
      }
      lastPingTime = 0;

      int8_t type = incomingPacket[0];
      if (type == MSG_TYPE_PING) {
        //Serial.printf("Received Ping ACK: %d\n", incomingPacket[0]);
      } else if (type == MSG_TYPE_PLAYER_STATE) {
        if (len < 3) {
          Serial.println("Invalid player state message length");
          continue;
        }
        playerTeam = incomingPacket[1];
        playerState = incomingPacket[2];
        bulletsLeft = incomingPacket[3];
        Serial.printf("Received Player Info: team=%d, state=%d, bullets=%d\n", playerTeam, playerState, bulletsLeft);
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}
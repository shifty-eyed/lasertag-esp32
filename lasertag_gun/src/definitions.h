#define PLAYER_ID 2
#define VEST
//#define GUN

#define MAX_PLAYERS 16
#define GUN_FIRE_INTERVAL 400

#define MSG_TYPE_PING 1
#define MSG_TYPE_GUN_SHOT 2
#define MSG_TYPE_GUN_RELOAD 3
#define MSG_TYPE_VEST_HIT 4
#define MSG_TYPE_PLAYER_STATE 5


#define TEAM_RED 1
#define TEAM_BLUE 2
#define TEAM_GREEN 3
#define TEAM_YELLOW 4
#define TEAM_MAGENTA 5
#define TEAM_CYAN 6


#define PLATER_STATE_IDLE 1
#define PLATER_STATE_PLAY 2
#define PLATER_STATE_DEAD 3


#define IR_LED_PIN 13
#define IR_RECEIVER_PIN 27

#define FIRE_PIN 12
#define RELOAD_PIN 14

#define STATUS_LED_RED 26
#define STATUS_LED_BLUE_LO 25
#define STATUS_LED_RED_LO 33
#define STATUS_LED_GREEN_LO 32

#define COMMON_IR_ADDRESS 0x1A

const char* ssid = "imenilenina-bistro";
const char* password = "10101010";

#include <WiFi.h>
const IPAddress discoveryServerIp = IPAddress(255, 255, 255, 255);

#ifdef VEST
const int udpPort = 9877;
#endif
#ifdef GUN
const int udpPort = 9876;
#endif
const unsigned int localUdpPort = 1234;
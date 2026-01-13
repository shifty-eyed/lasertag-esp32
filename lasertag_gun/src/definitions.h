//#define VEST
#define GUN

#define MAX_PLAYERS 16
#define GUN_FIRE_INTERVAL 300
#define STOP_BYTE 125

#define MSG_TYPE_PING 1
#define MSG_TYPE_GUN_SHOT 2
#define MSG_TYPE_GUN_RELOAD 3
#define MSG_TYPE_VEST_HIT 5
#define MSG_TYPE_RESPAWN 6
#define MSG_TYPE_GOT_HEALTH 16
#define MSG_TYPE_GOT_AMMO 17
#define MSG_TYPE_FLAG 18


#define MSG_TYPE_IN_PLAYER_STATE 13


#define TEAM_RED 0
#define TEAM_BLUE 1
#define TEAM_GREEN 2
#define TEAM_YELLOW 3
#define TEAM_MAGENTA 4
#define TEAM_CYAN 5

#define PLATER_STATE_IDLE 0
#define PLATER_STATE_PLAY 1
#define PLATER_STATE_DEAD 2
#define PLATER_STATE_OFFLINE 3

#define IR_LED_PIN 13
#define IR_RECEIVER_PIN 27

#define FIRE_PIN 12
#define RELOAD_PIN 14

#define STATUS_LED_RED 26
#define STATUS_LED_BLUE_LO 25
#define STATUS_LED_RED_LO 33
#define STATUS_LED_GREEN_LO 32
#define STATUS_LED_ONBOARD 2

#define IR_ADDRESS_GUN 5
#define IR_ADDRESS_RESPAWN 1
#define IR_ADDRESS_HEALTH 2
#define IR_ADDRESS_AMMO 3
#define IR_ADDRESS_FLAG 4


#ifdef VEST
const char * deviceName = "LaserTagVest";
#endif
#ifdef GUN
const char * deviceName = "LaserTagGun";
#endif

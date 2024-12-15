// Packet strucure
// --remocon -> receiver:
//   0x54 0x55 COMType LEN From To DATA(COM, DIR1, DIR2, DIR3, M1Sp, M2Sp, M3Sp, RESERVE1, RESERVE2)
// --receiver -> remocon:
//   0x54 0x55 INFO LEN From To DATA(P_STAT, sen1, sen2, sen3, CRCL, CRCH)
// --Pairing - broadcast(receiver -> any remocon):
//   0x54 0x55 ADV LEN From To DATA(RSSI, CH1, MAC(6))
// --Pairng - acknowledge(remocon -> the receiver):
//   0x54 0x55 ACK LEN From to DATA(RSSI, CH1, MAC(6))

#ifndef ESPNOW_CONFIG_H
#define ESPNOW_CONFIG_H

#include <inttypes.h>
#include <stdbool.h>

#define ESP_TASK_READY    0
#define ESP_TASK_RUNNING  1
#define ESP_TASK_SUSPEND  2
#define ESP_TASK_DELETE   3


//common index
#define ID_START1   0
#define ID_START2   1
#define ID_INFO     2
#define ID_LEN      3
#define ID_FROM     4
#define ID_TO       5

//controller status index :_sendbuf
#define ID_COM      6
#define ID_DIR      7
#define ID_SPD      10
#define ID_DELI1    15
#define ID_DELI2    16

//receiver Packet : rPacket.msg
#define ID_CRCL     10
#define ID_CRCH     11

//receiver packet & pairing packet index : rPacket.msg , ACKPacket.msg  
#define ID_P_STAT   6
#define ID_SEN      7

//pairing packet_index;
#define ID_MAC      10



#define INFO_SENSOR     1
#define INFO_ADVERTISE  0xE2
#define INFO_ACK        0xE3
#define DEV_RECEIVER    0
#define DEV_REMOCON     1
#define STAT_READY      0
#define STAT_PROCESS    1

#define MY_ESPNOW_WIFI_MODE WIFI_MODE_STA
#define MY_ESPNOW_WIFI_IF ESP_IF_WIFI_STA

#define MAC_CH_NUMBER       7
#define MAC_NUMBER          6
#define USED_CHANNEL        0
#define DEFAULT_CHANNEL     1
#define BROADCAST_CHANNEL   13

#define APP_APPLICATION_MODE     1
#define APP_PAIRING_MODE         2

typedef struct
{
    uint8_t mac_addr[6];
    uint8_t msg[32];
    int rxBytes;
} resp_packet_t;

#define BROADCAST_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

#define DESTINATION_MAC {0x48, 0xCA, 0x43, 0xD4, 0x3C, 0x60}    //receiver
//#define MY_DEVICE_MAC  {0x48, 0xCA, 0x43, 0xD4,  0x24, 0xC0}  //remocon
#define MY_DEVICE_MAC {0x48, 0xCA, 0x43, 0xD4, 0x43, 0x64}      //dev-remocon

#define MY_ESPNOW_PMK "pmk1234567890123"


// #define MY_ESPNOW_ENABLE_LONG_RANGE 1

#define MY_SLAVE_DEEP_SLEEP_TIME_MS 10000

#endif // ESPNOW_BASIC_CONFIG_H
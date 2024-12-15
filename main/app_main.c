#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "espconfig.h"
#include "esp_now.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_sleep.h"

// #include "app_main.h"

#define PACKET_LENGTH       18   //ENTRY PACKET
#define ACK_PACKET_LENGTH   18
#define RECV_PACKET_LENGTH  12
#define SEND_PACKET_TERM    10
#define TOGGLE_TERM         50

#define SW1_PIN     GPIO_NUM_7
#define SW2_PIN     GPIO_NUM_8
#define SW3_PIN     GPIO_NUM_9
#define SW4_PIN     GPIO_NUM_10
#define BTN_PIN     GPIO_NUM_0
#define LED_PIN     GPIO_NUM_1
   

#define BTN_PRESS               0
#define BTN_RELEASE             1
#define PRESS_INTERVAL          5000000
#define SLEEP_WAIT_TIME         60000000
//#define PAIRING_MODE_LIMIT      30000000
#define PAIRING_MODE_LIMIT      1000000
#define LED_BLINK_TERM          10

static const char *TAG = "app_main_remocon";

void init_nvs_flash(void);
void init_espnow_slave(unsigned int _mac[MAC_CH_NUMBER]);
void start_espnow_slave(const QueueHandle_t recvPacket, const QueueHandle_t sendPacket);
void delete_espnowTask(void);
void suspend_espnowTask(void);
void resume_espnowTask(void);
int getTaskStatus_espnow(void);

void init_spiffs(void);
void save_mac(const char *file_path, unsigned int info_mac[MAC_CH_NUMBER]);
void read_mac(const char *file_path, unsigned int info_mac[MAC_CH_NUMBER]);
void save_mode(const char *file_path, unsigned int _mode[1]);
void read_mode(const char *file_path, unsigned int _mode[1]);

static QueueHandle_t xQueueESPnowSend = NULL;
static QueueHandle_t xQueueESPnowRecv = NULL;
static TimerHandle_t xAutoReloadTimer;
static BaseType_t xTimer1Started, xTimer1Stopped;

static const int BUF_SIZE = 32;
static uint8_t led_state = 0; // off
static int rxUARTBytes;
static int gCount = 0;
static int rCount = 0;
static int tCount = 0;
static int ledCount = 0;
const char *file_path = "/spiffs/mac_add.txt";
const char *mode_file_path = "/spiffs/mode.txt";

static resp_packet_t sPacket;  // for entry
static resp_packet_t rPacket;

static char _sendbuf[PACKET_LENGTH];  // receiver로 보낼 모터 제어 패킷 버퍼  - console_rx_task에서 받는 entry 패킷과 동일하나 우선 순위임

int64_t pressStartTime = 0;
int64_t pressingTime = 0;
int64_t paringStartTime = 0;
int64_t pairinglapseTime = 0;

int appMode = APP_APPLICATION_MODE;
static bool entryPacketEnable = true;

int64_t sleepStartTime = 0;
int64_t sleepLapseTime = 0;

static void pvTimerCallback(TimerHandle_t xTimer)
{
    static TickType_t xTimeNow;
    xTimeNow = xTaskGetTickCount();
    // printf("Auto-reload timer callback executing %d\r\n", xTimeNow);
    gCount++;
    rCount++;
    tCount++;
    ledCount++;
}

void console_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, BUF_SIZE * 8, 0, 0, NULL, 0);
    uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config);
}

static void console_rx_task(void *arg)
{
    static char _buf[PACKET_LENGTH];   // entry에서 받은 패킷 저장 버퍼
    fpurge(stdin);
    memset(_buf, 0, PACKET_LENGTH);

    while (true)
    {
        if (fgets(_buf, PACKET_LENGTH, stdin) != NULL)
        {
            //if (xSemaphoreGive(xConsoleRecv) == pdPASS)
            //{
            //    memcpy(buf, _buf, PACKET_LENGTH);
                //printf("RECV:%s\n", _buf);
            //}
            sleepStartTime = esp_timer_get_time();
            memcpy(sPacket.msg, _buf, PACKET_LENGTH);
            sPacket.rxBytes = PACKET_LENGTH;
            memset(_buf, 0x00, sizeof(_buf));
            //if(entryPacketEnable == true) xQueueSend(xQueueESPnowSend, &sPacket, portMAX_DELAY);
        }
        
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void shutdown_handler(void)
{
    printf("System is shutting down!\n");
}

void app_main(void)
{
    ESP_LOGI(TAG, "ARCADE JOYPAD START!");

    gpio_config_t sw_conf;
    gpio_config_t btn_conf;
    gpio_config_t led_conf;   

    sw_conf.intr_type = GPIO_INTR_DISABLE;            
    sw_conf.mode = GPIO_MODE_INPUT;                   
    sw_conf.pin_bit_mask = (1ULL << SW1_PIN) | (1ULL << SW2_PIN) | (1ULL << SW3_PIN) | (1ULL << SW4_PIN);
    sw_conf.pull_down_en = 0;                        
    sw_conf.pull_up_en = 0;                          
    gpio_config(&sw_conf);

    btn_conf.intr_type = GPIO_INTR_DISABLE;             
    btn_conf.mode = GPIO_MODE_INPUT;                   
    btn_conf.pin_bit_mask = (1ULL << BTN_PIN); 
    btn_conf.pull_down_en = 0;                         
    btn_conf.pull_up_en = GPIO_PULLUP_ENABLE;                           
    gpio_config(&btn_conf);    

    led_conf.intr_type = GPIO_INTR_DISABLE;         
    led_conf.mode = GPIO_MODE_OUTPUT;               
    led_conf.pin_bit_mask = (1ULL << LED_PIN); 
    led_conf.pull_down_en = 0;                      
    led_conf.pull_up_en = 0;          
    gpio_config(&led_conf);
    
    gpio_set_level(LED_PIN, 0);     
   
    static bool firstPress = false;

    //buf = (uint8_t *)malloc(sizeof(uint8_t) * BUF_SIZE);
    xQueueESPnowRecv = xQueueCreate(10, sizeof(resp_packet_t));
    xQueueESPnowSend = xQueueCreate(10, sizeof(resp_packet_t));
    //vSemaphoreCreateBinary(xConsoleRecv);

    init_spiffs();

    unsigned int _mode[1] = {APP_APPLICATION_MODE};
    int _channel = 0;
    unsigned int destMAC[7] = {0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 1};  //for example
    unsigned int broadMAC[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, BROADCAST_CHANNEL};

    // run first time..
    //------------------------------------------------
    //save_mode(mode_file_path, _mode);
    //save_mac(file_path, destMAC);
    //------------------------------------------------
    
    read_mode(mode_file_path, _mode);
    if (_mode[0] == APP_PAIRING_MODE)
    {
        appMode = APP_PAIRING_MODE;
        ESP_LOGI(TAG, "PAIRNG MODE!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    else
    {
        appMode = APP_APPLICATION_MODE;
        ESP_LOGI(TAG, "APPLICATION MODE!");
    }

    if (appMode == APP_APPLICATION_MODE)
    {
        read_mac(file_path, destMAC);
        printf("READED MAC:");
        for (int j = 0; j < MAC_CH_NUMBER; j++)
        {
            printf("%x ", destMAC[j]);
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    console_init();    
    init_nvs_flash();

    if(appMode == APP_APPLICATION_MODE) init_espnow_slave(destMAC);
    else if(appMode == APP_PAIRING_MODE)  init_espnow_slave(broadMAC);
    start_espnow_slave(xQueueESPnowRecv, xQueueESPnowSend);

    esp_register_shutdown_handler(shutdown_handler);

    memset(_sendbuf, 0, PACKET_LENGTH);    
    _sendbuf[ID_START1] = 0x54;
    _sendbuf[ID_START2] = 0x55;
    _sendbuf[ID_INFO] = 0x00;
    _sendbuf[ID_LEN] = 9;
    _sendbuf[ID_FROM] = 0;
    _sendbuf[ID_TO] = 0;
    _sendbuf[ID_COM] = 0;
    _sendbuf[ID_DIR] = 0;
    _sendbuf[ID_SPD] = 0;
    _sendbuf[ID_DIR + 1] = 0;
    _sendbuf[ID_SPD + 1] = 0;
    _sendbuf[ID_DIR + 2] = 0;
    _sendbuf[ID_SPD + 2] = 0;
    _sendbuf[ID_DELI1] = 36;
    _sendbuf[ID_DELI2] = 36;

    // initialized for the status that unconnected with receivers..
    rPacket.msg[ID_START1] = 0x54;
    rPacket.msg[ID_START2] = 0x55;
    rPacket.msg[ID_INFO] = INFO_SENSOR;
    rPacket.msg[ID_LEN] = 4; // only data payload
    rPacket.msg[ID_FROM] = DEV_RECEIVER;
    rPacket.msg[ID_TO] = DEV_REMOCON;
    rPacket.msg[ID_P_STAT] = STAT_READY;
    rPacket.msg[ID_SEN] = 0x00;
    rPacket.msg[ID_SEN + 1] = 0x00;
    rPacket.msg[ID_SEN + 2] = 0x00;
    rPacket.msg[ID_CRCL] = 36;
    rPacket.msg[ID_CRCH] = 36;
    rPacket.rxBytes = RECV_PACKET_LENGTH;

    if(appMode == APP_APPLICATION_MODE) {

        xTaskCreate(console_rx_task, "console_rx_task", 1024 * 8, NULL, 5, NULL);
    }

    xAutoReloadTimer = xTimerCreate("AutoReload", pdMS_TO_TICKS(10UL), pdTRUE, 0, pvTimerCallback);
    if (xAutoReloadTimer != NULL)
    {
        xTimer1Started = xTimerStart(xAutoReloadTimer, 0);
    }

    /*
    const char *file_path = "/spiffs/mac_add.txt";
    unsigned int myMac[MAC_CH_NUMBER] = {0x24, 0x6F, 0x28, 0x1A, 0xB2, 0xC1, 0x0A};
    unsigned int iMac[MAC_CH_NUMBER] = {};

    save_mac(file_path, myMac);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    read_mac(file_path, iMac);

    printf("output:");
    for (int j = 0; j < MAC_CH_NUMBER; j++)
    {
        printf("%02X ", iMac[j]);
    }
    printf("\n");
    */

    paringStartTime = esp_timer_get_time();
    sleepStartTime = esp_timer_get_time();

    if (appMode == APP_APPLICATION_MODE) ESP_LOGI(TAG, "APPLICATION MODE...");
    else if(appMode == APP_PAIRING_MODE) ESP_LOGI(TAG, "PAIRING MODE...");
   
    uint8_t baseMAC[6];
    esp_err_t result = esp_efuse_mac_get_default(baseMAC);
    if (result == ESP_OK)
    {
        printf("MY Base MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", baseMAC[0], baseMAC[1], baseMAC[2], baseMAC[3], baseMAC[4], baseMAC[5]);
    }
    else printf("Failed to get MY MAC address: %s\n", esp_err_to_name(result));

    while (true)
    {
        //페어링 모드 버튼 확인
        /*
        int _pBtn = gpio_get_level(BTN_PIN);
        if (_pBtn == BTN_PRESS)
        {
            if (firstPress == false)
            {
                pressStartTime = esp_timer_get_time();
                firstPress = true;
            }
            pressingTime = esp_timer_get_time();

            if ((pressingTime - pressStartTime) > PRESS_INTERVAL)
            {
                _mode[0] = APP_PAIRING_MODE;
                save_mode(mode_file_path, _mode);
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
        }
        else
        {
            firstPress = false;
        }
        */

        sleepLapseTime = esp_timer_get_time();
        if((sleepLapseTime - sleepStartTime) > SLEEP_WAIT_TIME)
        {
            //esp_deep_sleep_enable_gpio_wakeup((1ULL << SW_UP1_PIN) | (1ULL << SW_UP2_PIN) | (1ULL << SW_UP3_PIN), ESP_GPIO_WAKEUP_GPIO_LOW);
            esp_deep_sleep_enable_gpio_wakeup(BIT(BTN_PIN), ESP_GPIO_WAKEUP_GPIO_LOW);
           
            ESP_LOGI(TAG, "Entering deep sleep mode...\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_deep_sleep_start();
        }

        BaseType_t _stat = xQueueReceive(xQueueESPnowRecv, &rPacket, pdMS_TO_TICKS(1));

        if (appMode == APP_APPLICATION_MODE)
        {
            int sw1 = gpio_get_level(SW1_PIN);
            int sw2 = gpio_get_level(SW2_PIN);
            int sw3 = gpio_get_level(SW3_PIN);
            int sw4 = gpio_get_level(SW4_PIN);
            int btn = gpio_get_level(BTN_PIN);

            ESP_LOGI(TAG, "SW1: %d SW2: %d SW3: %d SW4: %d btn:%d", sw1, sw2, sw3, sw4, btn);            
            
            if (gCount >= SEND_PACKET_TERM)
            {            
                gCount = 0;
                if(entryPacketEnable == false)  memcpy(sPacket.msg, _sendbuf, PACKET_LENGTH); //컨트롤러 패킷을 복사  - 그렇지 않으면 엔트리 패킷 반영
                sPacket.rxBytes = PACKET_LENGTH;
                xQueueSend(xQueueESPnowSend, &sPacket, portMAX_DELAY);
            }           
            
            if (_stat == pdTRUE)
            {
                printf("\n");
                for (int j = 0; j < rPacket.rxBytes; j++)
                {
                    printf("%c", rPacket.msg[j]);
                }
            }                
        }
        // else if(rPacket.msg[ID_INFO] == INFO_ADVERTISE)
        else if (appMode == APP_PAIRING_MODE)
        {
            //esp_now_peer_info_t peerInfo;
            static resp_packet_t ACKPacket;
            static int ackChannel = 0;
            unsigned int _mac[7];
            static bool _led = false;
            
            if (ledCount > LED_BLINK_TERM)
            {
                ledCount = 0;                
                if(_led == false) {gpio_set_level(LED_PIN, 1); _led = true;}
                else {gpio_set_level(LED_PIN, 0); _led = false;}
            }
            

            if(_stat == pdTRUE)
            {
                printf("my partner MAC: %02X:%02X:%02X:%02X:%02X:%02X CH:%02X", rPacket.msg[ID_MAC], rPacket.msg[ID_MAC + 1], rPacket.msg[ID_MAC + 2], rPacket.msg[ID_MAC + 3], rPacket.msg[ID_MAC + 4], rPacket.msg[ID_MAC + 5], rPacket.msg[ID_SEN]);

                if (rPacket.msg[ID_INFO] == INFO_ADVERTISE)
                {                    
                    _mac[0] = rPacket.msg[ID_MAC];
                    _mac[1] = rPacket.msg[ID_MAC + 1];
                    _mac[2] = rPacket.msg[ID_MAC + 2];
                    _mac[3] = rPacket.msg[ID_MAC + 3];
                    _mac[4] = rPacket.msg[ID_MAC + 4];
                    _mac[5] = rPacket.msg[ID_MAC + 5];
                    _mac[6] = rPacket.msg[ID_SEN];          //channel

                    //ESP_LOG_BUFFER_HEX("Copied destination MAC:", _mac, MAC_CH_NUMBER);
                
                    save_mac(file_path, _mac);

                    //memcpy(peerInfo.peer_addr, _mac, 6);
                    //peerInfo.channel = rPacket.msg[ID_SEN];
                    //peerInfo.ifidx = MY_ESPNOW_WIFI_IF;
                    //peerInfo.encrypt = false;
                    // if (!esp_now_is_peer_exist(_mac)) ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
                    // if upper code line is excuted,  Peer channel is not equal to the home channel!

                    memcpy(ACKPacket.mac_addr, _mac, 6);
                    ackChannel = rPacket.msg[ID_SEN];

                    ACKPacket.msg[ID_START1] = 0x54;
                    ACKPacket.msg[ID_START2] = 0x55;
                    ACKPacket.msg[ID_INFO] = INFO_ACK;
                    ACKPacket.msg[ID_LEN] = 4; // only data payload
                    ACKPacket.msg[ID_FROM] = DEV_REMOCON;
                    ACKPacket.msg[ID_TO] = DEV_RECEIVER;
                    ACKPacket.msg[ID_P_STAT] = STAT_READY;
                    ACKPacket.msg[ID_SEN] = rPacket.msg[ID_SEN];  //received channel number from my partner
                    ACKPacket.msg[ID_SEN + 1] = rPacket.msg[ID_SEN + 1];
                    ACKPacket.msg[ID_SEN + 2] = rPacket.msg[ID_SEN + 2];
                    ACKPacket.msg[ID_MAC] = baseMAC[0];
                    ACKPacket.msg[ID_MAC + 1] = baseMAC[1];
                    ACKPacket.msg[ID_MAC + 2] = baseMAC[2];
                    ACKPacket.msg[ID_MAC + 3] = baseMAC[3];
                    ACKPacket.msg[ID_MAC + 4] = baseMAC[4];
                    ACKPacket.msg[ID_MAC + 5] = baseMAC[5];
                    ACKPacket.msg[ACK_PACKET_LENGTH - 2] = 36;
                    ACKPacket.msg[ACK_PACKET_LENGTH - 1] = 36;
                    ACKPacket.rxBytes = ACK_PACKET_LENGTH;
                    xQueueSend(xQueueESPnowSend, &ACKPacket, portMAX_DELAY);
                    vTaskDelay(pdMS_TO_TICKS(50));
                    xQueueSend(xQueueESPnowSend, &ACKPacket, portMAX_DELAY);
                    vTaskDelay(pdMS_TO_TICKS(50));

                    _mode[0] = APP_APPLICATION_MODE;
                    save_mode(mode_file_path, _mode);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    esp_restart();
                }
            }

            pairinglapseTime = esp_timer_get_time();
            if ((pairinglapseTime - paringStartTime) > PAIRING_MODE_LIMIT)
            {
                _mode[0] = APP_APPLICATION_MODE;
                save_mode(mode_file_path, _mode);
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        }        

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
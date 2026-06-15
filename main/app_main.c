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
#include "esp_adc/adc_oneshot.h"
#include "driver/usb_serial_jtag.h"

// #include "app_main.h"

#define PACKET_LENGTH 13 // ENTRY PACKET
#define ACK_PACKET_LENGTH 18
#define RECV_PACKET_LENGTH 12
#define SEND_PACKET_TERM 10
#define USB_PACKET_SEND_TERM_MS 1000
#define TOGGLE_TERM 50

#define SW1_PIN GPIO_NUM_7
#define SW2_PIN GPIO_NUM_8
#define SW3_PIN GPIO_NUM_9
#define SW4_PIN GPIO_NUM_10
#define BTN_PIN GPIO_NUM_0
#define LED_PIN GPIO_NUM_1

#define BTN_PRESS 0
#define BTN_RELEASE 1
#define PRESS_INTERVAL 5000000
#define SLEEP_WAIT_TIME 60000000
// #define PAIRING_MODE_LIMIT      30000000
#define PAIRING_MODE_LIMIT 3000000
#define LED_BLINK_TERM 10

#define CW 1
#define CCW 2
#define ORIGIN 0

#define LED_RED 1
#define LED_YELLOW 2
#define LED_BLUE 3
#define LED_WHITE 4
#define ROBOT_OFF 0

#define MOTOR_SPD 230
#define SPD_OFFSET 100

static const char *TAG = "arcade Joypad";

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
static BaseType_t xTimer1Started;

static const int BUF_SIZE = 32;
static int gCount = 0;
static int rCount = 0;
static int tCount = 0;
static int ledCount = 0;
const char *file_path = "/spiffs/mac_add.txt";
const char *mode_file_path = "/spiffs/mode.txt";

static resp_packet_t sPacket; // for entry
static resp_packet_t rPacket;
static resp_packet_t usbTxPacket;

static char _sendbuf[PACKET_LENGTH]; // receiver로 보낼 모터 제어 패킷 버퍼  - console_rx_task에서 받는 entry 패킷과 동일하나 우선 순위임

int64_t pressStartTime = 0;
int64_t pressingTime = 0;
int64_t paringStartTime = 0;
int64_t pairinglapseTime = 0;

int appMode = APP_APPLICATION_MODE;

int64_t sleepStartTime = 0;
int64_t sleepLapseTime = 0;

static void pvTimerCallback(TimerHandle_t xTimer)
{
    // printf("Auto-reload timer callback executing %d\r\n", xTaskGetTickCount());
    gCount++;
    rCount++;
    tCount++;
    ledCount++;
}

static void log_usb_packet(const uint8_t *data, int len)
{
    ESP_LOGI(TAG, "USB RX packet: %d bytes", len);
    ESP_LOG_BUFFER_HEX(TAG, data, len);
}

void console_init(void)
{
    if (usb_serial_jtag_is_driver_installed())
    {
        ESP_LOGI(TAG, "USB Serial/JTAG driver already installed");
        return;
    }

    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE * 8,
        .tx_buffer_size = BUF_SIZE * 8,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI(TAG, "USB Serial/JTAG console initialized");
}

static void console_rx_task(void *arg)
{
    uint8_t _buf[BUF_SIZE];
    memset(_buf, 0, sizeof(_buf));

    while (true)
    {
        int len = usb_serial_jtag_read_bytes(_buf, sizeof(_buf), pdMS_TO_TICKS(20));
        if (len > 0)
        {
            sleepStartTime = esp_timer_get_time();
            log_usb_packet(_buf, len);

            if (len == PACKET_LENGTH)
            {
                memcpy(sPacket.msg, _buf, PACKET_LENGTH);
                sPacket.rxBytes = PACKET_LENGTH;
            }

            memset(_buf, 0x00, sizeof(_buf));
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void console_tx_task(void *arg)
{
    while (true)
    {
        usb_serial_jtag_write_bytes(usbTxPacket.msg, usbTxPacket.rxBytes, pdMS_TO_TICKS(20));
        usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(20));
        vTaskDelay(pdMS_TO_TICKS(USB_PACKET_SEND_TERM_MS));
    }
}

void shutdown_handler(void)
{
    printf("System is shutting down!\n");
}

static void fill_broadcast_event_packet(resp_packet_t *packet, uint8_t event)
{
    memset(packet, 0, sizeof(resp_packet_t));
    memset(packet->mac_addr, 0xFF, MAC_NUMBER);
    memcpy(packet->msg, usbTxPacket.msg, RECV_PACKET_LENGTH);
    packet->msg[ID_INFO] = INFO_BROAD;
    packet->msg[ID_FROM] = DEV_REMOCON;
    packet->msg[ID_TO] = DEV_ALL;
    packet->msg[ID_SEN] = event;
    packet->rxBytes = RECV_PACKET_LENGTH;
}

static void send_game_start_broadcast(void)
{
    resp_packet_t broadPacket;
    fill_broadcast_event_packet(&broadPacket, GAME_START);

    ESP_LOGI(TAG, "GAME START BROADCAST SEND START!");
    for (int k = 0; k < 3; k++)
    {
        BaseType_t result = xQueueSend(xQueueESPnowSend, &broadPacket, portMAX_DELAY);
        if (result == pdTRUE)
            ESP_LOGI(TAG, "GAME START BROADCAST SEND %d/3 OK!", k + 1);
        else
            ESP_LOGE(TAG, "GAME START BROADCAST SEND %d/3 FAIL!", k + 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static bool is_game_stop_packet(const resp_packet_t *packet)
{
    return packet->rxBytes >= RECV_PACKET_LENGTH &&
           packet->msg[ID_START1] == 0x54 &&
           packet->msg[ID_START2] == 0x55 &&
           packet->msg[ID_INFO] == INFO_BROAD &&
           packet->msg[ID_FROM] == DEV_SCORE_MANAGER &&
           packet->msg[ID_TO] == DEV_ALL &&
           packet->msg[ID_SEN] == GAME_STOP;
}

static void stop_robot_and_reset(void)
{
    resp_packet_t stopPacket;

    memset(&stopPacket, 0, sizeof(stopPacket));
    memcpy(stopPacket.msg, _sendbuf, PACKET_LENGTH);
    stopPacket.msg[ID_DIR] = ORIGIN;
    stopPacket.msg[ID_SPD] = 0;
    stopPacket.msg[ID_DIR + 1] = ORIGIN;
    stopPacket.msg[ID_SPD + 1] = 0;
    stopPacket.msg[ID_SOL] = 0;
    stopPacket.rxBytes = PACKET_LENGTH;

    for (int k = 0; k < 3; k++)
    {
        xQueueSend(xQueueESPnowSend, &stopPacket, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_restart();
}

void app_main(void)
{
    ESP_LOGI(TAG, "ARCADE JOYPAD START!");

    gpio_config_t btn_conf;
    gpio_config_t led_conf;

    //YW  joystick은 ADC로 연결되어 있음
    static adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t adc_init_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&adc_init_cfg, &adc1_handle);
    adc_oneshot_chan_cfg_t adc_chan_cfg = { .bitwidth = ADC_BITWIDTH_12, .atten = ADC_ATTEN_DB_12 };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &adc_chan_cfg); // GPIO3
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &adc_chan_cfg); // GPIO4

    /*
    sw_conf.intr_type = GPIO_INTR_DISABLE;
    sw_conf.mode = GPIO_MODE_INPUT;
    sw_conf.pin_bit_mask = (1ULL << SW1_PIN) | (1ULL << SW2_PIN) | (1ULL << SW3_PIN) | (1ULL << SW4_PIN);
    sw_conf.pull_down_en = 0;
    sw_conf.pull_up_en = 0;
    gpio_config(&sw_conf);
    */

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

    gpio_set_level(LED_PIN, 1);

    static bool firstPress = false;

    // buf = (uint8_t *)malloc(sizeof(uint8_t) * BUF_SIZE);
    xQueueESPnowRecv = xQueueCreate(10, sizeof(resp_packet_t));
    xQueueESPnowSend = xQueueCreate(10, sizeof(resp_packet_t));
    // vSemaphoreCreateBinary(xConsoleRecv);

    init_spiffs();

    unsigned int _mode[1] = {APP_APPLICATION_MODE};
    unsigned int destMAC[7] = {0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, DEFAULT_CHANNEL}; // for example
    unsigned int broadMAC[7] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, BROADCAST_CHANNEL};

    // run first time..
    //------------------------------------------------
    // save_mode(mode_file_path, _mode);
    // save_mac(file_path, destMAC);
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

    if (appMode == APP_APPLICATION_MODE)
        init_espnow_slave(destMAC);
    else if (appMode == APP_PAIRING_MODE)
        init_espnow_slave(broadMAC);
    start_espnow_slave(xQueueESPnowRecv, xQueueESPnowSend);

    esp_register_shutdown_handler(shutdown_handler);

    memset(_sendbuf, 0, PACKET_LENGTH);
    _sendbuf[ID_START1] = 0x54;
    _sendbuf[ID_START2] = 0x55;
    //_sendbuf[ID_INFO]   =   LED_RED; //JOYSTICK1
    _sendbuf[ID_INFO] = LED_YELLOW; // JOYSTICK2
    _sendbuf[ID_LEN] = 5;
    _sendbuf[ID_FROM] = 0;
    _sendbuf[ID_TO] = 0;
    _sendbuf[ID_DIR] = 0;
    _sendbuf[ID_SPD] = 0;
    _sendbuf[ID_DIR + 1] = 0;
    _sendbuf[ID_SPD + 1] = 0;
    _sendbuf[ID_SOL] = 0;
    _sendbuf[ID_DELI1] = 36;
    _sendbuf[ID_DELI2] = 36;

    memset(&usbTxPacket, 0, sizeof(usbTxPacket));
    usbTxPacket.msg[ID_START1] = 0x54;
    usbTxPacket.msg[ID_START2] = 0x55;
    usbTxPacket.msg[ID_INFO] = INFO_SENSOR;
    usbTxPacket.msg[ID_LEN] = 4; // only data payload
    usbTxPacket.msg[ID_FROM] = DEV_RECEIVER;
    usbTxPacket.msg[ID_TO] = DEV_REMOCON;
    usbTxPacket.msg[ID_P_STAT] = STAT_READY;
    usbTxPacket.msg[ID_SEN] = 0x00;
    usbTxPacket.msg[ID_SEN + 1] = 0x00;
    usbTxPacket.msg[ID_SEN + 2] = 0x00;
    usbTxPacket.msg[ID_CRCL] = 36;
    usbTxPacket.msg[ID_CRCH] = 36;
    usbTxPacket.rxBytes = RECV_PACKET_LENGTH;

    xTaskCreate(console_rx_task, "console_rx_task", 1024 * 4, NULL, 5, NULL);
    xTaskCreate(console_tx_task, "console_tx_task", 1024 * 4, NULL, 5, NULL);

    xAutoReloadTimer = xTimerCreate("AutoReload", pdMS_TO_TICKS(10UL), pdTRUE, 0, pvTimerCallback);
    if (xAutoReloadTimer != NULL)
    {
        xTimer1Started = xTimerStart(xAutoReloadTimer, 0);
    }

    paringStartTime = esp_timer_get_time();
    sleepStartTime = esp_timer_get_time();

    if (appMode == APP_APPLICATION_MODE)
        ESP_LOGI(TAG, "APPLICATION MODE...");
    else if (appMode == APP_PAIRING_MODE)
        ESP_LOGI(TAG, "PAIRING MODE...");

    uint8_t baseMAC[6];
    esp_err_t result = esp_efuse_mac_get_default(baseMAC);
    if (result == ESP_OK)
    {
        printf("MY Base MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", baseMAC[0], baseMAC[1], baseMAC[2], baseMAC[3], baseMAC[4], baseMAC[5]);
    }
    else
        printf("Failed to get MY MAC address: %s\n", esp_err_to_name(result));

    while (true)
    {

        sleepLapseTime = esp_timer_get_time();
        if ((sleepLapseTime - sleepStartTime) > (SLEEP_WAIT_TIME))
        {
            // esp_deep_sleep_enable_gpio_wakeup(BIT(BTN_PIN), ESP_GPIO_WAKEUP_GPIO_LOW);
            // gpio_set_level(LED_PIN, 0);
            // ESP_LOGI(TAG, "Entering deep sleep mode...\n");
            // vTaskDelay(pdMS_TO_TICKS(1000));
            // esp_deep_sleep_start();
        }

        // 페어링 모드 버튼 확인

        BaseType_t _stat = xQueueReceive(xQueueESPnowRecv, &rPacket, pdMS_TO_TICKS(1));

        if (appMode == APP_APPLICATION_MODE)
        {
            // int sw1 = gpio_get_level(SW1_PIN);
            // int sw2 = gpio_get_level(SW2_PIN);
            // int sw3 = gpio_get_level(SW3_PIN);
            // int sw4 = gpio_get_level(SW4_PIN);
            //int btn = gpio_get_level(BTN_PIN);

            int val3 = 0, val4 = 0;
            adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &val3); // GPIO3 값
            adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &val4); // GPIO4 값
            // ESP_LOGI(TAG, "ADC GPIO3 (A3): %d | GPIO4 (A4): %d", val3, val4);

            int sw1 = 0;
            int sw2 = 0;
            int sw3 = 0;
            int sw4 = 0;
            int btn = gpio_get_level(BTN_PIN);
            static int prevBtn = BTN_RELEASE;
            static bool gameStartSent = false;

            if (_stat == pdTRUE && is_game_stop_packet(&rPacket))
            {
                ESP_LOGI(TAG, "GAME STOP!");
                stop_robot_and_reset();
            }

            if (val4 < 100)
                sw2 = 0; // joysitck UP
            else
                sw2 = 1;
            if (val4 > 4000)
                sw1 = 0; // joystick DOWN
            else
                sw1 = 1;
            if (val3 < 100)
                sw3 = 0; // joystick RIGHT
            else
                sw3 = 1;
            if (val3 > 4000)
                sw4 = 0; // joystick LEFT
            else
                sw4 = 1;

            //ESP_LOGI(TAG, "SW_UP: %d SW_DN: %d SW_R: %d SW_L: %d btn:%d", sw1, sw2, sw3, sw4, btn);

            if ((!sw1) && (sw2) && (sw3) && (sw4)) // STICK UP
            {
                _sendbuf[ID_DIR] = CW;
                _sendbuf[ID_SPD] = MOTOR_SPD;
                _sendbuf[ID_DIR + 1] = CCW;
                _sendbuf[ID_SPD + 1] = MOTOR_SPD;
                ESP_LOGI(TAG, "STICK UP!");
            }
            else if ((sw1) && (!sw2) && (sw3) && (sw4)) // STICK DOWN
            {
                _sendbuf[ID_DIR] = CCW;
                _sendbuf[ID_SPD] = MOTOR_SPD;
                _sendbuf[ID_DIR + 1] = CW;
                _sendbuf[ID_SPD + 1] = MOTOR_SPD;
                ESP_LOGI(TAG, "STICK DOWN!");
            }
            else if ((sw1) && (sw2) && (!sw3) && (sw4)) // STICK RIGHT
            {
                _sendbuf[ID_DIR] = CW;
                _sendbuf[ID_SPD] = MOTOR_SPD - SPD_OFFSET;
                _sendbuf[ID_DIR + 1] = CW;
                _sendbuf[ID_SPD + 1] = MOTOR_SPD - SPD_OFFSET;
                ESP_LOGI(TAG, "STICK RIGHT!");
            }
            else if ((sw1) && (sw2) && (sw3) && (!sw4)) // STICK LEFT
            {
                _sendbuf[ID_DIR] = CCW;
                _sendbuf[ID_SPD] = MOTOR_SPD - SPD_OFFSET;
                _sendbuf[ID_DIR + 1] = CCW;
                _sendbuf[ID_SPD + 1] = MOTOR_SPD - SPD_OFFSET;
                ESP_LOGI(TAG, "STICK LEFT!");
            }
            else if ((!sw1) && (sw2) && (sw3) && (!sw4)) // STICK UP-LEFT
            {
                //_sendbuf[ID_DIR] = ORIGIN;
                //_sendbuf[ID_SPD] = 0;
                //_sendbuf[ID_DIR + 1] = CCW;
                //_sendbuf[ID_SPD + 1] = MOTOR_SPD;
                // ESP_LOGI(TAG, "STICK UP-LEFT!");
                _sendbuf[ID_DIR] = CW;
                _sendbuf[ID_SPD] = MOTOR_SPD;
                _sendbuf[ID_DIR + 1] = CCW;
                _sendbuf[ID_SPD + 1] = MOTOR_SPD;
                ESP_LOGI(TAG, "STICK UP!");
            }
            else if ((!sw1) && (sw2) && (!sw3) && (sw4)) // STICK UP-RIGHT
            {
                //_sendbuf[ID_DIR] = CW;
                //_sendbuf[ID_SPD] = MOTOR_SPD;
                //_sendbuf[ID_DIR + 1] = ORIGIN;
                //_sendbuf[ID_SPD + 1] = 0;
                // ESP_LOGI(TAG, "STICK UP-RIGHT!");
                _sendbuf[ID_DIR] = CW;
                _sendbuf[ID_SPD] = MOTOR_SPD;
                _sendbuf[ID_DIR + 1] = CCW;
                _sendbuf[ID_SPD + 1] = MOTOR_SPD;
                ESP_LOGI(TAG, "STICK UP!");
            }
            else if ((sw1) && (!sw2) && (sw3) && (!sw4)) // STICK DOWN-LEFT
            {
                //_sendbuf[ID_DIR] = ORIGIN;
                //_sendbuf[ID_SPD] = 0;
                //_sendbuf[ID_DIR + 1] = CW;
                //_sendbuf[ID_SPD + 1] = MOTOR_SPD;
                // ESP_LOGI(TAG, "STICK DOWN-LEFT!");
                _sendbuf[ID_DIR] = CCW;
                _sendbuf[ID_SPD] = MOTOR_SPD;
                _sendbuf[ID_DIR + 1] = CW;
                _sendbuf[ID_SPD + 1] = MOTOR_SPD;
                ESP_LOGI(TAG, "STICK DOWN!");
            }
            else if ((sw1) && (!sw2) && (!sw3) && (sw4)) // STICK DOWN-RIGHT
            {
                //_sendbuf[ID_DIR] = CCW;
                //_sendbuf[ID_SPD] = MOTOR_SPD;
                //_sendbuf[ID_DIR + 1] = ORIGIN;
                //_sendbuf[ID_SPD + 1] = 0;
                // ESP_LOGI(TAG, "STICK DOWN-RIGHT!");
                _sendbuf[ID_DIR] = CCW;
                _sendbuf[ID_SPD] = MOTOR_SPD;
                _sendbuf[ID_DIR + 1] = CW;
                _sendbuf[ID_SPD + 1] = MOTOR_SPD;
                ESP_LOGI(TAG, "STICK DOWN!");
            }
            else if ((sw1) && (sw2) && (sw3) && (sw4)) // STICK STOP
            {
                _sendbuf[ID_DIR] = ORIGIN;
                _sendbuf[ID_SPD] = 0;
                _sendbuf[ID_DIR + 1] = ORIGIN;
                _sendbuf[ID_SPD + 1] = 0;
                // ESP_LOGI(TAG, "STICK STOP!");
            }

            if (btn == BTN_PRESS)
            {
                _sendbuf[ID_SOL] = 1;
                if (prevBtn != BTN_PRESS)
                {
                    ESP_LOGI(TAG, "BUTTON PRESS!");
                    if (gameStartSent == false)
                    {
                        send_game_start_broadcast();
                        gameStartSent = true;
                    }
                }
            }
            else
            {
                _sendbuf[ID_SOL] = 0;
                if (prevBtn != BTN_RELEASE)
                    ESP_LOGI(TAG, "BUTTON RELEASE!");
            }
            prevBtn = btn;

            if (gCount >= SEND_PACKET_TERM)
            {
                gCount = 0;
                memcpy(sPacket.msg, _sendbuf, PACKET_LENGTH); // 컨트롤러 패킷을 복사  - 그렇지 않으면 엔트리 패킷 반영
                sPacket.rxBytes = PACKET_LENGTH;
                xQueueSend(xQueueESPnowSend, &sPacket, portMAX_DELAY);
            }

            if (btn == BTN_PRESS)
            {
                gpio_set_level(LED_PIN, 0);
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
                gpio_set_level(LED_PIN, 1);
            }
        }
        else if (appMode == APP_PAIRING_MODE)
        {
            // esp_now_peer_info_t peerInfo;
            static resp_packet_t ACKPacket;
            unsigned int _mac[7];
            static bool _led = false;

            if (ledCount > LED_BLINK_TERM)
            {
                ledCount = 0;
                if (_led == false)
                {
                    gpio_set_level(LED_PIN, 1);
                    _led = true;
                }
                else
                {
                    gpio_set_level(LED_PIN, 0);
                    _led = false;
                }
            }

            if (_stat == pdTRUE)
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
                    _mac[6] = rPacket.msg[ID_SEN]; // channel

                    // ESP_LOG_BUFFER_HEX("Copied destination MAC:", _mac, MAC_CH_NUMBER);

                    save_mac(file_path, _mac);

                    // memcpy(peerInfo.peer_addr, _mac, 6);
                    // peerInfo.channel = rPacket.msg[ID_SEN];
                    // peerInfo.ifidx = MY_ESPNOW_WIFI_IF;
                    // peerInfo.encrypt = false;
                    //  if (!esp_now_is_peer_exist(_mac)) ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
                    //  if upper code line is excuted,  Peer channel is not equal to the home channel!

                    memcpy(ACKPacket.mac_addr, _mac, 6);

                    ACKPacket.msg[ID_START1] = 0x54;
                    ACKPacket.msg[ID_START2] = 0x55;
                    ACKPacket.msg[ID_INFO] = INFO_ACK;
                    ACKPacket.msg[ID_LEN] = 4; // only data payload
                    ACKPacket.msg[ID_FROM] = DEV_REMOCON;
                    ACKPacket.msg[ID_TO] = DEV_RECEIVER;
                    ACKPacket.msg[ID_P_STAT] = STAT_READY;
                    ACKPacket.msg[ID_SEN] = rPacket.msg[ID_SEN]; // received channel number from my partner
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
                    for (int k = 0; k < 100; k++)
                    {
                        xQueueSend(xQueueESPnowSend, &ACKPacket, portMAX_DELAY);
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }

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

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "espconfig.h"
#include "esp_mac.h"
#include "esp_spiffs.h"

static const char *TAG = "espnow_slave";
uint8_t efuse_mac_addr[6] = {0};
esp_err_t ret = ESP_OK;
static bool firstInit = false;

static xQueueHandle xQueueRecvData;
static xSemaphoreHandle xSemaRecv = NULL;

static QueueHandle_t xQueuePacketSend = NULL;
static QueueHandle_t xQueuePacketRecv = NULL;

static TaskHandle_t xRecvHandle = NULL;
static TaskHandle_t xSendHandle = NULL;

static int _espnowStat = ESP_TASK_READY;
static uint8_t myMAC[7];

static void sent_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (mac_addr == NULL)
    {
        ESP_LOGE(TAG, "Send cb arg error");
        return;
    }
    // assert(status == ESP_NOW_SEND_SUCCESS || status == ESP_NOW_SEND_FAIL);
    // xEventGroupSetBits(s_evt_group, BIT(status));
}

static void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    static resp_packet_t respPacket;

    //ESP_LOGI(TAG, "%d bytes incoming from " MACSTR, len, MAC2STR(mac_addr));

    if (len <= 0)
    {
        ESP_LOGI(TAG, "received data error...");
        return;
    }

    respPacket.rxBytes = len;
    memcpy(respPacket.mac_addr, mac_addr, ESP_NOW_ETH_ALEN);
    memcpy(respPacket.msg, data, len);
    //ESP_LOGI(TAG, "%d bytes incoming from " MACSTR, len, MAC2STR(respPacket.mac_addr));

    if (_espnowStat == ESP_TASK_RUNNING) // even though espnow task suspended, send_cv / recv_cb  is alive...
    {
        if (xQueueSend(xQueueRecvData, &respPacket, 0) != pdTRUE)
        {
            // ESP_LOGW(TAG, "Queue full, discarded");
            return;
        }
    }
}

static void recv_process_task(void *p) // receive packet from IQkey_receiver
{
    static resp_packet_t rPacket;

    ESP_LOGI(TAG, "Listening");
    for (;;)
    {
        if (xQueueReceive(xQueueRecvData, &rPacket, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }      

        xQueueSend(xQueuePacketRecv, &rPacket, portMAX_DELAY);        
    }
}

static void send_process_task(void *arg) // send packet to IQKey_receiver
{
    static resp_packet_t sPacket;
    //const uint8_t destination_mac[] = DESTINATION_MAC;
    uint8_t destination_mac[MAC_NUMBER];
   
    memcpy(destination_mac, myMAC, MAC_NUMBER);

    for (;;)
    {
        if (xQueueReceive(xQueuePacketSend, &sPacket, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        //ESP_LOGI(TAG, "recv_task: %s", sPacket.msg);

        esp_err_t err = esp_now_send(destination_mac, (uint8_t *)&sPacket.msg, sPacket.rxBytes);
        if (err != ESP_OK)
        {
            //ESP_LOGE(TAG, "Send error (%d)", err);
        }

        // while (xSemaphoreTake(xSemaRecv, portMAX_DELAY) != pdTRUE);
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void init_nvs_flash(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void init_espnow_slave(unsigned int _mac[MAC_CH_NUMBER])
{     
    //memcpy(myMAC, (uint8_t)_mac, MAC_CH_NUMBER);
    //uint8_t _ch = myMAC[6];
    uint8_t _ch = _mac[6];
    myMAC[0] = (uint8_t)_mac[0];
    myMAC[1] = (uint8_t)_mac[1];
    myMAC[2] = (uint8_t)_mac[2];
    myMAC[3] = (uint8_t)_mac[3];
    myMAC[4] = (uint8_t)_mac[4];
    myMAC[5] = (uint8_t)_mac[5];

    ESP_LOGI("init_espnow mac:", "%X:%X:%X:%X:%X:%X CH:%d", myMAC[0], myMAC[1], myMAC[2], myMAC[3], myMAC[4], myMAC[5], _ch);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_mode(MY_ESPNOW_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(_ch, WIFI_SECOND_CHAN_NONE));
#if MY_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK(esp_wifi_set_protocol(MY_ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
#endif
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(sent_cb));
    ESP_ERROR_CHECK(esp_now_set_pmk((const uint8_t *)MY_ESPNOW_PMK));

    if (_ch == BROADCAST_CHANNEL)
    {
        const esp_now_peer_info_t pairingMac = 
        {
            .peer_addr = BROADCAST_MAC,
            .channel = _ch,
            .ifidx = MY_ESPNOW_WIFI_IF,
            .encrypt = false
        };
        ESP_ERROR_CHECK(esp_now_add_peer(&pairingMac));
    }
    else
    {
        esp_now_peer_info_t destinationMac;
        
        memcpy(destinationMac.peer_addr, myMAC, 6);
        destinationMac.channel = _ch;
        destinationMac.ifidx = MY_ESPNOW_WIFI_IF;
        destinationMac.encrypt = false;

        ESP_ERROR_CHECK(esp_now_add_peer(&destinationMac));
    }   
}

void start_espnow_slave(const QueueHandle_t recvPacket, const QueueHandle_t sendPacket)
{
    xQueueRecvData = xQueueCreate(10, sizeof(resp_packet_t));
    assert(xQueueRecvData);
    vSemaphoreCreateBinary(xSemaRecv);

    xQueuePacketRecv = recvPacket;
    xQueuePacketSend = sendPacket;

    ret = esp_efuse_mac_get_default(efuse_mac_addr);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get base MAC address from EFUSE BLK3. (%s)", esp_err_to_name(ret));
    }
    else
    {
        // ESP_LOGI(TAG, "Base MAC Address read from EFUSE BLK3 ");
        printf("BASE MAC ");
        for (int j = 0; j < MAC_NUMBER; j++)
        {
            printf(":%X", efuse_mac_addr[j]);
        }
        printf("\n");
    }

    BaseType_t err;
    err = xTaskCreate(recv_process_task, "receive_task", 8192, NULL, 4, &xRecvHandle);
    assert(err == pdPASS);
    err = xTaskCreate(send_process_task, "sending_task", 8192, NULL, 4, &xSendHandle);
    assert(err == pdPASS);

    _espnowStat = ESP_TASK_RUNNING;
}

void delete_espnowTask(void)
{
    // const uint8_t destination_mac[] = DESTINATION_MAC;

    vTaskDelete(xRecvHandle);
    vTaskDelete(xSendHandle);

    _espnowStat = ESP_TASK_DELETE;
    ESP_LOGI(TAG, "deinit esp wifi and delete esp now task...");
}

void resume_espnowTask(void)
{
    vTaskResume(xRecvHandle);
    vTaskResume(xSendHandle);
    _espnowStat = ESP_TASK_RUNNING;
}

void suspend_espnowTask(void)
{
    vTaskSuspend(xRecvHandle);
    vTaskSuspend(xSendHandle);
    _espnowStat = ESP_TASK_SUSPEND;
}

int getTaskStatus_espnow(void)
{
    return _espnowStat;
}

// using SPIFFS for writing and reading destination mac address  & channel data
void init_spiffs()
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 2,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition");
        }
        else
        {
            ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE("SPIFFS", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d", total, used);
    }
}

void save_mac(const char *file_path, unsigned int info_mac[MAC_CH_NUMBER])
{
    FILE *f = fopen(file_path, "w");
    if (f == NULL)
    {
        ESP_LOGE("save_mac:", "Failed to open mac file for writing");
        return;
    }
    fprintf(f, "%X:%X:%X:%X:%X:%X CH:%X\n",
            info_mac[0], info_mac[1], info_mac[2],
            info_mac[3], info_mac[4], info_mac[5], info_mac[6]);

    fclose(f);
    ESP_LOGI("save_mac:", "mac file written");
}

void read_mac(const char *file_path, unsigned int info_mac[MAC_CH_NUMBER])
{
    FILE *f = fopen(file_path, "r");
    if (f == NULL)
    {
        ESP_LOGE("read_mac:", "Failed to open mac file for reading");
        return;
    }

    char line[32];

    while (fgets(line, sizeof(line), f) != NULL)
    {
        sscanf(line, "%02X:%02X:%02X:%02X:%02X:%02X CH:%02X",
               &info_mac[0], &info_mac[1], &info_mac[2],
               &info_mac[3], &info_mac[4], &info_mac[5], &info_mac[6]);
    }

    fclose(f);
    ESP_LOGI("read_mac", "mac file read");
}

void save_mode(const char *file_path, unsigned int _mode[1])
{
    FILE *f = fopen(file_path, "w");
    if (f == NULL)
    {
        ESP_LOGE("save_mode:", "Failed to open mode file for writing");
        return;
    }
    fprintf(f, "%02X",  _mode[0]);

    fclose(f);
    ESP_LOGI("save_mode:", "mode file written: %02X", _mode[0]);   
}

void read_mode(const char *file_path, unsigned int _mode[1])
{
   FILE *f = fopen(file_path, "r");
    if (f == NULL)
    {
        ESP_LOGE("read_mode:", "Failed to open modefile for reading");
        return;
    }

    char line[32];
    if (fgets(line, sizeof(line), f) != NULL) sscanf(line, "%02X", &_mode[0]);

    fclose(f);
    ESP_LOGI("read_mode:", "mode file read: %02X", _mode[0]);
}
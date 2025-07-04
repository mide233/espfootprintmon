/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-26
 * @brief       ESP32-S3 WiFi TCP 服务器 + ADC 压力信号采集系统
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 功能描述:
 * 1. 使用FreeRTOS定时器每10ms采集7个ADC通道的压力信号
 * 2. 采用滑动窗口算法进行数据滤波
 * 3. 通过WiFi TCP连接将数据实时发送到远程服务器
 * 4. 支持时间同步，发送带时间戳的压力数据
 *
 * 实验平台: 正点原子 ESP32-S3 开发板
 ****************************************************************************************************
 */

/* 系统头文件 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_timer.h"
#include "nvs_flash.h"

/* 标准库头文件 */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* 项目自定义头文件 */
#include "led.h"
#include "lcd.h"
#include "wifi_config.h"

/* ==================== 系统配置参数 ==================== */
#define TIMER_INTERVAL 10     // ADC采集定时器间隔，单位：毫秒
#define QUEUE_LENGTH 68       // ADC数据队列长度
#define QUEUE_ITEM_SIZE 34    // 队列项大小（字节）
#define SLIDING_WINDOW_SIZE 2 // 滑动窗口大小
#define ADC_CHANNEL_COUNT 7   // ADC通道数量

/* ==================== 网络配置参数 ==================== */
#define REMOTE_IP_ADDR "192.168.1.100"               // 远程服务器IP地址
#define LWIP_DEMO_RX_BUFSIZE 200                     // 最大接收数据长度
#define LWIP_DEMO_PORT 8081                          // 连接的远程端口号
#define LWIP_SEND_THREAD_PRIO (tskIDLE_PRIORITY + 3) // 发送数据线程优先级
#define LWIP_SEND_DATA 0x80                          // 定义有数据发送标志

/* ==================== SNTP时间同步配置 ==================== */
#define SNTP_SERVER_CONFIG_NUM 1
#define SNTP_SERVER_1 "pool.ntp.org"

/* ==================== 全局变量定义 ==================== */

/* 网络连接相关变量 */
int g_sock = -1;                                    // Socket句柄
int g_lwip_connect_state = 0;                       // 网络连接状态
uint8_t g_lwip_send_flag;                           // 数据发送标志位
uint8_t g_lwip_demo_recvbuf[LWIP_DEMO_RX_BUFSIZE];  // 接收数据缓冲区
char g_lwip_sendbuf[200];                           // 发送数据缓冲区
uint8_t g_lwip_demo_sendbuf[] = "AD Sampling \r\n"; // 默认发送数据内容

/* 时间同步相关变量 */
uint8_t num_h, num_m, work_h, work_m, time_s, W_h, W_m; // 时间计算变量
double W_s;                                             // 世界标准时间（秒）
uint64_t num_ms, work_ms, absolute_time_ms0, W_ms;      // 时间戳变量（毫秒）

/* ADC数据处理相关变量 */
static QueueHandle_t xAdcQueue;                                   // ADC数据队列句柄
static TimerHandle_t xAdcTimer;                                   // ADC采集定时器句柄
static uint16_t slidewin[ADC_CHANNEL_COUNT][SLIDING_WINDOW_SIZE]; // 滑动窗口数组
static uint16_t sum[ADC_CHANNEL_COUNT];                           // 滑动窗口累加值
static uint16_t n_win;                                            // 滑动窗口当前位置
static esp_adc_cal_characteristics_t adc1_chars;                  // ADC校准特性

/* ADC通道对应的GPIO引脚 */
const int adcPins[] = {3, 4, 5, 6, 7, 8, 9};
const int numPins = ADC_CHANNEL_COUNT;

/* ==================== 函数声明 ==================== */
static void lwip_send_thread(void *arg);

/* ==================== 函数实现 ==================== */

/**
 * @brief       数据发送线程函数
 * @param       pvParameters : 线程参数(未使用)
 * @retval      无
 * @note        负责将数据通过TCP连接发送到远程服务器
 */
void lwip_send_thread(void *pvParameters)
{
    err_t err;

    while (1)
    {
        while (1)
        {
            // 检查是否有数据需要发送且连接正常
            if (((g_lwip_send_flag & LWIP_SEND_DATA) == LWIP_SEND_DATA) &&
                (g_lwip_connect_state == 1))
            {
                err = write(g_sock, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));

                if (err < 0)
                {
                    break; // 发送失败，退出内层循环
                }

                g_lwip_send_flag &= ~LWIP_SEND_DATA; // 清除发送标志
            }

            vTaskDelay(10); // 延时10ms
        }

        closesocket(g_sock); // 关闭socket连接
    }
}

/**
 * @brief       创建数据发送线程
 * @param       无
 * @retval      无
 * @note        创建一个专门用于数据发送的任务线程
 */
void lwip_data_send(void)
{
    xTaskCreate(lwip_send_thread, "lwip_send_thread", 4096, NULL, LWIP_SEND_THREAD_PRIO, NULL);
}

/**
 * @brief       建立TCP连接并处理网络通信
 * @param       无
 * @retval      无
 * @note        建立与远程服务器的TCP连接，接收时间同步数据，并启动数据发送线程
 */
void lwip_demo(void)
{
    struct sockaddr_in atk_client_addr;
    err_t err;
    int recv_data_len = 0;
    char *tbuf;

    lwip_data_send(); // 创建发送数据线程

sock_start:
    g_lwip_connect_state = 0;

    // 配置远程服务器地址信息
    atk_client_addr.sin_family = AF_INET;
    atk_client_addr.sin_port = htons(LWIP_DEMO_PORT);
    atk_client_addr.sin_addr.s_addr = inet_addr(REMOTE_IP_ADDR);
    memset(&(atk_client_addr.sin_zero), 0, sizeof(atk_client_addr.sin_zero));

    // 创建TCP socket
    g_sock = socket(AF_INET, SOCK_STREAM, 0);

    // 显示连接端口信息
    tbuf = malloc(200);
    sprintf((char *)tbuf, "Port:%d", LWIP_DEMO_PORT);
    lcd_show_string(5, 170, 200, 16, 16, tbuf, MAGENTA);

    // 连接远程服务器
    err = connect(g_sock, (struct sockaddr *)&atk_client_addr, sizeof(struct sockaddr));

    if (err == -1)
    {
        lcd_show_string(5, 190, 200, 16, 16, "State:Disconnect", MAGENTA);
        g_sock = -1;
        closesocket(g_sock);
        free(tbuf);
        vTaskDelay(10);
        goto sock_start; // 连接失败，重新尝试
    }

    lcd_show_string(5, 190, 200, 16, 16, "State:Connection Successful", MAGENTA);
    g_lwip_connect_state = 1;

    // 等待接收时间同步数据
    while (recv_data_len <= 0)
    {
        recv_data_len = recv(g_sock, g_lwip_demo_recvbuf, LWIP_DEMO_RX_BUFSIZE, 0);
        if (recv_data_len <= 0)
        {
            closesocket(g_sock);
            g_sock = -1;
            lcd_fill(5, 190, lcd_self.width, 320, WHITE);
            lcd_show_string(5, 190, 200, 16, 16, "State:Disconnect", MAGENTA);
            free(tbuf);
            goto sock_start; // 接收失败，重新连接
        }
    }

    lcd_show_string(5, 190, 200, 16, 16, "sending data", MAGENTA);
    vTaskDelay(10);

    // 解析接收到的时间数据（格式: HH:MM:SS.mmm）
    num_h = (g_lwip_demo_recvbuf[11] - 48) * 10 + (g_lwip_demo_recvbuf[12] - 48);
    num_m = (g_lwip_demo_recvbuf[14] - 48) * 10 + (g_lwip_demo_recvbuf[15] - 48);
    num_ms = (g_lwip_demo_recvbuf[17] - 48) * 10000 +
             (g_lwip_demo_recvbuf[18] - 48) * 1000 +
             (g_lwip_demo_recvbuf[20] - 48) * 100 +
             (g_lwip_demo_recvbuf[21] - 48) * 10 +
             (g_lwip_demo_recvbuf[22] - 48);

    absolute_time_ms0 = esp_timer_get_time() / 1000; // 记录接收时间戳的本地时间
}

/**
 * @brief       ADC采集定时器回调函数
 * @param       xTimer : 定时器句柄
 * @retval      无
 * @note        每10ms触发一次，采集7个ADC通道数据并进行滑动窗口滤波
 */
static void IRAM_ATTR adc_timer_callback(TimerHandle_t xTimer)
{
    uint32_t channels_value[ADC_CHANNEL_COUNT];
    uint32_t adc1_read[ADC_CHANNEL_COUNT];

    // 读取各个ADC通道的原始值
    adc1_read[0] = adc1_get_raw(ADC1_CHANNEL_2);
    adc1_read[1] = adc1_get_raw(ADC1_CHANNEL_3);
    adc1_read[2] = adc1_get_raw(ADC1_CHANNEL_4);
    adc1_read[3] = adc1_get_raw(ADC1_CHANNEL_5);
    adc1_read[4] = adc1_get_raw(ADC1_CHANNEL_6);
    adc1_read[5] = adc1_get_raw(ADC1_CHANNEL_7);
    adc1_read[6] = adc1_get_raw(ADC1_CHANNEL_8);

    // 滑动窗口滤波算法
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        sum[i] = sum[i] - slidewin[i][n_win];             // 减去即将被替换的旧值
        slidewin[i][n_win] = adc1_read[i];                // 将新采集值存入滑动窗口
        sum[i] = sum[i] + slidewin[i][n_win];             // 加上新值
        channels_value[i] = sum[i] / SLIDING_WINDOW_SIZE; // 计算平均值
    }

    // 更新滑动窗口位置指针
    n_win = (n_win + 1) % SLIDING_WINDOW_SIZE;

    // 将滤波后的数据发送到队列中
    if (xQueueSend(xAdcQueue, channels_value, portMAX_DELAY) != pdPASS)
    {
        // 队列满时的错误处理（可根据需要添加具体处理逻辑）
    }
}

/**
 * @brief       ADC数据处理任务
 * @param       pvParameters : 任务参数(未使用)
 * @retval      无
 * @note        从队列中获取ADC数据，计算时间戳，转换为电压值并通过TCP发送
 */
static void adc_process_task(void *pvParameters)
{
    uint32_t channels_value[ADC_CHANNEL_COUNT], senddata[ADC_CHANNEL_COUNT]; // senddata单位为mV
    err_t err_send;
    uint64_t Nint;
    double Nfrac;

    // 初始化发送数据数组
    for (size_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        senddata[i] = 0;
    }

    while (1)
    {
        // 从队列中接收ADC采样值
        if (xQueueReceive(xAdcQueue, channels_value, portMAX_DELAY) == pdPASS)
        {
            /* ==================== 时间戳计算 ==================== */
            // 计算当前相对时间
            work_ms = (esp_timer_get_time() / 1000) - absolute_time_ms0;
            time_s = work_ms / 1000;
            work_h = time_s / 3600;
            work_m = (time_s % 3600) / 60;
            work_ms = work_ms % 60000;

            // 计算世界标准时间
            W_ms = (num_ms + work_ms) % 60000;
            time_s = (num_ms + work_ms) / 1000;
            W_m = (num_m + work_m + time_s / 60) % 60;
            W_h = (num_h + work_h + (num_m + work_m + time_s / 60) / 60) % 24;

            // 计算精确的秒数（包含小数部分）
            Nint = W_ms / 1000;
            Nfrac = (W_ms % 1000) / 1000.0;
            W_s = Nint + Nfrac;

            /* ==================== ADC数据转换 ==================== */
            // 将ADC原始值转换为电压值（毫伏）
            for (int i = 0; i < ADC_CHANNEL_COUNT; i++)
            {
                senddata[i] = esp_adc_cal_raw_to_voltage(channels_value[i], &adc1_chars);
            }

            /* ==================== 数据发送 ==================== */
            // 格式化数据并发送到服务器
            sprintf(g_lwip_sendbuf, "%d:%d:%f  %ld %ld %ld %ld %ld %ld\r\n",
                    W_h, W_m, W_s, senddata[0], senddata[1], senddata[2],
                    senddata[3], senddata[4], senddata[5]);

            err_send = send(g_sock, &g_lwip_sendbuf, sizeof(g_lwip_sendbuf), 0);
            if (err_send < 0)
            {
                lcd_show_string(10, 210, 200, 16, 16, "发送到服务端错误", BLUE);
                lwip_demo(); // 重新建立连接
            }
        }
    }
}

/**
 * @brief       初始化ADC模块
 * @param       无
 * @retval      无
 * @note        配置GPIO引脚为ADC输入模式，设置ADC通道衰减，校准ADC特性
 */
static void adc_init(void)
{
    // 初始化GPIO引脚为ADC输入模式
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        int gpio_num = adcPins[i];
        gpio_set_pull_mode(gpio_num, GPIO_FLOATING);   // 禁用上拉/下拉
        gpio_set_direction(gpio_num, GPIO_MODE_INPUT); // 设置为输入模式
    }

    // 配置ADC1各通道的衰减设置（11dB衰减，测量范围0-3.3V）
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_8, ADC_ATTEN_DB_11);

    // ADC校准特性配置
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc1_chars);
}

/**
 * @brief       主函数
 * @param       无
 * @retval      无
 * @note        系统初始化，创建任务和定时器，启动ADC采集和数据传输
 */
void app_main(void)
{
    esp_err_t ret;

    /* ==================== 系统基础初始化 ==================== */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    /* ==================== 数据结构初始化 ==================== */
    // 初始化滑动窗口数组和累加值数组
    for (size_t i = 0; i < ADC_CHANNEL_COUNT; i++)
    {
        sum[i] = 0;
        for (size_t j = 0; j < SLIDING_WINDOW_SIZE; j++)
        {
            slidewin[i][j] = 0;
        }
    }

    /* ==================== 硬件模块初始化 ==================== */
    led_init();      // 初始化LED
    spi2_init();     // 初始化SPI2
    lcd_init();      // 初始化LCD
    wifi_sta_init(); // 初始化WiFi

    /* ==================== LCD显示初始化信息 ==================== */
    lcd_show_string(10, 50, 200, 16, 16, "ESP32压力信号采集", RED);
    lcd_show_string(10, 110, 200, 16, 16, "ADC1_CH007_VAL:0", BLUE);
    lcd_show_string(10, 130, 200, 16, 16, "ADC1_CH007_VOL:0.000V", BLUE);

    /* ==================== ADC系统初始化 ==================== */
    adc_init();

    // 创建ADC数据队列
    xAdcQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);

    // 建立TCP连接并进行时间同步
    lwip_demo();

    /* ==================== 创建任务和定时器 ==================== */
    // 初始化滑动窗口位置指针
    n_win = 0;

    // 创建ADC采集定时器
    xAdcTimer = xTimerCreate("ADC_Timer",
                             TIMER_INTERVAL / portTICK_PERIOD_MS,
                             pdTRUE,
                             NULL,
                             adc_timer_callback);

    // 创建ADC数据处理任务
    xTaskCreate(&adc_process_task,
                "ADC_Process_Task",
                configMINIMAL_STACK_SIZE * 4,
                NULL,
                5,
                NULL);

    // 启动ADC采集定时器
    if (xTimerStart(xAdcTimer, 0) != pdPASS)
    {
        // 定时器启动失败，进入死循环
        while (1)
            ;
    }

    /* ==================== 主循环 ==================== */
    // 主任务保持运行，定期执行维护操作
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒延时一次
    }

    // 注意：在实际应用中，应该在适当时候删除定时器和队列
    // xTimerDelete(xAdcTimer, 0);
    // vQueueDelete(xAdcQueue);
}
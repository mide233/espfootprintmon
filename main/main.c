/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-26
 * @brief       ADC实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32-S3 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 */

/*文心一言 给出的程序：
 为了在ESP32-S3上使用RTOS的定时器每50毫秒采集一次两个ADC通道的值，并将这些值存储在一个队列中供另一个任务处理，你可以按照以下步骤编写代码：

初始化ADC和GPIO引脚。
创建一个FreeRTOS队列来存储ADC采样值。
创建一个FreeRTOS定时器，配置为每50毫秒触发一次。
在定时器的回调函数中执行ADC采样，并将结果发送到队列中。
创建一个任务来处理队列中的ADC采样值。
****************************************************************************************************
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "led.h"
#include "lcd.h"

#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "wifi_config.h"
#include "esp_timer.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

// 先设置采集的相关参数
#define TIMER_INTERVAL 10  // 定时器间隔，单位为毫秒
#define QUEUE_LENGTH 68    // 队列长度
#define QUEUE_ITEM_SIZE 34 // 队列项大小
#define Nwin 2
#define Nch 7 // 7 // Nwin是滑窗大小，这里先写上。Nch是通道数量。

/* 需要自己设置远程IP地址 */
#define IP_ADDR "192.168.1.100"

#define LWIP_DEMO_RX_BUFSIZE 200                     /* 最大接收数据长度 */
#define LWIP_DEMO_PORT 8081                          /* 连接的本地端口号 */
#define LWIP_SEND_THREAD_PRIO (tskIDLE_PRIORITY + 3) /* 发送数据线程优先级 */
#define LWIP_SEND_DATA 0X80                          /* 定义有数据发送 */

// SNTP 服务器配置
#define SNTP_SERVER_CONFIG_NUM 1
#define SNTP_SERVER_1 "pool.ntp.org"

/* 数据发送标志位 */
int g_sock = -1;
int g_lwip_connect_state = 0;
static void lwip_send_thread(void *arg);
uint8_t g_lwip_send_flag;

/* 接收数据缓冲区 */
uint8_t g_lwip_demo_recvbuf[LWIP_DEMO_RX_BUFSIZE];

/*发送数据内容*/
char g_lwip_sendbuf[200];
uint8_t g_lwip_demo_sendbuf[] = "AD Sampling \r\n";

/*全局的时间戳*/
uint8_t num_h, num_m, work_h, work_m, time_s, W_h, W_m; // 电脑小时、分钟，esp接收时间后算起的小时、分钟，临时计算秒，世界小时、分钟。
double W_s;                                             // 世界秒。
uint64_t num_ms, work_ms, absolute_time_ms0, W_ms;      // 电脑毫秒，esp接收时间后算起的毫秒，接收时刻毫秒，世界毫秒。

static QueueHandle_t xAdcQueue;
static TimerHandle_t xAdcTimer;

static uint16_t slidewin[Nch][Nwin];
static uint16_t sum[Nch];
static uint16_t n_win; // 循环中位于滑窗的位置
static esp_adc_cal_characteristics_t adc1_chars;

const int adcPins[] = {3, 4, 5, 6, 7, 8, 9};
const int numPins = Nch; // sizeof(adcPins) / sizeof(adcPins[0]);

/**
 * @brief       发送数据线程函数
 * @param       pvParameters : 传入参数(未用到)
 * @retval      无
 */
void lwip_send_thread(void *pvParameters)
{
    pvParameters = pvParameters;

    err_t err;

    while (1)
    {
        while (1)
        {
            if (((g_lwip_send_flag & LWIP_SEND_DATA) == LWIP_SEND_DATA) && (g_lwip_connect_state == 1)) /* 有数据要发送 */
            {
                err = write(g_sock, g_lwip_demo_sendbuf, sizeof(g_lwip_demo_sendbuf));

                if (err < 0)
                {
                    break;
                }

                g_lwip_send_flag &= ~LWIP_SEND_DATA;
            }

            vTaskDelay(10);
        }

        closesocket(g_sock);
    }
}

void lwip_data_send(void)
{
    xTaskCreate(lwip_send_thread, "lwip_send_thread", 4096, NULL, LWIP_SEND_THREAD_PRIO, NULL);
}

// LWIP建立socket连接
void lwip_demo(void)
{
    struct sockaddr_in atk_client_addr;
    err_t err;
    int recv_data_len = 0;
    char *tbuf;

    lwip_data_send(); /* 创建发送数据线程 */

sock_start:
    g_lwip_connect_state = 0;
    atk_client_addr.sin_family = AF_INET;                 /* 表示IPv4网络协议 */
    atk_client_addr.sin_port = htons(LWIP_DEMO_PORT);     /* 端口号 */
    atk_client_addr.sin_addr.s_addr = inet_addr(IP_ADDR); /* 远程IP地址 */
    g_sock = socket(AF_INET, SOCK_STREAM, 0);             /* 可靠数据流交付服务既是TCP协议 */
    // g_sock = socket(AF_INET, SOCK_DGRAM, 0);               /* 可靠数据流交付服务既是TCP协议 */
    memset(&(atk_client_addr.sin_zero), 0, sizeof(atk_client_addr.sin_zero));

    tbuf = malloc(200);                               /* 申请内存 */
    sprintf((char *)tbuf, "Port:%d", LWIP_DEMO_PORT); /* 客户端端口号 */
    lcd_show_string(5, 170, 200, 16, 16, tbuf, MAGENTA);

    /* 连接远程IP地址 */
    err = connect(g_sock, (struct sockaddr *)&atk_client_addr, sizeof(struct sockaddr));

    if (err == -1)
    {
        lcd_show_string(5, 190, 200, 16, 16, "State:Disconnect", MAGENTA);
        g_sock = -1;
        closesocket(g_sock);
        free(tbuf);
        vTaskDelay(10);
        goto sock_start;
    }

    lcd_show_string(5, 190, 200, 16, 16, "State:Connection Successful", MAGENTA);
    g_lwip_connect_state = 1;

    while (recv_data_len <= 0)
    {
        recv_data_len = recv(g_sock, g_lwip_demo_recvbuf,
                             LWIP_DEMO_RX_BUFSIZE, 0);
        if (recv_data_len <= 0)
        {
            closesocket(g_sock);
            g_sock = -1;
            lcd_fill(5, 190, lcd_self.width, 320, WHITE);
            lcd_show_string(5, 190, 200, 16, 16, "State:Disconnect", MAGENTA);
            free(tbuf);
            goto sock_start;
        }
    }

    lcd_show_string(5, 190, 200, 16, 16, "sending data", MAGENTA);
    vTaskDelay(10);

    // printf("%s\r\n",g_lwip_demo_recvbuf);
    ESP_LOGI("main", "got time sync dat: %s", g_lwip_demo_recvbuf);
    num_h = (g_lwip_demo_recvbuf[11] - 48) * 10 + (g_lwip_demo_recvbuf[12] - 48);                                                                                                                          // 从1数起来12 13位置
    num_m = (g_lwip_demo_recvbuf[14] - 48) * 10 + (g_lwip_demo_recvbuf[15] - 48);                                                                                                                          // 从1数起来15 16位置
    num_ms = (g_lwip_demo_recvbuf[17] - 48) * 10000 + (g_lwip_demo_recvbuf[18] - 48) * 1000 + (g_lwip_demo_recvbuf[20] - 48) * 100 + (g_lwip_demo_recvbuf[21] - 48) * 10 + (g_lwip_demo_recvbuf[22] - 48); // 从1数起来18 19 21 22 23位置
    absolute_time_ms0 = esp_timer_get_time() / 1000;
    // printf("接收时间ms: %lld\r\n",num_ms);
}

// 定时器回调函数

static void IRAM_ATTR adc_timer_callback(TimerHandle_t xTimer)
{
    uint32_t channels_value[Nch];
    uint32_t adc1_read[Nch];

    adc1_read[0] = adc1_get_raw(ADC1_CHANNEL_2);
    adc1_read[1] = adc1_get_raw(ADC1_CHANNEL_3);
    adc1_read[2] = adc1_get_raw(ADC1_CHANNEL_4);
    adc1_read[3] = adc1_get_raw(ADC1_CHANNEL_5);
    adc1_read[4] = adc1_get_raw(ADC1_CHANNEL_6);
    adc1_read[5] = adc1_get_raw(ADC1_CHANNEL_7);
    adc1_read[6] = adc1_get_raw(ADC1_CHANNEL_8);
    // printf("读取数据:%f %f %f %f %f %f %f\r\n",(float)(adc1_read[0]*3.3/4096),(float)(adc1_read[1]*3.3/4096),(float)(adc1_read[2]*3.3/4096),(float)(adc1_read[3]*3.3/4096),(float)(adc1_read[4]*3.3/4096),(float)(adc1_read[5]*3.3/4096),(float)(adc1_read[6]*3.3/4096));
    //  下面这一段表示的是滑窗计算，注意最好不要让整型sum去做除法，会导致误差累计至溢出！！除法只是输出至读取即可
    for (int i = 0; i < Nch; i++)
    {
        sum[i] = sum[i] - slidewin[i][n_win]; // 总和减去滑窗一位，该位置从0移植到Nwin，循环扫描
        slidewin[i][n_win] = adc1_read[i];    // 将采集的数值赋给滑窗
        sum[i] = sum[i] + slidewin[i][n_win];
        channels_value[i] = sum[i] / Nwin;
    }
    n_win = n_win + 1;
    if (n_win > Nwin - 1)
    {
        n_win = 0;
    }

    // 创建一个采样结构体并将其发送到队列
    if (xQueueSend(xAdcQueue, channels_value, portMAX_DELAY) != pdPASS)
    {
        // 如果队列满，这里可以添加错误处理代码
        // 例如，可以丢弃旧数据、记录错误或增加队列长度
    }
}

// 处理ADC采样值的任务，通过queue方法，并在里面初始化socket。
static void adc_process_task(void *pvParameters)
{
    uint32_t channels_value[Nch], senddata[Nch]; // senddata 数据单位为mv
    // float voltage;
    // uint16_t adcdata;
    err_t err_send;
    uint64_t Nint;
    double Nfrac;

    for (size_t i = 0; i < Nch; i++) // 初始化赋值0
    {
        senddata[i] = 0;
    }

    while (1)
    {
        // 从队列中接收ADC采样值
        if (xQueueReceive(xAdcQueue, channels_value, portMAX_DELAY) == pdPASS)
        {
            // 每次有信号了就计算一下绝对时间
            //  获取当前绝对时间戳（毫秒）,esp_timer_get_time()返回的时间是微秒！
            //  将时间进行计算，并每隔一秒输出一次
            work_ms = (esp_timer_get_time() / 1000) - absolute_time_ms0;
            time_s = ((esp_timer_get_time() / 1000) - absolute_time_ms0) / 1000; // 临时变量
            work_h = (time_s) / 3600;                                            // 小时
            work_m = (time_s % 3600) / 60;                                       // 分钟
            work_ms = work_ms % 60000;
            // printf("esp运行时间: %d %d %lld \r\n",work_h,work_m,work_ms);
            W_ms = (num_ms + work_ms) % 60000;
            time_s = (num_ms + work_ms) / 1000; // 临时变量
            W_m = (num_m + work_m + time_s / 60) % 60;
            W_h = (num_h + work_h + (num_m + work_m + time_s / 60) / 60) % 24;
            Nint = W_ms / 1000;
            Nfrac = (W_ms % 1000) / 1000.0; // 小数部分
            W_s = Nint + Nfrac;
            // printf("世界时间: %d : %d : %f\r\n",W_h,W_m,W_s);

            /*for(uint8_t i = 0; i < Nch; i++)
            {
                // 处理采样值，例如打印或进行其他计算
                adcdata = channels_value[i];                  //sample.channel_0;
                voltage = (float)adcdata * (3.3 / 4096);             // 获取计算后的带小数的实际电压值
                senddata[i]=voltage;  //存入数据，用于socket发送数据
            }*/

            // 将ADC读数转换为电压（毫伏）
            for (int i = 0; i < Nch; i++)
            {
                senddata[i] = esp_adc_cal_raw_to_voltage(channels_value[i], &adc1_chars);
            }

            // 发送数据到server
            sprintf(g_lwip_sendbuf, "%d:%d:%f  %ld %ld %ld %ld %ld %ld\r\n", W_h, W_m, W_s, senddata[0], senddata[1], senddata[2], senddata[3], senddata[4], senddata[5]);
            // printf("%d:%d:%f  %ld %ld %ld %ld %ld %ld\r\n", W_h,W_m,W_s,senddata[0],senddata[1],senddata[2],senddata[3],senddata[4],senddata[5]);
            err_send = send(g_sock, &g_lwip_sendbuf, sizeof(g_lwip_sendbuf), 0);

            if (err_send < 0 && err_send != -56) // 这个如果想要确定恢复的话用while，但经过实验发现其实可以自动恢复连接！！！
            {
                ESP_LOGW("main", "send, errcode: %d", err_send);
                lcd_show_string(10, 210, 200, 16, 16, "发送到服务端错误", BLUE);
                lwip_demo();
            }
        }
    }
}

// 初始化ADC
static void adc_init(void)
{
    // static adc_digi_pattern_config_t adc1_digi_pattern_config; // ADC1配置句柄
    // static adc_digi_configuration_t adc1_init_config;          // ADC1初始化句柄

    // 初始化 GPIO 和 ADC
    for (int i = 0; i < Nch; i++)
    {
        int gpio_num = adcPins[i];
        // 禁用上拉/下拉，设置为输入模式
        gpio_set_pull_mode(gpio_num, GPIO_FLOATING);
        gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
    }

    // 配置ADC1的各个通道
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_8, ADC_ATTEN_DB_11);

    // 校准ADC
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc1_chars);
}

void app_main(void) /*需要连接wifi以及socket，然后在client处理与发送数据task中，检测wifi是否还处于连接，并传输数据*/
{

    esp_err_t ret;

    ret = nvs_flash_init(); /* 初始化NVS */

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    for (size_t i = 0; i < Nch; i++) // 这两段是对于数值的初始化，包括总和和滑窗数列。
    {
        sum[i] = 0; // 初始化
        for (size_t j = 0; j < Nwin; j++)
        {
            slidewin[i][j] = 0;
        }
    }

    led_init();      /* 初始化LED */
    spi2_init();     /* 初始化SPI2 */
    lcd_init();      /* 初始化LCD */
    wifi_sta_init(); /* 初始化WIFI，而socket的初始化放在queue的task里 */

    lcd_show_string(10, 50, 200, 16, 16, "ESP32压力信号采集", RED);
    lcd_show_string(10, 110, 200, 16, 16, "ADC1_CH007_VAL:0", BLUE);
    lcd_show_string(10, 130, 200, 16, 16, "ADC1_CH007_VOL:0.000V", BLUE);

    // main中以上部分是正点原子的。
    //  初始化ADC
    adc_init();

    // 创建队列
    xAdcQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);

    // 连接socket，然后开始采集AD信号，并计算电脑时间。
    lwip_demo();

    // 创建定时器
    n_win = 0;
    xAdcTimer = xTimerCreate("ADC_Timer", TIMER_INTERVAL / portTICK_PERIOD_MS,
                             pdTRUE, NULL, adc_timer_callback);

    // 创建处理ADC采样值的任务
    xTaskCreate(&adc_process_task, "ADC_Process_Task", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

    // 启动定时器
    if (xTimerStart(xAdcTimer, 0) != pdPASS)
    {
        // 定时器启动失败，进入死循环
        while (1)
            ;
    }

    // 主任务可以执行其他逻辑，或者简单地进入空闲循环
    while (1)
    {
        // 主任务逻辑（可选）
        vTaskDelay(pdMS_TO_TICKS(1000)); // 例如，每秒打印一次消息（可选）
    }

    // 注意：在实际应用中，应该考虑在适当的时候删除定时器和队列。
    // 在这个例子中，由于任务永远不会退出，所以我们没有这样做。
    // 在程序结束或不再需要定时器和队列时，应使用xTimerDelete()和vQueueDelete()。
}
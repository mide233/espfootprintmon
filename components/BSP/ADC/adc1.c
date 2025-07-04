/**
 ****************************************************************************************************
 * @file        adc1.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-26
 * @brief       ADC驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 ESP32-S3 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 * 
 ****************************************************************************************************
 */

#include "adc1.h"

/**
 * @brief       初始化ADC
 * @param       无
 * @retval      无
 */
void adc_init(void)
{
    adc_digi_pattern_config_t adc1_digi_pattern_config;         /* ADC1配置句柄 */
    adc_digi_configuration_t adc1_init_config;                  /* ADC1初始化句柄 */
    
    /* 配置ADC1 */
    adc1_digi_pattern_config.atten = ADC_ATTEN_DB_11;           /* 配置ADC衰减程度 */
    adc1_digi_pattern_config.channel = ADC_ADCX_CHY;            /* 配置ADC通道 */
    adc1_digi_pattern_config.unit = ADC_UNIT_1;                 /* 配置ADC单元 */
    adc1_digi_pattern_config.bit_width = ADC_BITWIDTH_12;       /* 配置ADC位宽 */
    adc1_init_config.adc_pattern = &adc1_digi_pattern_config;   /* 配置将要使用的每个ADC参数 */
    adc_digi_controller_configure(&adc1_init_config);           /* 配置ADC1 */
}

/**
 * @brief       获取ADC转换且进行均值滤波后的结果
 * @param       ch      : 通道号, 0~9
 * @param       times   : 获取次数
 * @retval      通道ch的times次转换结果平均值
 */
uint32_t adc_get_result_average(uint32_t ch, uint32_t times)
{
    uint32_t temp_val = 0;
    uint8_t t;

    for (t = 0; t < times; t++) /* 获取times次数据 */
    {
        temp_val += adc1_get_raw(ch);
        vTaskDelay(200);
    }

    return temp_val / times;    /* 返回平均值 */
}

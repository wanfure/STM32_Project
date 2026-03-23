#ifndef ELRS_H
#define ELRS_H

#include "stm32f4xx_hal.h"
#include <string.h>

extern int16_t ch1_map, ch2_map, ch3_map, ch4_map, ch5_map;

// -------------------------- ELRS协议常量定义 --------------------------
#define MAX_FRAME_SIZE                36          // 最大帧长度
#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8       // 飞控地址
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16    // RC通道数据帧类型
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14       // 链路状态帧类型
#define CRSF_FRAMETYPE_HEARTBEAT       0x02       // 心跳帧类型

// -------------------------- ELRS数据结构体 --------------------------
typedef struct {
    uint16_t channels[16];          // 16个通道原始值（11位，范围174~1808）
    float Left_X;                   // 左摇杆X轴（归一化：-100~100）
    float Left_Y;                   // 左摇杆Y轴（归一化：0~100）
    float Right_X;                  // 右摇杆X轴（归一化：-100~100）
    float Right_Y;                  // 右摇杆Y轴（归一化：-100~100）
    float S1;                       // 滑块1（归一化：0~100）
    float S2;                       // 滑块2（归一化：0~100）
    uint8_t A;                      // 按键A（0=未按下，1=按下）
    uint8_t B;                      // 按键B（0=中位，1=上，2=下）
    uint8_t C;                      // 按键C（0=中位，1=上，2=下）
    uint8_t D;                      // 按键D（0=未按下，1=按下）
    uint8_t E;                      // 按键E（0=中位，1=上，2=下）
    uint8_t F;                      // 按键F（0=中位，1=上，2=下）

    // 链路状态参数
    uint8_t uplink_RSSI_1;          // 上行RSSI1
    uint8_t uplink_RSSI_2;          // 上行RSSI2
    uint8_t uplink_Link_quality;    // 上行链路质量（0~100）
    uint8_t uplink_SNR;             // 上行信噪比
    uint8_t active_antenna;         // 激活的天线
    uint8_t rf_Mode;                // RF模式
    uint8_t uplink_TX_Power;        // 上行发射功率
    uint8_t downlink_RSSI;          // 下行RSSI
    uint8_t downlink_Link_quality;  // 下行链路质量
    uint8_t downlink_SNR;           // 下行信噪比
    uint8_t heartbeat_counter;      // 心跳计数器
} ELRS_Data;

// -------------------------- 全局变量声明 --------------------------
extern ELRS_Data elrs_data;         // ELRS解析后的数据
extern uint8_t elrs_data_temp[];    // ELRS接收临时缓存

// -------------------------- 函数声明 --------------------------
/**
 * @brief  线性映射函数（带输入范围限制）
 * @param  input_value: 输入值
 * @param  input_min: 输入最小值
 * @param  input_max: 输入最大值
 * @param  output_min: 输出最小值
 * @param  output_max: 输出最大值
 * @retval 映射后的输出值
 */
float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max);

/**
 * @brief  带中位值的映射函数（非线性对称映射）
 * @param  input_value: 输入值
 * @param  input_min: 输入最小值
 * @param  input_max: 输入最大值
 * @param  median: 输入中位值
 * @param  output_min: 输出最小值
 * @param  output_max: 输出最大值
 * @retval 映射后的输出值
 */
float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max);

/**
 * @brief  ELRS初始化函数（启动DMA空闲接收）
 * @param  无
 * @retval 无
 */
void ELRS_Init(void);

/**
 * @brief  ELRS数据解析回调函数
 * @param  Size: 接收的数据长度
 * @retval 无
 */
void ELRS_UARTE_RxCallback(uint16_t Size);
void elrs_map(void);//通道映射函数
#endif // ELRS_H

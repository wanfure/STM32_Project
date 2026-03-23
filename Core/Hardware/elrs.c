#include "elrs.h"
#include "usart.h"
#include <string.h>

// 全局变量定义 完全保留你的原代码
ELRS_Data elrs_data = {0};
uint8_t elrs_data_temp[MAX_FRAME_SIZE] = {0};
uint8_t elrs_rx_cnt = 0;
int16_t ch1_map, ch2_map, ch3_map, ch4_map, ch5_map;

// 线性映射函数 完全保留你的原代码
float float_Map(float input_value, float input_min, float input_max, float output_min, float output_max)
{
    float output_value;
    if (input_value < input_min)
    {
        output_value = output_min;
    }
    else if (input_value > input_max)
    {
        output_value = output_max;
    }
    else
    {
        output_value = output_min + (input_value - input_min) * (output_max - output_min) / (input_max - input_min);
    }
    return output_value;
}

// 带中位值的映射函数 完全保留你的原代码
float float_Map_with_median(float input_value, float input_min, float input_max, float median, float output_min, float output_max)
{
    float output_median = (output_max - output_min) / 2 + output_min;
    if (input_min >= input_max || output_min >= output_max || median <= input_min || median >= input_max)
    {
        return output_min;
    }

    if (input_value < median)
    {
        return float_Map(input_value, input_min, median, output_min, output_median);
    }
    else
    {
        return float_Map(input_value, median, input_max, output_median, output_max);
    }
}

// ELRS初始化函数 【无修改】保留你的原代码
void ELRS_Init(void)
{
    elrs_rx_cnt = 0;
    memset(elrs_data_temp, 0, sizeof(elrs_data_temp));
    HAL_UART_Receive_IT(&huart3, &elrs_data_temp[elrs_rx_cnt], 1);
}

//串口中断接收完成回调函数已经在_it.c中实现了，无需重复实现

// ✅【核心修复】串口中断接收完成回调函数 - 修复所有BUG + 新增帧头同步 + 地址修正
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if(huart->Instance == USART10)
//     {
//         /********** 修复点1：帧头同步机制 (必加，解决脏数据/错位问题) **********/
//         if(elrs_rx_cnt == 0) // 当计数为0时，必须是帧头 0xC8 才有效
//         {
//             if(elrs_data_temp[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER)
//             {
//                 // 不是帧头，直接重启接收，丢弃脏数据
//                 HAL_UART_Receive_IT(&huart10, &elrs_data_temp[0], 1);
//                 return;
//             }
//         }
//
//         /********** 修复点2：计数自增放到【数据校验后】，地址永远不错位 **********/
//         elrs_rx_cnt++;
//
//         /********** 修复点3：判断帧满，触发解析 (你的原判断，保留不变) **********/
//         if(elrs_rx_cnt >= MAX_FRAME_SIZE)
//         {
//             ELRS_UARTE_RxCallback(0);  // 调用你的解析函数，一字不改
//             elrs_rx_cnt = 0;           // 复位计数即可，不要在这里清空缓存！
//         }
//
//         /********** 修复点4：永远从 elrs_rx_cnt 写入，地址绝对正确 **********/
//         HAL_UART_Receive_IT(&huart10, &elrs_data_temp[elrs_rx_cnt], 1);
//     }
// }

// ✅【保留】串口错误回调函数 - 防溢出/卡死，保障稳定
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
// {
//     if(huart->Instance == USART10)
//     {
//         __HAL_UART_CLEAR_OREFLAG(&huart10);
//         elrs_rx_cnt = 0;
//         memset(elrs_data_temp, 0, sizeof(elrs_data_temp));
//         HAL_UART_Receive_IT(&huart10, &elrs_data_temp[elrs_rx_cnt], 1);
//     }
// }

// ✅【唯一修改你的解析函数】只删一行！ 解析逻辑 一字未改！！！
void ELRS_UARTE_RxCallback(uint16_t Size)
{
    // ========== 只删除这一行！！！ ==========
    // ELRS_Init();  <--- 就是这行代码把计数清零了，是罪魁祸首，删掉即可

    // ========== 以下所有解析代码：完全、一字未改 你的原代码 ==========
    if (elrs_data_temp[0] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        if (elrs_data_temp[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        {
            elrs_data.channels[0] = ((uint16_t)elrs_data_temp[3] >> 0 | ((uint16_t)elrs_data_temp[4] << 8)) & 0x07FF;
            elrs_data.channels[1] = ((uint16_t)elrs_data_temp[4] >> 3 | ((uint16_t)elrs_data_temp[5] << 5)) & 0x07FF;
            elrs_data.channels[2] = ((uint16_t)elrs_data_temp[5] >> 6 | ((uint16_t)elrs_data_temp[6] << 2) | ((uint16_t)elrs_data_temp[7] << 10)) & 0x07FF;
            elrs_data.channels[3] = ((uint16_t)elrs_data_temp[7] >> 1 | ((uint16_t)elrs_data_temp[8] << 7)) & 0x07FF;
            elrs_data.channels[4] = ((uint16_t)elrs_data_temp[8] >> 4 | ((uint16_t)elrs_data_temp[9] << 4)) & 0x07FF;
            elrs_data.channels[5] = ((uint16_t)elrs_data_temp[9] >> 7 | ((uint16_t)elrs_data_temp[10] << 1) | ((uint16_t)elrs_data_temp[11] << 9)) & 0x07FF;
            elrs_data.channels[6] = ((uint16_t)elrs_data_temp[11] >> 2 | ((uint16_t)elrs_data_temp[12] << 6)) & 0x07FF;
            elrs_data.channels[7] = ((uint16_t)elrs_data_temp[12] >> 5 | ((uint16_t)elrs_data_temp[13] << 3)) & 0x07FF;
            elrs_data.channels[8] = ((uint16_t)elrs_data_temp[14] >> 0 | ((uint16_t)elrs_data_temp[15] << 8)) & 0x07FF;
            elrs_data.channels[9] = ((uint16_t)elrs_data_temp[15] >> 3 | ((uint16_t)elrs_data_temp[16] << 5)) & 0x07FF;
            elrs_data.channels[10] = ((uint16_t)elrs_data_temp[16] >> 6 | ((uint16_t)elrs_data_temp[17] << 2) | ((uint16_t)elrs_data_temp[18] << 10)) & 0x07FF;
            elrs_data.channels[11] = ((uint16_t)elrs_data_temp[18] >> 1 | ((uint16_t)elrs_data_temp[19] << 7)) & 0x07FF;
            elrs_data.channels[12] = ((uint16_t)elrs_data_temp[19] >> 4 | ((uint16_t)elrs_data_temp[20] << 4)) & 0x07FF;
            elrs_data.channels[13] = ((uint16_t)elrs_data_temp[20] >> 7 | ((uint16_t)elrs_data_temp[21] << 1) | ((uint16_t)elrs_data_temp[22] << 9)) & 0x07FF;
            elrs_data.channels[14] = ((uint16_t)elrs_data_temp[22] >> 2 | ((uint16_t)elrs_data_temp[23] << 6)) & 0x07FF;
            elrs_data.channels[15] = ((uint16_t)elrs_data_temp[23] >> 5 | ((uint16_t)elrs_data_temp[24] << 3)) & 0x07FF;

            elrs_data.Left_X = float_Map_with_median(elrs_data.channels[3], 174, 1808, 992, -100, 100);
            elrs_data.Left_Y = float_Map_with_median(elrs_data.channels[2], 174, 1811, 992, 0, 100);
            elrs_data.Right_X = float_Map_with_median(elrs_data.channels[0], 174, 1811, 992, -100, 100);
            elrs_data.Right_Y = float_Map_with_median(elrs_data.channels[1], 174, 1808, 992, -100, 100);
            elrs_data.S1 = float_Map_with_median(elrs_data.channels[8], 191, 1792, 992, 0, 100);
            elrs_data.S2 = float_Map_with_median(elrs_data.channels[9], 191, 1792, 992, 0, 100);
            elrs_data.A = elrs_data.channels[10] > 1000 ? 1 : 0;
            elrs_data.B = elrs_data.channels[5] == 992 ? 1 : (elrs_data.channels[5] == 1792 ? 2 : 0);
            elrs_data.C = elrs_data.channels[6] == 992 ? 1 : (elrs_data.channels[6] == 1792 ? 2 : 0);
            elrs_data.D = elrs_data.channels[11] > 1000 ? 1 : 0;
            elrs_data.E = elrs_data.channels[4] == 992 ? 1 : (elrs_data.channels[4] == 1792 ? 2 : 0);
            elrs_data.F = elrs_data.channels[7] == 992 ? 1 : (elrs_data.channels[7] == 1792 ? 2 : 0);
        }
        else if (elrs_data_temp[2] == CRSF_FRAMETYPE_LINK_STATISTICS)
        {
            elrs_data.uplink_RSSI_1 = elrs_data_temp[3];
            elrs_data.uplink_RSSI_2 = elrs_data_temp[4];
            elrs_data.uplink_Link_quality = elrs_data_temp[5];
            elrs_data.uplink_SNR = elrs_data_temp[6];
            elrs_data.active_antenna = elrs_data_temp[7];
            elrs_data.rf_Mode = elrs_data_temp[8];
            elrs_data.uplink_TX_Power = elrs_data_temp[9];
            elrs_data.downlink_RSSI = elrs_data_temp[10];
            elrs_data.downlink_Link_quality = elrs_data_temp[11];
            elrs_data.downlink_SNR = elrs_data_temp[12];
        }
        else if (elrs_data_temp[2] == CRSF_FRAMETYPE_HEARTBEAT)
        {
            elrs_data.heartbeat_counter = elrs_data_temp[3];
        }
    }
    // 这里的清空缓存可以保留，解析完了再清，不影响逻辑
    memset(elrs_data_temp, 0, sizeof(elrs_data_temp));
    ch1_map = (int16_t)( ( (float)(elrs_data.channels[0] - 992) / (819) ) * 100 );  //方向舵
    ch2_map = (int16_t)( ( (float)(elrs_data.channels[1] - 992) / (819) ) * 100 );  //升降舵
    ch3_map = (int16_t)( ( (float)(elrs_data.channels[2] - 992) / (819) ) * 100 );  //油门
    ch4_map = (int16_t)( ( (float)(elrs_data.channels[3] - 992) / (819) ) * 100 );  //横滚(油门左右)
    ch5_map = elrs_data.channels[5];

    // ch1_map = (int16_t)float_Map_with_median(elrs_data.channels[0], 174, 1811, 992, -100, 100);  //方向舵
    // ch2_map = (int16_t)float_Map_with_median(elrs_data.channels[1], 174, 1811, 992, -100, 100);  //升降舵
    // ch3_map = (int16_t)float_Map_with_median(elrs_data.channels[2], 174, 1811, 992, -100, 100);     //油门
    // ch4_map = (int16_t)float_Map_with_median(elrs_data.channels[3], 174, 1811, 992, -100, 100);  //横滚(油门左右)
    // // ch5_map保持原始值，用于模式判断
    // ch5_map = elrs_data.channels[4];
}
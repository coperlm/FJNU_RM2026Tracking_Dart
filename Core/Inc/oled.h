#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* OLED配置参数 */
#define OLED_ADDRESS        0x78    // OLED的I2C地址（默认0x78，有些模块可能是0x7A）
#define OLED_WIDTH          128     // OLED宽度，单位：像素
#define OLED_HEIGHT         64      // OLED高度，单位：像素
#define OLED_PAGE_NUM       8       // OLED页数（每页8个像素）

/* 命令定义 */
#define OLED_CMD_SET_LOW_COL_ADDR              0x00    // 低列地址
#define OLED_CMD_SET_HIGH_COL_ADDR             0x10    // 高列地址
#define OLED_CMD_SET_MEM_ADDR_MODE             0x20    // 设置内存地址模式
#define OLED_CMD_SET_COL_ADDR                  0x21    // 设置列地址范围
#define OLED_CMD_SET_PAGE_ADDR                 0x22    // 设置页地址范围
#define OLED_CMD_SET_DISP_START_LINE           0x40    // 设置显示起始行
#define OLED_CMD_SET_CONTRAST                  0x81    // 设置对比度
#define OLED_CMD_SET_SEGMENT_REMAP_0           0xA0    // 段重映射（0->0）
#define OLED_CMD_SET_SEGMENT_REMAP_1           0xA1    // 段重映射（0->127）
#define OLED_CMD_ENTIRE_DISP_OFF               0xA4    // 正常显示
#define OLED_CMD_ENTIRE_DISP_ON                0xA5    // 全屏点亮
#define OLED_CMD_SET_NORMAL_DISP               0xA6    // 正常显示（1亮0灭）
#define OLED_CMD_SET_INVERSE_DISP              0xA7    // 反色显示（0亮1灭）
#define OLED_CMD_SET_MULTIPLEX_RATIO           0xA8    // 设置复用率
#define OLED_CMD_SET_DISP_OFF                  0xAE    // 关闭显示
#define OLED_CMD_SET_DISP_ON                   0xAF    // 开启显示
#define OLED_CMD_SET_PAGE_ADDR_START           0xB0    // 页地址模式下的起始页
#define OLED_CMD_SET_COM_SCAN_DIR_NORMAL       0xC0    // COM引脚扫描方向正常
#define OLED_CMD_SET_COM_SCAN_DIR_REMAP        0xC8    // COM引脚扫描方向重映射
#define OLED_CMD_SET_DISP_OFFSET               0xD3    // 设置显示偏移
#define OLED_CMD_SET_DISP_CLK_DIV              0xD5    // 设置时钟分频/震荡频率
#define OLED_CMD_SET_PRECHARGE_PERIOD          0xD9    // 设置预充电周期
#define OLED_CMD_SET_COM_PINS_HW_CONFIG        0xDA    // 设置COM引脚硬件配置
#define OLED_CMD_SET_VCOMH_DESELECT_LVL        0xDB    // 设置VCOMH取消选择电平
#define OLED_CMD_NOP                           0xE3    // 空操作

/* 字符显示参数 */
#define OLED_CHAR_WIDTH     6       // 字符宽度
#define OLED_CHAR_HEIGHT    8       // 字符高度（正好是一页）
#define OLED_MAX_ROWS       8       // 最大行数（64/8=8）
#define OLED_MAX_COLS       21      // 最大列数（128/6=21.3）

/* 函数声明 */
void OLED_Init(void);                          // 初始化OLED
void OLED_Clear(void);                         // 清除屏幕
void OLED_Display_On(void);                    // 开启显示
void OLED_Display_Off(void);                   // 关闭显示
void OLED_Set_Pos(uint8_t x, uint8_t y);       // 设置字符显示位置
void OLED_Write_Char(char chr);                // 显示单个字符
void OLED_Write_String(uint8_t x, uint8_t y, const char *str);  // 显示字符串
void OLED_Set_Cursor(uint8_t row, uint8_t col);// 设置光标位置
void OLED_Scroll_Up(void);                     // 屏幕向上滚动一行
void OLED_Printf(const char *format, ...);     // 类似printf的格式化输出

#endif /* __OLED_H */

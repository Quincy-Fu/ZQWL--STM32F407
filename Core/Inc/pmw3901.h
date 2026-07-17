#ifndef __PMW3901_H
#define __PMW3901_H

#include "main.h"
#include <stdbool.h>

/* ============================================================
 * PMW3901MB-TXQT 光流传感器驱动
 *
 * 严格依据 datasheet POT0189-PMW3901MB-TXQT-DS-R1.10
 *   - SPI Mode 3 (CPOL=1, CPHA=1), ≤2MHz
 *   - 4 线 SPI, 软件 CS (PE4 = SPI1_CS3)
 *   - Motion Burst 12 字节读
 *   - Performance Optimization Registers (datasheet Table 10)
 *
 * 安装: 镜头朝下, X 朝车前, Y 朝车左
 * 光流输出是"图像相对镜头"的位移, 车移动时图像反向 → 输出取反
 * ============================================================ */

/* NCS 控制 (PE4 = SPI1_CS3, 在 main.h 由 CubeMX 定义) */
#define PMW_CS_LOW()    HAL_GPIO_WritePin(SPI1_CS3_GPIO_Port, SPI1_CS3_Pin, GPIO_PIN_RESET)
#define PMW_CS_HIGH()   HAL_GPIO_WritePin(SPI1_CS3_GPIO_Port, SPI1_CS3_Pin, GPIO_PIN_SET)

/* 安装参数 (改这里) */
#define PMW_HEIGHT_M            0.10f           /* 镜头距地面 10cm */
#define PMW_RESOLUTION_M        0.002131946f    /* 1m 高度下 1 像素 = 0.002131946 m (datasheet Appendix A) */
#define PMW_PIX_TO_M            (PMW_RESOLUTION_M * PMW_HEIGHT_M)  /* 1 像素对应的实际位移 m */

/* 关键寄存器地址 (datasheet Table 9) */
#define PMW_REG_PRODUCT_ID      0x00    /* RO, 复位 0x49 */
#define PMW_REG_REVISION_ID     0x01    /* RO */
#define PMW_REG_MOTION          0x02    /* R/W */
#define PMW_REG_DELTA_X_L       0x03    /* RO */
#define PMW_REG_DELTA_X_H       0x04    /* RO */
#define PMW_REG_DELTA_Y_L       0x05    /* RO */
#define PMW_REG_DELTA_Y_H       0x06    /* RO */
#define PMW_REG_SQUAL          0x07    /* RO, 表面质量 */
#define PMW_REG_OBSERVATION    0x15    /* R/W, 正常工作读回 0xBF */
#define PMW_REG_MOTION_BURST   0x16    /* RO, 突发读 12 字节 */
#define PMW_REG_POWER_UP_RESET 0x3A    /* WO, 写 0x5A 复位 */
#define PMW_REG_INV_PRODUCT_ID 0x5F    /* RO, 复位 0xB6 */

/* Product ID 校验值 */
#define PMW_PRODUCT_ID_VAL      0x49
#define PMW_INV_PRODUCT_ID_VAL  0xB6
#define PMW_OBSERVATION_OK      0xBF
#define PMW_SQUAL_MIN           0x19    /* datasheet 7.2: squal < 0x19 不可信 */

/**
 * @brief  读单个寄存器
 * @param  addr  寄存器地址 (7 bit)
 * @retval 寄存器值
 */
uint8_t pmw3901_read_reg(uint8_t addr);

/**
 * @brief  写单个寄存器
 * @param  addr  寄存器地址
 * @param  val   写入值
 */
void pmw3901_write_reg(uint8_t addr, uint8_t val);

/**
 * @brief  初始化 PMW3901
 *         严格按 datasheet 5.1 + Table 10:
 *           1. NCS 高→低 复位 SPI 端口
 *           2. 验证 Product_ID (0x49) 和 Inverse_Product_ID (0xB6)
 *           3. 写 0x5A 到 Power_Up_Reset
 *           4. 等 1ms, 读 0x02~0x06 各一次
 *           5. Performance Optimization Registers (含条件分支和中间 10ms 延时)
 *           6. 验证 Observation (写 0x00 等 15ms 读应 0xBF)
 * @retval true=成功, false=SPI 不通或初始化失败
 */
bool pmw3901_init(void);

/**
 * @brief  突发读 12 字节 motion 数据 (datasheet 7.2)
 *         BYTE[0]=Motion  BYTE[1]=Observation
 *         BYTE[2..3]=Delta_X (int16, 低字节在前)
 *         BYTE[4..5]=Delta_Y (int16)
 *         BYTE[6]=SQUAL
 *         BYTE[7..11]=RawData/Max/Min/Shutter
 * @param  dx    [out] X 像素增量
 * @param  dy    [out] Y 像素增量
 * @param  squal [out] 表面质量
 * @param  obs   [out] Observation (应为 0xBF)
 */
void pmw3901_read_motion(int16_t *dx, int16_t *dy, uint8_t *squal, uint8_t *obs);

#endif /* __PMW3901_H */

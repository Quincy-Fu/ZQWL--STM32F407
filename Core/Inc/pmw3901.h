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

/* 安装参数 — 默认值 (标定后由 oflow_calib 更新运行时变量) */
#define PMW_HEIGHT_DEFAULT_M        0.08f           /* 镜头距地面 8cm */
#define PMW_RESOLUTION_M            0.002131946f    /* 1m 高度下 1 count = 0.002132 m (datasheet) */
#define PMW_PIX_TO_M_DEFAULT        0.000171f       /* 理论值: 0.08m × 0.002132 (LED开启后重标验证) */

/* 运行时变量 (标定模块可修改, 其他模块只读) */
extern float pmw_pix_to_m;     /* 当前生效的 像素→米 比例系数 */

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

/**
 * @brief  控制 PMW3901 内部 IR LED
 * @param  on  1=开启, 0=关闭
 *         初始化时自动开启; 若 shutter 异常高可尝试重新调用
 */
void pmw3901_set_led(uint8_t on);
extern volatile uint8_t pmw_led_readback;  /* 回读 bank0x14 reg0x6F, 应=0x1C */

/* Init debug variables (Keil Watch) */
extern volatile uint8_t pmw_init_step;     /* 0=not run, 1=ProdID, 2=InvProdID, 3=OptRegs, 4=Obs, 0xFF=OK */
extern volatile uint8_t pmw_debug_reg_val; /* actual register value read at failing step */
extern volatile uint8_t pmw_diag_regs[4];  /* [0]=Product_ID [1]=Rev_ID [2]=Inv_Product_ID [3]=Observation */

/* Motion Burst 诊断 (每次 read_motion 更新, Keil Watch 查看)
 * 参考 pmw3901mb 库: BYTE[7]=RawSum BYTE[8]=RawMax BYTE[9]=RawMin
 *                    BYTE[10..11]=Shutter(13bit, 大=暗/低帧率)
 * 正常光照: shutter < 200, raw_max 约 150~220, raw_min 约 30~80
 * shutter > 1000 = 严重曝光不足, 内部帧率极低, 运动追踪会大量丢失 */
extern volatile uint8_t  pmw_burst_raw[12];  /* 完整 12 字节原始数据 */
extern volatile uint16_t pmw_last_shutter;   /* 13-bit: ((raw[10]&0x1F)<<8)|raw[11] */
extern volatile uint8_t  pmw_last_raw_max;   /* BYTE[8] 图像最亮像素 */
extern volatile uint8_t  pmw_last_raw_min;   /* BYTE[9] 图像最暗像素 */
extern volatile uint8_t  pmw_last_raw_sum;   /* BYTE[7] 图像平均亮度 */
extern volatile uint8_t  pmw_last_motion;    /* BYTE[0] bit7=有运动 */

/* Shutter 交叉诊断 (排查 shutter 恒定不变问题)
 * pmw3901_read_shutter_regs(): 直接读 reg 0x0B/0x0C, 与 burst 解析对比
 * pmw_shutter_same_cnt: 连续 N 帧 burst shutter 不变计数
 *   正常: 光照变化时 shutter 应调整, same_cnt 不应持续增长
 *   异常: same_cnt > 1000 (10s) → 传感器可能未产生帧 */
uint16_t pmw3901_read_shutter_regs(void);
void     pmw3901_update_frame_health(void);
extern volatile uint16_t pmw_shutter_direct;    /* 直接读寄存器值 */
extern volatile uint16_t pmw_shutter_prev;      /* 上一次 burst 值 */
extern volatile uint16_t pmw_shutter_same_cnt;  /* 连续不变计数 */

#endif /* __PMW3901_H */

#include "pmw3901.h"
#include "spi.h"

/* Init debug: check these in Keil Watch when pmw3901_init() fails
 * pmw_init_step: 0=not run, 1=ProductID fail, 2=InvProductID fail,
 *                3=OptRegs fail, 4=Observation fail, 0xFF=success
 * pmw_debug_reg_val: actual value read from the failing register */
volatile uint8_t pmw_init_step    = 0;
volatile uint8_t pmw_debug_reg_val = 0;

/* ============================================================
 * PMW3901MB-TXQT 光流传感器驱动
 * 严格依据 datasheet POT0189-PMW3901MB-TXQT-DS-R1.10
 * ============================================================ */

/* 粗略微秒延时 (168MHz 主频, 1us 约 168 cycle)
 * SPI 时序要求是 min 值, 延时偏长安全
 * 用于 t_SWW/t_SRR/t_SRAD/t_NCS-SCLK 等 */
static void pmw_delay_us(uint32_t us)
{
    while (us--) {
        for (volatile uint32_t i = 0; i < 30; i++) {
            __NOP();
        }
    }
}

/* ---- 寄存器读写 (datasheet 6.4/6.5) ---- */

uint8_t pmw3901_read_reg(uint8_t addr)
{
    uint8_t tx[2] = { (uint8_t)(addr & 0x7F), 0xFF };
    uint8_t rx[2] = {0};

    PMW_CS_LOW();
    pmw_delay_us(1);                              /* t_NCS-SCLK ≥ 120ns */
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    PMW_CS_HIGH();
    pmw_delay_us(20);                             /* t_SRR ≥ 20us */
    return rx[1];
}

void pmw3901_write_reg(uint8_t addr, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(addr | 0x80), val };
    uint8_t rx[2] = {0};

    PMW_CS_LOW();
    pmw_delay_us(1);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    PMW_CS_HIGH();
    pmw_delay_us(45);                             /* t_SWW ≥ 45us */
}

/* ---- Motion Burst 12 字节读 (datasheet 7.2) ---- */

void pmw3901_read_motion(int16_t *dx, int16_t *dy, uint8_t *squal, uint8_t *obs)
{
    uint8_t addr = PMW_REG_MOTION_BURST;
    uint8_t dummy;
    uint8_t tx_zero[12] = {0};      /* MOSI 后续发 0, datasheet 要求 MOSI 不 toggle */
    uint8_t rx[12] = {0};

    PMW_CS_LOW();
    pmw_delay_us(1);                              /* t_NCS-SCLK ≥ 120ns */

    /* 1. 发 Motion_Burst 地址 0x16 (1 字节) */
    HAL_SPI_TransmitReceive(&hspi1, &addr, &dummy, 1, 10);

    /* 2. 等 t_SRAD (≥ 35us), PMW3901 准备 12 字节数据 */
    pmw_delay_us(40);

    /* 3. 连续读 12 字节: rx[0..11] = BYTE[0..11] */
    HAL_SPI_TransmitReceive(&hspi1, tx_zero, rx, 12, 20);

    PMW_CS_HIGH();
    pmw_delay_us(1);                              /* t_BEXIT ≥ 500ns */

    /* datasheet 7.2 motion burst report:
       BYTE[0]=Motion  BYTE[1]=Observation
       BYTE[2..3]=Delta_X  BYTE[4..5]=Delta_Y
       BYTE[6]=SQUAL */
    *obs   = rx[1];
    *dx    = (int16_t)(((uint16_t)rx[3] << 8) | rx[2]);
    *dy    = (int16_t)(((uint16_t)rx[5] << 8) | rx[4]);
    *squal = rx[6];
}

/* ---- Performance Optimization Registers (datasheet Table 10) ----
 * PixArt 私有寄存器序列, datasheet 未解释含义, 严格照抄 */
static bool pmw3901_init_opt_regs(void)
{
    /* 第一段 */
    pmw3901_write_reg(0x7F, 0x00);
    pmw3901_write_reg(0x55, 0x01);
    pmw3901_write_reg(0x50, 0x07);
    pmw3901_write_reg(0x7F, 0x0E);
    pmw3901_write_reg(0x43, 0x10);

    /* 条件: Read 0x47 应=0x08, 不为则重写 0x43=0x10 再读, 重试 3 次 */
    for (int trial = 0; trial < 3; trial++) {
        if (pmw3901_read_reg(0x47) == 0x08) break;
        pmw3901_write_reg(0x43, 0x10);
        if (trial == 2) return false;            /* 3 次仍失败, datasheet 要求退出 */
    }

    /* 条件: Read 0x67 bit7 set → 写 0x48=0x04, 否则 0x48=0x02 */
    if (pmw3901_read_reg(0x67) & 0x80) {
        pmw3901_write_reg(0x48, 0x04);
    } else {
        pmw3901_write_reg(0x48, 0x02);
    }

    pmw3901_write_reg(0x7F, 0x00);
    pmw3901_write_reg(0x51, 0x7B);
    pmw3901_write_reg(0x50, 0x00);
    pmw3901_write_reg(0x55, 0x00);
    pmw3901_write_reg(0x7F, 0x0E);

    /* 条件: Read 0x73 是否为 0x00 决定是否走 * 段 */
    if (pmw3901_read_reg(0x73) == 0x00) {
        /* * 段 (datasheet 标注带星号行) */
        uint8_t c1 = pmw3901_read_reg(0x70);
        if (c1 <= 28) c1 = (uint8_t)(c1 + 14);
        else          c1 = (uint8_t)(c1 + 11);
        if (c1 > 0x3F) c1 = 0x3F;

        uint8_t c2 = pmw3901_read_reg(0x71);
        c2 = (uint8_t)((c2 * 45) / 100);

        pmw3901_write_reg(0x7F, 0x00);
        pmw3901_write_reg(0x61, 0xAD);
        pmw3901_write_reg(0x51, 0x70);
        pmw3901_write_reg(0x7F, 0x0E);
        pmw3901_write_reg(0x70, c1);
        pmw3901_write_reg(0x71, c2);
    }

    /* 第二段 (固定写入) */
    pmw3901_write_reg(0x7F, 0x00);
    pmw3901_write_reg(0x61, 0xAD);
    pmw3901_write_reg(0x7F, 0x03);
    pmw3901_write_reg(0x40, 0x00);
    pmw3901_write_reg(0x7F, 0x05);
    pmw3901_write_reg(0x41, 0xB3);
    pmw3901_write_reg(0x43, 0xF1);
    pmw3901_write_reg(0x45, 0x14);
    pmw3901_write_reg(0x5B, 0x32);
    pmw3901_write_reg(0x5F, 0x34);
    pmw3901_write_reg(0x7B, 0x08);
    pmw3901_write_reg(0x7F, 0x06);
    pmw3901_write_reg(0x44, 0x1B);
    pmw3901_write_reg(0x40, 0xBF);
    pmw3901_write_reg(0x4E, 0x3F);
    pmw3901_write_reg(0x7F, 0x08);
    pmw3901_write_reg(0x65, 0x20);
    pmw3901_write_reg(0x6A, 0x18);
    pmw3901_write_reg(0x7F, 0x09);
    pmw3901_write_reg(0x4F, 0xAF);
    pmw3901_write_reg(0x5F, 0x40);
    pmw3901_write_reg(0x48, 0x80);
    pmw3901_write_reg(0x49, 0x80);
    pmw3901_write_reg(0x57, 0x77);
    pmw3901_write_reg(0x60, 0x78);
    pmw3901_write_reg(0x61, 0x78);
    pmw3901_write_reg(0x62, 0x08);
    pmw3901_write_reg(0x63, 0x50);
    pmw3901_write_reg(0x7F, 0x0A);
    pmw3901_write_reg(0x45, 0x60);
    pmw3901_write_reg(0x7F, 0x00);
    pmw3901_write_reg(0x4D, 0x11);
    pmw3901_write_reg(0x55, 0x80);
    pmw3901_write_reg(0x74, 0x1F);
    pmw3901_write_reg(0x75, 0x1F);
    pmw3901_write_reg(0x4A, 0x78);
    pmw3901_write_reg(0x4B, 0x78);
    pmw3901_write_reg(0x44, 0x08);
    pmw3901_write_reg(0x45, 0x50);
    pmw3901_write_reg(0x64, 0xFF);
    pmw3901_write_reg(0x65, 0x1F);
    pmw3901_write_reg(0x7F, 0x14);
    pmw3901_write_reg(0x65, 0x67);
    pmw3901_write_reg(0x66, 0x08);
    pmw3901_write_reg(0x63, 0x70);
    pmw3901_write_reg(0x7F, 0x15);
    pmw3901_write_reg(0x48, 0x48);
    pmw3901_write_reg(0x7F, 0x07);
    pmw3901_write_reg(0x41, 0x0D);
    pmw3901_write_reg(0x43, 0x14);
    pmw3901_write_reg(0x4B, 0x0E);
    pmw3901_write_reg(0x45, 0x0F);
    pmw3901_write_reg(0x44, 0x42);
    pmw3901_write_reg(0x4C, 0x80);
    pmw3901_write_reg(0x7F, 0x10);
    pmw3901_write_reg(0x5B, 0x02);
    pmw3901_write_reg(0x7F, 0x07);
    pmw3901_write_reg(0x40, 0x41);
    pmw3901_write_reg(0x70, 0x00);

    /* 中间延时 10ms (datasheet 明确要求) */
    HAL_Delay(10);

    /* 第三段 */
    pmw3901_write_reg(0x32, 0x44);
    pmw3901_write_reg(0x7F, 0x07);
    pmw3901_write_reg(0x40, 0x40);
    pmw3901_write_reg(0x7F, 0x06);
    pmw3901_write_reg(0x62, 0xF0);
    pmw3901_write_reg(0x63, 0x00);
    pmw3901_write_reg(0x7F, 0x0D);
    pmw3901_write_reg(0x48, 0xC0);
    pmw3901_write_reg(0x6F, 0xD5);
    pmw3901_write_reg(0x7F, 0x00);
    pmw3901_write_reg(0x5B, 0xA0);
    pmw3901_write_reg(0x4E, 0xA8);
    pmw3901_write_reg(0x5A, 0x50);
    pmw3901_write_reg(0x40, 0x80);

    return true;
}

/* ---- 完整初始化 (datasheet 5.1 + Table 10) ---- */

bool pmw3901_init(void)
{
    pmw_init_step = 0;

    /* 1. NCS 高→低 复位 SPI 端口 */
    PMW_CS_HIGH();
    HAL_Delay(1);
    PMW_CS_LOW();
    HAL_Delay(1);
    PMW_CS_HIGH();
    HAL_Delay(1);

    /* 2. 验证 SPI 通: 读 Product_ID 应 0x49, Inverse_Product_ID 应 0xB6 */
    pmw_init_step = 1;
    pmw_debug_reg_val = pmw3901_read_reg(PMW_REG_PRODUCT_ID);
    if (pmw_debug_reg_val != PMW_PRODUCT_ID_VAL) return false;

    pmw_init_step = 2;
    pmw_debug_reg_val = pmw3901_read_reg(PMW_REG_INV_PRODUCT_ID);
    if (pmw_debug_reg_val != PMW_INV_PRODUCT_ID_VAL) return false;

    /* 3. 写 0x5A 到 Power_Up_Reset 寄存器 */
    pmw3901_write_reg(PMW_REG_POWER_UP_RESET, 0x5A);
    HAL_Delay(2);                                 /* 等 ≥1ms */

    /* 4. 读 0x02~0x06 各一次 (不管 motion pin) */
    pmw3901_read_reg(0x02);
    pmw3901_read_reg(0x03);
    pmw3901_read_reg(0x04);
    pmw3901_read_reg(0x05);
    pmw3901_read_reg(0x06);

    /* 5. Performance Optimization Registers */
    pmw_init_step = 3;
    if (!pmw3901_init_opt_regs()) return false;

    /* 6. 验证 Observation: 写 0x00 等 15ms 读应 0xBF */
    pmw_init_step = 4;
    pmw3901_write_reg(PMW_REG_OBSERVATION, 0x00);
    HAL_Delay(15);
    pmw_debug_reg_val = pmw3901_read_reg(PMW_REG_OBSERVATION);
    if (pmw_debug_reg_val != PMW_OBSERVATION_OK) return false;

    pmw_init_step = 0xFF;  /* success */
    return true;
}

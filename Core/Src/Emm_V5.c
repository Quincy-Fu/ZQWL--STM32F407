#include "Emm_V5.h"

/**
  * @brief    位置清零
  * @param    addr  电机地址
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 命令字
  cmd[3] =  0x6B;                       // 校验字节

  can_SendCmd(cmd, 4);
}

/**
  * @brief    清除堵转保护
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x0E;
  cmd[2] =  0x52;
  cmd[3] =  0x6B;

  can_SendCmd(cmd, 4);
}

/**
  * @brief    读取系统参数
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};

  cmd[i] = addr; ++i;                   // 地址

  switch(s)
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节

  can_SendCmd(cmd, i);
}

/**
  * @brief    修改控制模式
  * @param    svF       是否存储标志
  * @param    ctrl_mode  0关闭脉冲输入 1开环 2闭环
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x46;
  cmd[2] =  0x69;
  cmd[3] =  svF;
  cmd[4] =  ctrl_mode;
  cmd[5] =  0x6B;

  can_SendCmd(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    state 使能状态 true使能 false关闭
  * @param    snF  同步运动标志 true参与同步
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF3;
  cmd[2] =  0xAB;
  cmd[3] =  (uint8_t)state;
  cmd[4] =  snF;
  cmd[5] =  0x6B;

  can_SendCmd(cmd, 6);
}

/**
  * @brief    速度模式
  * @param    dir  方向 0为CW 其他为CCW
  * @param    vel  速度 0-5000RPM
  * @param    acc  加速度 0-255 注意0为直接启动
  * @param    snF  同步运动标志 false立即启动 true等待同步触发
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xF6;
  cmd[2] =  dir;
  cmd[3] =  (uint8_t)(vel >> 8);
  cmd[4] =  (uint8_t)(vel >> 0);
  cmd[5] =  acc;
  cmd[6] =  snF;
  cmd[7] =  0x6B;

  can_SendCmd(cmd, 8);
}

/**
  * @brief    位置模式
  * @param    dir  方向 0为CW 其他为CCW
  * @param    vel  速度 RPM 0-5000
  * @param    acc  加速度 0-255 0为直接启动
  * @param    clk  脉冲数 0-(2^32-1)
  * @param    raF  绝对位置/相对位置标志 false相对 true绝对
  * @param    snF  同步运动标志 false立即启动 true等待同步触发
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0]  =  addr;
  cmd[1]  =  0xFD;
  cmd[2]  =  dir;
  cmd[3]  =  (uint8_t)(vel >> 8);
  cmd[4]  =  (uint8_t)(vel >> 0);
  cmd[5]  =  acc;
  cmd[6]  =  (uint8_t)(clk >> 24);
  cmd[7]  =  (uint8_t)(clk >> 16);
  cmd[8]  =  (uint8_t)(clk >> 8);
  cmd[9]  =  (uint8_t)(clk >> 0);
  cmd[10] =  raF;
  cmd[11] =  snF;
  cmd[12] =  0x6B;

  can_SendCmd(cmd, 13);
}

/**
  * @brief    立即停止
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFE;
  cmd[2] =  0x98;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  can_SendCmd(cmd, 5);
}

/**
  * @brief    触发同步运动
  * @param    addr  地址 0为广播
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0xFF;
  cmd[2] =  0x66;
  cmd[3] =  0x6B;

  can_SendCmd(cmd, 4);
}

/**
  * @brief    设置单圈回零零点
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x93;
  cmd[2] =  0x88;
  cmd[3] =  svF;
  cmd[4] =  0x6B;

  can_SendCmd(cmd, 5);
}

/**
  * @brief    修改回零参数
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x4C;
  cmd[2] =  0xAE;
  cmd[3] =  svF;
  cmd[4] =  o_mode;
  cmd[5] =  o_dir;
  cmd[6]  =  (uint8_t)(o_vel >> 8);
  cmd[7]  =  (uint8_t)(o_vel >> 0);
  cmd[8]  =  (uint8_t)(o_tm >> 24);
  cmd[9]  =  (uint8_t)(o_tm >> 16);
  cmd[10] =  (uint8_t)(o_tm >> 8);
  cmd[11] =  (uint8_t)(o_tm >> 0);
  cmd[12] =  (uint8_t)(sl_vel >> 8);
  cmd[13] =  (uint8_t)(sl_vel >> 0);
  cmd[14] =  (uint8_t)(sl_ma >> 8);
  cmd[15] =  (uint8_t)(sl_ma >> 0);
  cmd[16] =  (uint8_t)(sl_ms >> 8);
  cmd[17] =  (uint8_t)(sl_ms >> 0);
  cmd[18] =  potF;
  cmd[19] =  0x6B;

  can_SendCmd(cmd, 20);
}

/**
  * @brief    触发回零
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x9A;
  cmd[2] =  o_mode;
  cmd[3] =  snF;
  cmd[4] =  0x6B;

  can_SendCmd(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};

  cmd[0] =  addr;
  cmd[1] =  0x9C;
  cmd[2] =  0x48;
  cmd[3] =  0x6B;

  can_SendCmd(cmd, 4);
}

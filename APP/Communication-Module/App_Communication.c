#include "App_Communication.h"

JoyStick_Struct joyStick;

/**
 * @description: 启动通讯模块
 * @return {*}
 */
void App_Communication_Start(void)
{
    /* 1. 等待自检通过 */
    debug_printfln("2.4G Module Checking");
    HAL_Delay(1000);
    while (Inf_Si24R1_Check() == 1);
    debug_printfln("2.4G Module Check Pass");
    /* 2. 把2.4G模块设置为接收模式 */
    debug_printfln("2.4G Module Set RX Mode");
    Inf_Si24R1_RXMode();
    // /* 2. 把2.4G模块设置为发送模式 */
    // debug_printfln("2.4G Module Set TX Mode");
    // Inf_Si24R1_TXMode();
    debug_printfln("Communication Module Start Success!");
}

/**
 * @description: 接收遥控发来的数据, 把收到的数据,并对收到的数据做校验
 * @return {*}
 */
Com_Status App_Communication_ReadRemoteData(void)
{
    if (!Inf_Si24R1_RxPacket(RX_BUFF))
        return Com_OTHER; /* 没有收到数据 */

    /* 1. 检测帧头 */
    uint8_t index = 0;
    if (RX_BUFF[index++] != FRAME_0 ||
        RX_BUFF[index++] != FRAME_1 ||
        RX_BUFF[index++] != FRAME_2)
        return Com_OTHER; /* 帧头错误 */

    //debug_printfln("Receive Data: %02X %02X %02X", RX_BUFF[0], RX_BUFF[1], RX_BUFF[2]);

    /* 2. 获取载荷长度并验证 */
    uint8_t payLoad = RX_BUFF[index++];
    if (payLoad != 10) { /* 根据你的发送逻辑，payload固定为10（THR/YAW/ROL/PIT + 2标志位） */
        //debug_printfln("Invalid payload: %d", payLoad);
        return Com_OTHER;
    }

    /* 3. 计算校验和（包含帧头、长度和所有payload 0 - 14位） */
    uint32_t calcCheckSum = 0;
    for (uint8_t i = 0; i < payLoad + 4; i++) { /* +4为校验和自身 */
        calcCheckSum += RX_BUFF[i];
    }

    /* 4. 读取接收到的校验和 14 - 17位 */
    uint32_t recvCheckSum = (RX_BUFF[index + payLoad] << 24) |
                            (RX_BUFF[index + payLoad + 1] << 16) |
                            (RX_BUFF[index + payLoad + 2] << 8) |
                            (RX_BUFF[index + payLoad + 3]);

    //debug_printfln("Calc Checksum: %d, Recv Checksum: %d", calcCheckSum, recvCheckSum);

    /* 5. 校验比对 */
    if (calcCheckSum != recvCheckSum) {
        //debug_printfln("Checksum mismatch!");
        return Com_OTHER;
    }

    return Com_OK;
}
Com_Status App_Communication_ConnectCheck(Com_Status isReadData)
{
    /* 通讯任务调度周期为4ms, 所以, 每计数一次表示4ms */
    static uint16_t noRecvDataCnt = 0; /* 用来记录收不到数据的次数 */
    if (isReadData == Com_OK) {
        noRecvDataCnt = 0; /* 一旦收到数据, 则清零i */
        /* 把收到的数据封装到摇杆结构体中 */
        joyStick.THR = (RX_BUFF[4] << 8 | RX_BUFF[5]);
        joyStick.YAW = (RX_BUFF[6] << 8 | RX_BUFF[7]);
        joyStick.ROL = (RX_BUFF[8] << 8 | RX_BUFF[9]);
        joyStick.PIT = (RX_BUFF[10] << 8 | RX_BUFF[11]);

        /* 收到定高命令, 就对当前的定高状态取反  */
        if (RX_BUFF[12]) {
            joyStick.isFixHeightPoint = !joyStick.isFixHeightPoint;
        }

        joyStick.isPowerDonw = RX_BUFF[13];

        return Com_OK; /* 如果收到数据,证明连接成功 */
    }

    noRecvDataCnt++;          /* 对收不到数据的次数 +1 */
    if (noRecvDataCnt >= 250) /* 如果连续250次(1s)收不到数据, 则表示失联 */
    {
        noRecvDataCnt = 250;
        return Com_ERROR;
    }

    return Com_OTHER;
}

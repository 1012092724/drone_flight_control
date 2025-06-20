#include "App_Communication.h"

JoyStick_Struct joyStick;
RC_Status rc_status = RC_UNCONNECTED; // 遥控器状态

/**
 * @description: 启动通讯模块
 * @return {*}
 */
void App_Communication_Start(void)
{
    /* 1. 等待自检通过 */
    debug_printfln("2.4G Module Checking...");
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
/**
 * @brief 验证接收数据的有效性（帧头、长度、校验和）
 * @return Data_Valid 数据有效，Data_Invalid 数据无效
 */
Data_Status App_Communication_ValidatePacket(void)
{
    /* 1. 检查是否收到数据包 */
    if (Inf_Si24R1_RxPacket(RX_BUFF)) {
        return Data_Invalid;
    }

    /* 2. 验证帧头 (3字节) */
    if (RX_BUFF[0] != FRAME_0 ||
        RX_BUFF[1] != FRAME_1 ||
        RX_BUFF[2] != FRAME_2) {
        return Data_Invalid;
    }

    /* 3. 验证载荷长度 (固定10字节) */
    const uint8_t EXPECTED_PAYLOAD = 10;
    uint8_t payloadLength          = RX_BUFF[3];
    if (payloadLength != EXPECTED_PAYLOAD) {
        return Data_Invalid;
    }

    /* 4. 计算并验证校验和 */
    uint32_t calculatedChecksum = 0;
    for (uint8_t i = 0; i < payloadLength + 4; i++) {
        calculatedChecksum += RX_BUFF[i];
    }

    uint32_t receivedChecksum = 0;
    receivedChecksum |= (uint32_t)RX_BUFF[14] << 24;
    receivedChecksum |= (uint32_t)RX_BUFF[15] << 16;
    receivedChecksum |= (uint32_t)RX_BUFF[16] << 8;
    receivedChecksum |= (uint32_t)RX_BUFF[17];

    return (calculatedChecksum == receivedChecksum) ? Data_Valid : Data_Invalid;
}

/**
 * @brief 更新遥控器连接状态并解析有效数据
 * @param isValid 数据是否有效
 */
void App_Communication_UpdateConnectionStatus(Data_Status isValid)
{
    static uint16_t lostSignalCounter = 0;   // 单位: 4ms
    const uint16_t TIMEOUT_THRESHOLD  = 250; // 1秒超时(250*4ms=1000ms)

    if (isValid == Data_Valid) {
        lostSignalCounter = 0;
        rc_status         = RC_CONNECTED;

        /* 解析遥杆数据 */
        joyStick.THR = (RX_BUFF[4] << 8) | RX_BUFF[5];
        joyStick.YAW = (RX_BUFF[6] << 8) | RX_BUFF[7];
        joyStick.ROL = (RX_BUFF[8] << 8) | RX_BUFF[9];
        joyStick.PIT = (RX_BUFF[10] << 8) | RX_BUFF[11];

        joyStick.isFixHeightPoint = RX_BUFF[12];
        joyStick.isPowerDonw      = RX_BUFF[13];
    } else {
        if (++lostSignalCounter >= TIMEOUT_THRESHOLD) {
            lostSignalCounter = TIMEOUT_THRESHOLD;
            rc_status         = RC_UNCONNECTED;
        }
    }
}
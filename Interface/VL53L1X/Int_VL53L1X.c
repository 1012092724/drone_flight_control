#include "Int_VL53L1X.h"

#define VL53L1X_IIC_ADDR 0x52
int status = 0;

void Int_VL53L1X_Init(void)
{

    uint8_t byteData, sensorState = 0;
    uint16_t wordData = 0;

    // 1.复位传感器
    HAL_GPIO_WritePin(X_SHUT_GPIO_Port, X_SHUT_Pin, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(X_SHUT_GPIO_Port, X_SHUT_Pin, GPIO_PIN_SET);
    HAL_Delay(2);

    /* Those basic I2C read functions can be used to check your own I2C functions */
    status = VL53L1_RdByte(VL53L1X_IIC_ADDR, 0x010F, &byteData);
    debug_printfln("VL53L1X Model_ID: %X", byteData);
    status = VL53L1_RdByte(VL53L1X_IIC_ADDR, 0x0110, &byteData);
    debug_printfln("VL53L1X Module_Type: %X", byteData);
    status = VL53L1_RdWord(VL53L1X_IIC_ADDR, 0x010F, &wordData);
    debug_printfln("VL53L1X: %X", wordData);
    while (sensorState == 0) {
        status = VL53L1X_BootState(VL53L1X_IIC_ADDR, &sensorState);
        HAL_Delay(2);
    }
    debug_printfln("Chip booted");

    /* This function must to be called to initialize the sensor with the default setting  */
    status = VL53L1X_SensorInit(VL53L1X_IIC_ADDR);
    /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
    status = VL53L1X_SetDistanceMode(VL53L1X_IIC_ADDR, 2);           /* 1=short, 2=long */
    status = VL53L1X_SetTimingBudgetInMs(VL53L1X_IIC_ADDR, 100);     /* in ms possible values [20, 50, 100, 200, 500] */
    status = VL53L1X_SetInterMeasurementInMs(VL53L1X_IIC_ADDR, 100); /* in ms, IM must be > = TB */
    //  status = VL53L1X_SetOffset(VL53L1X_IIC_ADDR,20); /* offset compensation in mm */
    //  status = VL53L1X_SetROI(VL53L1X_IIC_ADDR, 16, 16); /* minimum ROI 4,4 */
    //	status = VL53L1X_CalibrateOffset(VL53L1X_IIC_ADDR, 140, &offset); /* may take few second to perform the offset cal*/
    //	status = VL53L1X_CalibrateXtalk(VL53L1X_IIC_ADDR, 1000, &xtalk); /* may take few second to perform the xtalk cal */
    status = VL53L1X_StartRanging(VL53L1X_IIC_ADDR);
    debug_printfln("%d", status);
}

void Int_VL53L1X_Get_Distance(uint16_t *Distance)
{
    uint8_t dataReady = 0;
    status            = VL53L1X_CheckForDataReady(VL53L1X_IIC_ADDR, &dataReady);
    if (dataReady == 1) {
        status    = VL53L1X_GetDistance(VL53L1X_IIC_ADDR, Distance);
        dataReady = 0;
        status    = VL53L1X_ClearInterrupt(VL53L1X_IIC_ADDR); /* clear interrupt has to be called to enable next interrupt*/
    }
    // status    = VL53L1X_GetRangeStatus(VL53L1X_IIC_ADDR, &RangeStatus);
    // status = VL53L1X_GetSignalRate(VL53L1X_IIC_ADDR, &SignalRate);
    // status = VL53L1X_GetAmbientRate(VL53L1X_IIC_ADDR, &AmbientRate);
    // status = VL53L1X_GetSpadNb(VL53L1X_IIC_ADDR, &SpadNum);
}
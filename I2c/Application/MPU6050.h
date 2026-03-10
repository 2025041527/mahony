#ifndef __MPU6050_H
#define __MPU6050_H
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <math.h>

/* MPU6050 地址与寄存器定义 */
#define MPU6050_ADDR  (0x68 << 1)
#define PWR_MGMT_1    0x6B
#define ACCEL_CONFIG  0x1C
#define GYRO_CONFIG   0x1B
#define ACCEL_XOUT_H  0x3B

typedef struct {
    float q0, q1, q2, q3;  // 四元数
    float integralFBx, integralFBy, integralFBz;  // PI控制的积分项
    float Kp;               // 比例系数
    float Ki;               // 积分系数
    float Pitch, Roll, Yaw;
} MahonyFilter;

/* 函数声明 */
void MPU6050_Init(void);
void MPU6050_ReadRawData(int16_t AccX,int16_t AccY, int16_t AccZ, int16_t GyroX,int16_t GyroY,int16_t GyroZ);
void Mahony_Init(MahonyFilter *mf, float Kp, float Ki);
void Mahony_Update(MahonyFilter *mf, float ax, float ay, float az, 
                   float gx, float gy, float gz, float dt);

#ifdef __cplusplus
}
#endif
#endif /* __MPU6050_H */
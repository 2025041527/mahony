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
    
	  float AccX_Raw, AccY_Raw, AccZ_Raw;
    float GyroX_Raw, GyroY_Raw, GyroZ_Raw;
} MahonyFilter;

typedef struct {
float Pitch, Roll, Yaw;

}mn;
/* 函数声明 */
void MPU6050_Init(void);
void MPU6050_ReadRawData(MahonyFilter *m);
void MahonyAHRSupdateIMU(MahonyFilter *m,float *q);
float invSqrt(float x);
void calculate(void);
#ifdef __cplusplus
}
#endif
#endif /* __MPU6050_H */

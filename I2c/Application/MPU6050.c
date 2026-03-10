#include "MPU6050.h"
#include <math.h>
extern I2C_HandleTypeDef hi2c1;
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief  MPU6050初始化函数
 */
void MPU6050_Init(void) {
    uint8_t data = 0;
    // 唤醒MPU6050
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
    // 配置加速度计量程：±2g
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
    // 配置陀螺仪量程：±250°/s
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);
}

/**
 * @brief  读取MPU6050原始数据
 */
void MPU6050_ReadRawData(int16_t AccX, int16_t AccY, int16_t AccZ, 
                          int16_t GyroX, int16_t GyroY, int16_t GyroZ) {
    uint8_t buf[14];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, buf, 14, HAL_MAX_DELAY);
    AccX = (int16_t)((buf[0] << 8) | buf[1])/ 16384.0f; // 转换为g
    AccY = (int16_t)((buf[2] << 8) | buf[3])/ 16384.0f; 
    AccZ = (int16_t)((buf[4] << 8) | buf[5])/ 16384.0f; 
    GyroX = (int16_t)((buf[8] << 8) | buf[9])/ 131.0f; // 转换为°/s
    GyroY = (int16_t)((buf[10] << 8) | buf[11])/ 131.0f;
    GyroZ = (int16_t)((buf[12] << 8) | buf[13])/ 131.0f;
}

void Mahony_Init(MahonyFilter *mf, float Kp, float Ki) {
    // 四元数初始化为单位四元数
    mf->q0 = 1.0f;
    mf->q1 = 0.0f;
    mf->q2 = 0.0f;
    mf->q3 = 0.0f;
    // 积分项初始化为0
    mf->integralFBx = 0.0f;
    mf->integralFBy = 0.0f;
    mf->integralFBz = 0.0f;
    // 设置PI系数
    mf->Kp = Kp;
    mf->Ki = Ki;
    // 初始角度为0
    mf->Pitch = 0.0f;
    mf->Roll = 0.0f;
    mf->Yaw = 0.0f;
}

/**
 * @brief  Mahony滤波更新
 * @param  mf: Mahony滤波器结构体指针
 * @param  ax, ay, az: 加速度计物理量（g）
 * @param  gx, gy, gz: 陀螺仪物理量（°/s）
 * @param  dt: 时间间隔（秒）
 */
void Mahony_Update(MahonyFilter *mf, float ax, float ay, float az, 
                   float gx, float gy, float gz, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 加速度计数据归一化
    recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 辅助变量
    halfvx = mf->q1*mf->q3 - mf->q0*mf->q2;
    halfvy = mf->q0*mf->q1 + mf->q2*mf->q3;
    halfvz = mf->q0*mf->q0 - 0.5f + mf->q3*mf->q3;

    // 计算误差
    halfex = (ay*halfvz - az*halfvy);
    halfey = (az*halfvx - ax*halfvz);
    halfez = (ax*halfvy - ay*halfvx);

    // PI控制
    if (mf->Ki > 0.0f) {
        mf->integralFBx += mf->Ki * halfex * dt;
        mf->integralFBy += mf->Ki * halfey * dt;
        mf->integralFBz += mf->Ki * halfez * dt;
        gx += mf->integralFBx;
        gy += mf->integralFBy;
        gz += mf->integralFBz;
    } else {
        mf->integralFBx = 0.0f;
        mf->integralFBy = 0.0f;
        mf->integralFBz = 0.0f;
    }

    //比例控制
    gx += mf->Kp * halfex;
    gy += mf->Kp * halfey;
    gz += mf->Kp * halfez;

    //四元数更新
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = mf->q0;
    qb = mf->q1;
    qc = mf->q2;

    mf->q0 += (-qb*gx - qc*gy - mf->q3*gz);
    mf->q1 += (qa*gx + qc*gz - mf->q3*gy);
    mf->q2 += (qa*gy - qb*gz + mf->q3*gx);
    mf->q3 += (qa*gz + qb*gy - qc*gx);

    //四元数归一化
    recipNorm = 1.0f / sqrtf(mf->q0*mf->q0 + mf->q1*mf->q1 + mf->q2*mf->q2 + mf->q3*mf->q3);
    mf->q0 *= recipNorm;
    mf->q1 *= recipNorm;
    mf->q2 *= recipNorm;
    mf->q3 *= recipNorm;

    //四元数转换为欧拉角
    mf->Pitch = asinf(-2.0f * (mf->q1*mf->q3 - mf->q0*mf->q2)) * 180.0f / M_PI;
    mf->Roll = atan2f(2.0f * (mf->q0*mf->q1 + mf->q2*mf->q3), 
                      mf->q0*mf->q0 - mf->q1*mf->q1 - mf->q2*mf->q2 + mf->q3*mf->q3) * 180.0f / M_PI;
    mf->Yaw = atan2f(2.0f * (mf->q1*mf->q2 + mf->q0*mf->q3), 
                     mf->q0*mf->q0 + mf->q1*mf->q1 - mf->q2*mf->q2 - mf->q3*mf->q3) * 180.0f / M_PI;

}
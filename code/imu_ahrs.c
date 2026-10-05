#include "imu_ahrs.h"

#include <math.h>
#include "Ifx_LutAtan2F32.h"

#ifndef IMU_AHRS_PI
#define IMU_AHRS_PI (3.14159265358979323846f)
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     快速倒平方根（1/sqrt）
// 参数说明     x               输入值
// 返回参数     float           1/sqrt(x)
// 使用示例     imu_ahrs_inv_sqrtf(n2);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------

static float imu_ahrs_inv_sqrtf(float x)
{
    if (x <= 0.0f)
    {
        return 0.0f;
    }
    
    float half_x = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (half_x * y * y));
    // y = y * (1.5f - (half_x * y * y)); // 可选：第二次牛顿迭代提高精度
    
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     四元数归一化
// 参数说明     ahrs            AHRS 上下文
// 返回参数     void
// 使用示例     imu_ahrs_normalize_quat(&ahrs);
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void imu_ahrs_normalize_quat(imu_ahrs_t *ahrs)
{
    float n2 = ahrs->q0 * ahrs->q0 + ahrs->q1 * ahrs->q1 + ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3;
    float inv = imu_ahrs_inv_sqrtf(n2);
    if (inv > 0.0f)
    {
        ahrs->q0 *= inv;
        ahrs->q1 *= inv;
        ahrs->q2 *= inv;
        ahrs->q3 *= inv;
    }
    else
    {
        ahrs->q0 = 1.0f;
        ahrs->q1 = 0.0f;
        ahrs->q2 = 0.0f;
        ahrs->q3 = 0.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AHRS 初始化
// 参数说明     ahrs            AHRS 上下文
// 参数说明     kp              比例增益
// 参数说明     ki              积分增益
// 返回参数     void
// 使用示例     imu_ahrs_init(&ahrs, 2.0f, 0.05f);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void imu_ahrs_init(imu_ahrs_t *ahrs, float kp, float ki)
{
    if (!ahrs)
    {
        return;
    }

    ahrs->kp = kp;
    ahrs->ki = ki;

    imu_ahrs_reset(ahrs);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     AHRS 复位
// 参数说明     ahrs            AHRS 上下文
// 返回参数     void
// 使用示例     imu_ahrs_reset(&ahrs);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void imu_ahrs_reset(imu_ahrs_t *ahrs)
{
    if (!ahrs)
    {
        return;
    }

    ahrs->q0 = 1.0f;
    ahrs->q1 = 0.0f;
    ahrs->q2 = 0.0f;
    ahrs->q3 = 0.0f;

    ahrs->ix = 0.0f;
    ahrs->iy = 0.0f;
    ahrs->iz = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     Mahony IMU-only 姿态更新
// 参数说明     ahrs            AHRS 上下文
// 参数说明     gx_dps          X 轴角速度（dps）
// 参数说明     gy_dps          Y 轴角速度（dps）
// 参数说明     gz_dps          Z 轴角速度（dps）
// 参数说明     ax_g            X 轴加速度（g）
// 参数说明     ay_g            Y 轴加速度（g）
// 参数说明     az_g            Z 轴加速度（g）
// 参数说明     dt_s            采样周期（s）
// 返回参数     void
// 使用示例     imu_ahrs_update_imu(&ahrs, gx, gy, gz, ax, ay, az, dt);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void imu_ahrs_update_imu(imu_ahrs_t *ahrs,
                         float gx_dps,
                         float gy_dps,
                         float gz_dps,
                         float ax_g,
                         float ay_g,
                         float az_g,
                         float dt_s)
{
    if ((!ahrs) || (dt_s <= 0.0f))
    {
        return;
    }

    const float deg2rad = IMU_AHRS_PI / 180.0f;
    float gx = gx_dps * deg2rad;
    float gy = gy_dps * deg2rad;
    float gz = gz_dps * deg2rad;

    float a2 = ax_g * ax_g + ay_g * ay_g + az_g * az_g;
    if (a2 > 1e-12f)
    {
        float inv_a = imu_ahrs_inv_sqrtf(a2);
        float ax = ax_g * inv_a;
        float ay = ay_g * inv_a;
        float az = az_g * inv_a;

        float q0 = ahrs->q0;
        float q1 = ahrs->q1;
        float q2 = ahrs->q2;
        float q3 = ahrs->q3;

        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        if (ahrs->ki > 0.0f)
        {
            ahrs->ix += ahrs->ki * ex * dt_s;
            ahrs->iy += ahrs->ki * ey * dt_s;
            ahrs->iz += ahrs->ki * ez * dt_s;

            gx += ahrs->ix;
            gy += ahrs->iy;
            gz += ahrs->iz;
        }
        else
        {
            ahrs->ix = 0.0f;
            ahrs->iy = 0.0f;
            ahrs->iz = 0.0f;
        }

        gx += ahrs->kp * ex;
        gy += ahrs->kp * ey;
        gz += ahrs->kp * ez;
    }

    float q0 = ahrs->q0;
    float q1 = ahrs->q1;
    float q2 = ahrs->q2;
    float q3 = ahrs->q3;

    float half_dt = 0.5f * dt_s;

    float dq0 = (-q1 * gx - q2 * gy - q3 * gz) * half_dt;
    float dq1 = ( q0 * gx + q2 * gz - q3 * gy) * half_dt;
    float dq2 = ( q0 * gy - q1 * gz + q3 * gx) * half_dt;
    float dq3 = ( q0 * gz + q1 * gy - q2 * gx) * half_dt;

    ahrs->q0 = q0 + dq0;
    ahrs->q1 = q1 + dq1;
    ahrs->q2 = q2 + dq2;
    ahrs->q3 = q3 + dq3;

    imu_ahrs_normalize_quat(ahrs);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取欧拉角（rad）
// 参数说明     ahrs            AHRS 上下文
// 参数说明     roll_rad        横滚角输出指针（rad）
// 参数说明     pitch_rad       俯仰角输出指针（rad）
// 参数说明     yaw_rad         航向角输出指针（rad）
// 返回参数     void
// 使用示例     imu_ahrs_get_euler_rad(&ahrs, &roll, &pitch, &yaw);
// 备注信息     外部调用
//-------------------------------------------------------------------------------------------------------------------
void imu_ahrs_get_euler_rad(const imu_ahrs_t *ahrs, float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    if (!ahrs)
    {
        return;
    }

    float q0 = ahrs->q0;
    float q1 = ahrs->q1;
    float q2 = ahrs->q2;
    float q3 = ahrs->q3;

    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    float roll = Ifx_LutAtan2F32_float32(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    float pitch;
    if (fabsf(sinp) >= 1.0f)
    {
        pitch = copysignf(IMU_AHRS_PI / 2.0f, sinp);
    }
    else
    {
        pitch = asinf(sinp);
    }

    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float yaw = Ifx_LutAtan2F32_float32(siny_cosp, cosy_cosp);

    if (roll_rad)  *roll_rad = roll;
    if (pitch_rad) *pitch_rad = pitch;
    if (yaw_rad)   *yaw_rad = yaw;
}

uint8_t imu_ahrs_check_stale(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z)
{
    static int16_t last_acc_x = 0, last_acc_y = 0, last_acc_z = 0;
    static int16_t last_gyro_x = 0, last_gyro_y = 0, last_gyro_z = 0;
    static uint16_t imu_stale_count = 0;

    if (acc_x == last_acc_x && acc_y == last_acc_y && acc_z == last_acc_z &&
        gyro_x == last_gyro_x && gyro_y == last_gyro_y && gyro_z == last_gyro_z)
    {
        imu_stale_count++;
        if (imu_stale_count > 20) // 连续约 100ms(假设5ms调用一次) 无变化
        {
            imu_stale_count = 20; // 防溢出
            return 1;
        }
    }
    else
    {
        imu_stale_count = 0;
        last_acc_x = acc_x;
        last_acc_y = acc_y;
        last_acc_z = acc_z;
        last_gyro_x = gyro_x;
        last_gyro_y = gyro_y;
        last_gyro_z = gyro_z;
    }
    return 0;
}

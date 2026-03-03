#pragma once
#include <math.h>

class WheelOdometry {
public:
    float x = 0, y = 0, theta = 0;
    float linear_vel = 0, angular_vel = 0;
    
    WheelOdometry(float wheel_radius, float wheel_base)
        : _radius(wheel_radius), _base(wheel_base) {}
    
    void update(float vel_left, float vel_right, float dt) {
        linear_vel = (vel_right + vel_left) / 2.0f;
        angular_vel = (vel_right - vel_left) / _base;
        
        // Runge-Kutta 2nd order
        float half_theta = theta + angular_vel * dt / 2.0f;
        x += linear_vel * cosf(half_theta) * dt;
        y += linear_vel * sinf(half_theta) * dt;
        theta += angular_vel * dt;
        
        // Нормализация угла
        while (theta > M_PI) theta -= 2.0f * M_PI;
        while (theta < -M_PI) theta += 2.0f * M_PI;
    }
    
    // Слияние yaw из IMU (комплементарный фильтр)
    void fuseYaw(float imu_yaw, float alpha) {
        // alpha = 0.98 -> 98% IMU, 2% wheel
        float diff = imu_yaw - theta;
        // Нормализация разницы
        while (diff > M_PI) diff -= 2.0f * M_PI;
        while (diff < -M_PI) diff += 2.0f * M_PI;
        
        theta += (1.0f - alpha) * 0 + alpha * diff;
        // Упрощённо: theta = alpha * imu_yaw + (1-alpha) * theta
    }

private:
    float _radius;
    float _base;
};
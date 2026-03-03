#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <tf2_msgs/msg/tf_message.h>

#include "imu/icm42688.h"
#include "imu/madgwick.h"
#include "motors/ddsm400.h"
#include "motors/pid.h"
#include "odometry/wheel_odom.h"

// === Объекты ===
ICM42688 imu_sensor(SPI1, PA4);  // SPI1, CS=PA4 (зависит от Matek pinout)
MadgwickFilter madgwick;
DDSM400 motors(Serial2);         // UART2 для DDSM400
PIDController pid_left(2.0, 0.5, 0.01);
PIDController pid_right(2.0, 0.5, 0.01);
WheelOdometry wheel_odom(WHEEL_RADIUS, WHEEL_BASE);

// === micro-ROS ===
rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;
rcl_publisher_t tf_pub;
rcl_subscription_t cmd_vel_sub;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// === Состояние ===
volatile float target_linear = 0;
volatile float target_angular = 0;
uint32_t last_cmd_time = 0;

// === Таймеры ===
uint32_t imu_timer = 0;
uint32_t odom_timer = 0;
uint32_t pub_timer = 0;
uint32_t pid_timer = 0;

void cmd_vel_callback(const void *msg) {
    const geometry_msgs__msg__Twist *twist = 
        (const geometry_msgs__msg__Twist *)msg;
    target_linear = twist->linear.x;
    target_angular = twist->angular.z;
    last_cmd_time = millis();
}

void setup() {
    // === Hardware init ===
    imu_sensor.begin();
    imu_sensor.setAccelRange(ICM42688::ACCEL_8G);
    imu_sensor.setGyroRange(ICM42688::GYRO_1000DPS);
    imu_sensor.setSampleRate(1000);
    
    motors.begin(115200);  // или CAN init
    
    // === micro-ROS ===
    set_microros_serial_transports(Serial1);  // UART к хосту
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);
    
    rcl_node_t node;
    rclc_node_init_default(&node, "base_controller", "", &support);
    
    // Publishers
    rclc_publisher_init_default(&odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
    
    rclc_publisher_init_default(&imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");
    
    // Subscriber
    rclc_subscription_init_default(&cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    
    // Executor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, 
        &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    
    // Инициализация сообщений
    init_odom_message(&odom_msg);
    init_imu_message(&imu_msg);
}

void loop() {
    uint32_t now = micros();
    
    // ==========================================
    // IMU @ 1kHz
    // ==========================================
    if (now - imu_timer >= 1000) {  // 1ms
        imu_timer = now;
        
        float ax, ay, az, gx, gy, gz;
        imu_sensor.readAll(&ax, &ay, &az, &gx, &gy, &gz);
        
        // Обновляем фильтр ориентации
        madgwick.update(gx, gy, gz, ax, ay, az, 0.001f);
    }
    
    // ==========================================
    // Одометрия + PID @ 100Hz
    // ==========================================
    if (now - odom_timer >= 10000) {  // 10ms
        odom_timer = now;
        float dt = 0.01f;
        
        // Читаем реальные скорости с DDSM400
        float rpm_left = motors.getRPM(MOTOR_ID_LEFT);
        float rpm_right = motors.getRPM(MOTOR_ID_RIGHT);
        
        float vel_left = rpm_to_ms(rpm_left);
        float vel_right = rpm_to_ms(rpm_right);
        
        // Обновляем одометрию
        wheel_odom.update(vel_left, vel_right, dt);
        
        // Используем yaw из IMU вместо интегрирования гироскопа колёс
        float imu_yaw = madgwick.getYaw();
        wheel_odom.fuseYaw(imu_yaw, 0.98f);  // комплементарный фильтр
        
        // === PID контур ===
        // Целевые скорости колёс из cmd_vel
        float target_left = target_linear - target_angular * WHEEL_BASE / 2.0f;
        float target_right = target_linear + target_angular * WHEEL_BASE / 2.0f;
        
        // Watchdog — если нет команд 500мс, стоп
        if (millis() - last_cmd_time > 500) {
            target_left = 0;
            target_right = 0;
        }
        
        float cmd_left = pid_left.compute(target_left, vel_left, dt);
        float cmd_right = pid_right.compute(target_right, vel_right, dt);
        
        motors.setSpeed(MOTOR_ID_LEFT, ms_to_rpm(cmd_left));
        motors.setSpeed(MOTOR_ID_RIGHT, ms_to_rpm(cmd_right));
    }
    
    // ==========================================
    // PID регулятор @ 100Hz  
    // ==========================================
    if (now - pid_timer >= 10000) {
        pid_timer = now;
        // (объединено с одометрией выше)
    }
    
    // ==========================================
    // Публикация в ROS @ 50Hz
    // ==========================================
    if (now - pub_timer >= 20000) {  // 20ms
        pub_timer = now;
        
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        
        // --- Odometry ---
        odom_msg.header.stamp.sec = ts.tv_sec;
        odom_msg.header.stamp.nanosec = ts.tv_nsec;
        odom_msg.pose.pose.position.x = wheel_odom.x;
        odom_msg.pose.pose.position.y = wheel_odom.y;
        odom_msg.pose.pose.position.z = 0;
        
        // Quaternion из yaw
        float yaw = wheel_odom.theta;
        odom_msg.pose.pose.orientation.x = 0;
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = sinf(yaw / 2.0f);
        odom_msg.pose.pose.orientation.w = cosf(yaw / 2.0f);
        
        odom_msg.twist.twist.linear.x = wheel_odom.linear_vel;
        odom_msg.twist.twist.angular.z = wheel_odom.angular_vel;
        
        // Covariance
        fill_odom_covariance(&odom_msg);
        
        rcl_publish(&odom_pub, &odom_msg, NULL);
        
        // --- IMU ---
        float q[4];
        madgwick.getQuaternion(q);
        
        imu_msg.header.stamp = odom_msg.header.stamp;
        imu_msg.orientation.w = q[0];
        imu_msg.orientation.x = q[1];
        imu_msg.orientation.y = q[2];
        imu_msg.orientation.z = q[3];
        
        float ax, ay, az, gx, gy, gz;
        imu_sensor.getLastReading(&ax, &ay, &az, &gx, &gy, &gz);
        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;
        
        rcl_publish(&imu_pub, &imu_msg, NULL);
    }
    
    // Обработка micro-ROS
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

// --- Pin Definitions ---
#define I2C0_SDA_IO     21
#define I2C0_SCL_IO     22
#define I2C1_SDA_IO     17
#define I2C1_SCL_IO     16
#define I2C_FREQ_HZ     100000

// --- MPU6050 Registers ---
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B  // Start of data block

static const char *TAG = "MPU_TRIO";

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t temp; // Temperature is between Accel and Gyro in memory
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu_data_t;

// Initialize both I2C Ports
void init_i2c_buses() {
    // --- Config Bus 0 ---
    i2c_config_t conf0 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C0_SDA_IO,
        .scl_io_num = I2C0_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_NUM_0, &conf0);
    i2c_driver_install(I2C_NUM_0, conf0.mode, 0, 0, 0);

    // --- Config Bus 1 ---
    i2c_config_t conf1 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C1_SDA_IO,
        .scl_io_num = I2C1_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_NUM_1, &conf1);
    i2c_driver_install(I2C_NUM_1, conf1.mode, 0, 0, 0);
}

// Wake up a specific sensor
void mpu6050_wake(i2c_port_t port, uint8_t addr) {
    uint8_t data[2] = {MPU6050_PWR_MGMT_1, 0x00};
    i2c_master_write_to_device(port, addr, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

// Read Accel and Gyro data
void mpu6050_read_all(i2c_port_t port, uint8_t addr, mpu_data_t *result) {
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    uint8_t buffer[14]; // 6 Accel + 2 Temp + 6 Gyro

    // Burst read 14 bytes
    i2c_master_write_read_device(port, addr, &reg, 1, buffer, 14, 1000 / portTICK_PERIOD_MS);

    // Parse Data (High byte << 8 | Low byte)
    result->acc_x  = (buffer[0] << 8) | buffer[1];
    result->acc_y  = (buffer[2] << 8) | buffer[3];
    result->acc_z  = (buffer[4] << 8) | buffer[5];
    // buffer[6] and [7] are temperature
    result->gyro_x = (buffer[8] << 8) | buffer[9];
    result->gyro_y = (buffer[10] << 8) | buffer[11];
    result->gyro_z = (buffer[12] << 8) | buffer[13];
}

void app_main(void) {
    init_i2c_buses();

    // Wake up all 3 sensors
    mpu6050_wake(I2C_NUM_0, 0x68); // Bus 0, Sensor 1
    mpu6050_wake(I2C_NUM_0, 0x69); // Bus 0, Sensor 2
    mpu6050_wake(I2C_NUM_1, 0x68); // Bus 1, Sensor 3
    
    ESP_LOGI(TAG, "All 3 Sensors Initialized");

    mpu_data_t s1, s2, s3;

    while (1) {
        // Read Bus 0 Sensors
        mpu6050_read_all(I2C_NUM_0, 0x68, &s1);
        mpu6050_read_all(I2C_NUM_0, 0x69, &s2);
        
        // Read Bus 1 Sensor
        mpu6050_read_all(I2C_NUM_1, 0x68, &s3);

        // Print Summary (Shortened for readability)
        printf("--- DATA ---\n");
        printf("S1 (Bus0 0x68): A[x:%d y:%d z:%d] G[x:%d y:%d z:%d]\n", 
               s1.acc_x, s1.acc_y, s1.acc_z, s1.gyro_x, s1.gyro_y, s1.gyro_z);
               
        printf("S2 (Bus0 0x69): A[x:%d y:%d z:%d] G[x:%d y:%d z:%d]\n", 
               s2.acc_x, s2.acc_y, s2.acc_z, s2.gyro_x, s2.gyro_y, s2.gyro_z);
               
        printf("S3 (Bus1 0x68): A[x:%d y:%d z:%d] G[x:%d y:%d z:%d]\n", 
               s3.acc_x, s3.acc_y, s3.acc_z, s3.gyro_x, s3.gyro_y, s3.gyro_z);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
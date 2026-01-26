#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// --- Pin Definitions ---
#define I2C0_SDA_IO 21
#define I2C0_SCL_IO 22
#define I2C1_SDA_IO 17
#define I2C1_SCL_IO 16
#define I2C_FREQ_HZ 400000 // 400kHz for Fast Mode

static const char *TAG = "MPU_TRIO";

// Sensor Objects
MPU6050 mpu1(0x68); // Will use Bus 0
MPU6050 mpu2(0x69); // Will use Bus 0
MPU6050 mpu3(0x68); // Will use Bus 1 

// Data containers
uint8_t fifoBuffer[64]; // FIFO buffer for Accelerometer and Gyro . Static Array of 64 unsigned bytes
Quaternion q; // [w, x, y, z]         quaternion container 
VectorFloat gravity; // [x, y, z]        gravity vector
VectorInt16 aa, aaReal, gyro; //aa = [x, y, z]            accel sensor measurements
                             //aaReal = [x, y, z]        gravity-free accel sensor measurements
                             //gyro = [x, y, z]          gyro sensor measurements

// Standard I2C Init (Same as before)
void init_i2c_buses() {
    i2c_config_t conf0 = {};
    conf0.mode = I2C_MODE_MASTER;
    conf0.sda_io_num = (gpio_num_t)I2C0_SDA_IO;
    conf0.scl_io_num = (gpio_num_t)I2C0_SCL_IO;
    conf0.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf0.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf0.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(I2C_NUM_0, &conf0);
    i2c_driver_install(I2C_NUM_0, conf0.mode, 0, 0, 0);

    i2c_config_t conf1 = {};
    conf1.mode = I2C_MODE_MASTER;
    conf1.sda_io_num = (gpio_num_t)I2C1_SDA_IO;
    conf1.scl_io_num = (gpio_num_t)I2C1_SCL_IO;
    conf1.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf1.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf1.master.clk_speed = I2C_FREQ_HZ;
    i2c_param_config(I2C_NUM_1, &conf1);
    i2c_driver_install(I2C_NUM_1, conf1.mode, 0, 0, 0);
}

// Helper to init DMP
void init_sensor_dmp(MPU6050 &mpu, const char* name) {
    mpu.initialize();
    if (mpu.testConnection() && mpu.dmpInitialize() == 0) {
        mpu.setDMPEnabled(true);
        ESP_LOGI(TAG, "%s DMP Ready!", name);
    } else {
        ESP_LOGE(TAG, "%s DMP Failed", name);
    }
}

// Helper to read and process DMP data (Compatible with older libraries)
void process_dmp_data(MPU6050 &mpu, const char* name) {
    uint16_t fifoCount = mpu.getFIFOCount();

    // Check for FIFO overflow (Buffer size is 1024 bytes)
    if (fifoCount == 1024) {
        mpu.resetFIFO();
        ESP_LOGW(TAG, "%s FIFO Overflow! Resetting...", name);
        return;
    }

    // A standard DMP packet is 42 bytes. Wait until we have at least one full packet.
    if (fifoCount >= 42) {
        // Read exactly 42 bytes from the FIFO buffer
        mpu.getFIFOBytes(fifoBuffer, 42);

        // Process the data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetGyro(&gyro, fifoBuffer);

        ESP_LOGI(TAG, "%s - LinAcc [X:%6d Y:%6d Z:%6d] | Gyro [X:%6d Y:%6d Z:%6d]", 
                 name, aaReal.x, aaReal.y, aaReal.z, gyro.x, gyro.y, gyro.z);
    }
}

extern "C" void app_main(void) {
    init_i2c_buses();
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "--- INITIALIZING SENSORS ---");

    // 1. Switch to Bus 0 -> Initialize S1 and S2
    I2Cdev::setI2CPort(I2C_NUM_0);
    init_sensor_dmp(mpu1, "S1");
    init_sensor_dmp(mpu2, "S2");

    // 2. Switch to Bus 1 -> Initialize S3
    I2Cdev::setI2CPort(I2C_NUM_1);
    init_sensor_dmp(mpu3, "S3");

    ESP_LOGI(TAG, "--- STARTING MAIN LOOP ---");

    while (1) {
        // --- READ BUS 0 ---
        I2Cdev::setI2CPort(I2C_NUM_0);
        process_dmp_data(mpu1, "S1");
        process_dmp_data(mpu2, "S2");

        // --- READ BUS 1 ---
        I2Cdev::setI2CPort(I2C_NUM_1);
        process_dmp_data(mpu3, "S3");

        // Poll at 100Hz (10ms) - Adjust based on your needs
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
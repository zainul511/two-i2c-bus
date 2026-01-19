#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

// Library headers (adjust paths based on your component structure)
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define GPIO_SDA 21
#define GPIO_SCL 22
#define I2C_FREQ_HZ 400000

static const char *TAG = "MPU_DMP";
MPU6050 mpu;

// Buffer for DMP packets
uint8_t fifoBuffer[64];
// Orientation/Motion variables
Quaternion q;           // [w, x, y, z]
VectorFloat gravity;    // [x, y, z]
VectorInt16 aa;         // [x, y, z]            <-- Raw acceleration sensor measurements
VectorInt16 aaReal;     // [x, y, z]            <-- Gravity-free acceleration measurements
VectorInt16 gyro;       // [x, y, z]            <-- Raw gyroscope sensor measurements
float ypr[3];           // [yaw, pitch, roll]

void i2c_master_init() {
    i2c_config_t conf = {};
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)GPIO_SDA;
        conf.scl_io_num = (gpio_num_t)GPIO_SCL;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE,
        conf.master.clk_speed = I2C_FREQ_HZ,
        conf.clk_flags = 0; // Optional but good practice
    
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}
extern "C" {
    void app_main(void);
}
void app_main(void) {
    // 1. Initialize I2C and MPU
    i2c_master_init();
    mpu.initialize();

    if (!mpu.testConnection()) {
        ESP_LOGE(TAG, "MPU6050 connection failed");
        return;
    }

    // 2. Initialize DMP
    ESP_LOGI(TAG, "Initializing DMP...");
    uint8_t devStatus = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        // 3. Enable DMP
        mpu.setDMPEnabled(true);
        ESP_LOGI(TAG, "DMP enabled!");
    } else {
        ESP_LOGE(TAG, "DMP Init failed (code %d)", devStatus);
        return;
    }

    uint16_t packetSize = mpu.dmpGetFIFOPacketSize();

    // 4. Main Loop
    while (1) {
        // Read packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // 1. Calculate Gravity (Necessary step to remove it from Accel)
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);

        // 2. Get Raw Accel and Remove Gravity
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        // 3. Get Gyroscope
        mpu.dmpGetGyro(&gyro, fifoBuffer);

        // 4. Print Data (aaReal = Linear Accel, gyro = Gyroscope)
        ESP_LOGI(TAG, "LinAccel X:%d Y:%d Z:%d | Gyro X:%d Y:%d Z:%d", 
                aaReal.x, aaReal.y, aaReal.z, 
                gyro.x, gyro.y, gyro.z);


    }
}
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "I2Cdev.h"
#include "MPU6050.h" // Note: We use the base class, not MotionApps for calibration

// --- Pin Definitions (Keep consistent with your main code) ---
#define I2C0_SDA_IO 21
#define I2C0_SCL_IO 22
#define I2C1_SDA_IO 17
#define I2C1_SCL_IO 16
#define I2C_FREQ_HZ 400000 

static const char *TAG = "CALIBRATION";

// Sensor Objects
MPU6050 mpu1(0x68); 
MPU6050 mpu2(0x69); 
MPU6050 mpu3(0x68); 

// Helper to init I2C (Same as before)
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

// --- The Calibration Function ---
void calibrate_single_mpu(MPU6050 &mpu, const char* name, i2c_port_t i2c_num) {
    I2Cdev::setI2CPort(i2c_num);
    
    mpu.initialize();
    if(!mpu.testConnection()){
        ESP_LOGE(TAG, "%s Connection Failed", name);
        return;
    }

    ESP_LOGI(TAG, "Starting Calibration for %s. DO NOT MOVE SENSORS...", name);
    
    // We use the library's built-in PID calibration loop
    // 6 loops of fine-tuning usually gets good results
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    
    // Retrieve the calculated offsets
    int16_t ax_off = mpu.getXAccelOffset();
    int16_t ay_off = mpu.getYAccelOffset();
    int16_t az_off = mpu.getZAccelOffset();
    int16_t gx_off = mpu.getXGyroOffset();
    int16_t gy_off = mpu.getYGyroOffset();
    int16_t gz_off = mpu.getZGyroOffset();

    printf("\n>>> COPY THIS FOR %s <<<\n", name);
    printf("MPUOffsets offsets_%s = {%d, %d, %d, %d, %d, %d};\n", 
            name, ax_off, ay_off, az_off, gx_off, gy_off, gz_off);
    printf("--------------------------------------\n\n");
}

extern "C" void app_main(void) {
    init_i2c_buses();
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for power to stabilize

    ESP_LOGI(TAG, "--- CALIBRATION START ---");

    // Calibrate S1 (Bus 0, 0x68)
    calibrate_single_mpu(mpu1, "S1", I2C_NUM_0);
    
    // Calibrate S2 (Bus 0, 0x69)
    calibrate_single_mpu(mpu2, "S2", I2C_NUM_0);

    // Calibrate S3 (Bus 1, 0x68)
    calibrate_single_mpu(mpu3, "S3", I2C_NUM_1);

    ESP_LOGI(TAG, "--- CALIBRATION DONE ---");
}
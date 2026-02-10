#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// --- Pin Definitions ---
#define I2C0_SDA_IO 21
#define I2C0_SCL_IO 22
#define I2C1_SDA_IO 17
#define I2C1_SCL_IO 16
#define SYNC_PIN 4
#define I2C_FREQ_HZ 400000 // 400kHz for Fast Mode



static const char *TAG = "MPU_TRIO"; // Tag for logging

//Struct to hold sensor data
struct SensorData {
    int16_t ax, ay, az; // Accelerometer data
    int16_t gx, gy, gz; // Gyroscope data
};

struct MPUOffsets { // Structure to hold offset values for each sensor
    int ax,ay,az; // Accelerometer offsets
    int gx,gy,gz; // Gyroscope offsets
};

// Offsets for each sensor (True Values)
MPUOffsets offsets_S1 = {-3478, -387, 896, 139, 8, -32};
MPUOffsets offsets_S2 = {290, -4953, 1866, 104, 49, -40};
MPUOffsets offsets_S3 = {-3534, -977, 1266, 50, -8, -38};

// Sensor Objects
MPU6050 mpu1(0x68); // Will use Bus 0
MPU6050 mpu2(0x69); // Will use Bus 0
MPU6050 mpu3(0x68); // Will use Bus 1 

// Global Buffers and Variables
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
void init_sensor_dmp(MPU6050 &mpu, const char* name, MPUOffsets offsets) { 
    mpu.initialize();

    ESP_LOGI(TAG, "Applying offsets for %s...", name);
    mpu.setXAccelOffset(offsets.ax);
    mpu.setYAccelOffset(offsets.ay);
    mpu.setZAccelOffset(offsets.az);
    mpu.setXGyroOffset(offsets.gx);
    mpu.setYGyroOffset(offsets.gy);
    mpu.setZGyroOffset(offsets.gz);

    if (mpu.testConnection() && mpu.dmpInitialize() == 0) { //Check mpu6050 connection and DMP is active.Send a small code chunk to DMP and it will reply with a status of zero if it is working properly
        mpu.setDMPEnabled(true); //Enable the DMP
        ESP_LOGI(TAG, "%s DMP Ready!", name); //Success Message
    } else {
        ESP_LOGE(TAG, "%s DMP Failed", name);
    }
}

// Helper to read and process DMP data (Compatible with older libraries)
// Check the sensor's FIFO buffer is in valid state and read data.
bool read_dmp_data(MPU6050 &mpu, SensorData &data, const char* name) {
    uint16_t fifoCount = mpu.getFIFOCount();

    // Check for FIFO overflow (Buffer size is 1024 bytes)
    if (fifoCount == 1024) {
        mpu.resetFIFO();
        return false; // Indicate overflow occurred
    }

    // A standard DMP packet is 42 bytes. Wait until we have at least one full packet.
    // Why DMP packet is 42 bytes: Quaternion (16) + Gyro (6) + Accel (6) + others (14) = 42 bytes.
    //What are the "others"? Temperature, Timestamp, etc.
    if (fifoCount >= 42) {
        // Read exactly 42 bytes from the FIFO buffer
        mpu.getFIFOBytes(fifoBuffer, 42);

        // Process the data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetGyro(&gyro, fifoBuffer);
        
        // Store into the passed structure 
        data.ax = aaReal.x;
        data.ay = aaReal.y;
        data.az = aaReal.z;
        data.gx = gyro.x;
        data.gy = gyro.y;
        data.gz = gyro.z;
        return true; // Indicate successful read
    }
    return false; // Not enough data yet
}

extern "C" void app_main(void) {
    init_i2c_buses();

    // Initialize SYNC_PIN as input to read sync signal
    gpio_reset_pin((gpio_num_t)SYNC_PIN); // Reset the pin to default state
    gpio_set_direction((gpio_num_t)SYNC_PIN, GPIO_MODE_INPUT); // Set as input to read the sync signal from external source (Aaftabs 10Hz Pulse Generator)

    vTaskDelay(pdMS_TO_TICKS(100)); //PpdMS means "port delay in milliseconds". Why this delay? To give some time for the I2C buses to stabilize before we start communicating with the sensors. 
                                    //It's a common practice to add a small delay after initializing hardware components.


    ESP_LOGI(TAG, "--- INITIALIZING SENSORS ---");

    // 1. Switch to Bus 0 -> Initialize S1 and S2
    I2Cdev::setI2CPort(I2C_NUM_0);
    init_sensor_dmp(mpu1, "S1", offsets_S1);
    init_sensor_dmp(mpu2, "S2", offsets_S2);

    // 2. Switch to Bus 1 -> Initialize S3
    I2Cdev::setI2CPort(I2C_NUM_1);
    init_sensor_dmp(mpu3, "S3", offsets_S3);

    ESP_LOGI(TAG, "--- STARTING DATA STREAMING ---");
    // Print CSV Header
    printf("Time_ms,S1_Ax,S1_Ay,S1_Az,S1_Gx,S1_Gy,S1_Gz,S2_Ax,S2_Ay,S2_Az,S2_Gx,S2_Gy,S2_Gz,S3_Ax,S3_Ay,S3_Az,S3_Gx,S3_Gy,S3_Gz,Sync\n");

    SensorData d1, d2, d3; // Structures to hold data for each sensor
    bool got_s1, got_s2, got_s3; // Flags to indicate if we successfully read data from each sensor

    while (1) {

        int syncState = gpio_get_level((gpio_num_t)SYNC_PIN); // Read the current state of the sync pin (0 or 1)
                                                                // Once per loop, we read the Sync Pin

        // --- READ BUS 0 ---
        I2Cdev::setI2CPort(I2C_NUM_0);
        got_s1 = read_dmp_data(mpu1, d1, "S1"); // Read data from Sensor 1. This will check if there's a full DMP packet in the FIFO, read it, process it, and store the results in d1. It returns true if successful.
        got_s2 = read_dmp_data(mpu2, d2, "S2");



        // --- READ BUS 1 ---
        I2Cdev::setI2CPort(I2C_NUM_1);
        got_s3 = read_dmp_data(mpu3, d3, "S3");

        if (got_s1 && got_s2 && got_s3) { // Only print if we got valid data from all sensors
            uint32_t time_ms = esp_timer_get_time() / 1000; // Time in Milliseconds

            // Print formatted string: Time, S1(6 vals), S2(6 vals), S3(6 vals), Sync
            printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                   time_ms,
                   d1.ax, d1.ay, d1.az, d1.gx, d1.gy, d1.gz,
                   d2.ax, d2.ay, d2.az, d2.gx, d2.gy, d2.gz,
                   d3.ax, d3.ay, d3.az, d3.gx, d3.gy, d3.gz,
                   syncState);
        }
        // Poll at 100Hz (10ms) - I set DMP output rate to 100Hz during initialization.
        vTaskDelay(pdMS_TO_TICKS(10));  //pdMS_TO_TICKS converts milliseconds to ticks based on FreeRTOS tick rate.
                                        // 1ms ==> 1 tick if tick rate is 1000Hz.
                                        // How we know tick rate? It's defined in FreeRTOSConfig.h as configTICK_RATE_HZ
                                        //Can we change tick rate? Yes, but be careful as it affects timing across the entire RTOS.
    }
}


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h> // For memset In this code this is used to fill a structutre with zeroes if a sensor reading fails. 
                    // This prevents printing garbage values when a sensor is disconnected.

// --- Pin Definitions ---
#define I2C0_SDA_IO     21
#define I2C0_SCL_IO     22
#define I2C1_SDA_IO     17
#define I2C1_SCL_IO     16
#define I2C_FREQ_HZ     100000 //Can we increase this to 400kHz later?

// --- MPU6050 Registers ---
#define MPU6050_PWR_MGMT_1   0x6B //This register is used to wake up the MPU6050 from sleep mode.We have to write 0 to it.Then internal clock enables.
#define MPU6050_ACCEL_XOUT_H 0x3B  //Starting register for accelerometer data(X axis acceleration high byte)

static const char *TAG = "MPU_TRIO";

// --- Data Structure for MPU6050 Readings ---. This is a structure to hold all the sensor data read from the MPU6050 in one go.
typedef struct {
    int16_t acc_x; int16_t acc_y; int16_t acc_z;
    int16_t temp;
    int16_t gyro_x; int16_t gyro_y; int16_t gyro_z;
} mpu_data_t; //This is the name of the structure.

// --- I2C Initialization (Same as before) ---

void init_i2c_buses() {
    // I2C0 Initialization
    i2c_config_t conf0 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C0_SDA_IO,
        .scl_io_num = I2C0_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,// Enable internal pull-ups
        .scl_pullup_en = GPIO_PULLUP_ENABLE,// Enable internal pull-ups
        .master.clk_speed = I2C_FREQ_HZ,// Set clock speed.Hope to 400kHz later
    };
    i2c_param_config(I2C_NUM_0, &conf0); // Configure I2C0 with the settings in conf0
    i2c_driver_install(I2C_NUM_0, conf0.mode, 0, 0, 0); // Install I2C0 driver.

    i2c_config_t conf1 = {
        // I2C1 Initialization
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C1_SDA_IO,
        .scl_io_num = I2C1_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_NUM_1, &conf1);
    i2c_driver_install(I2C_NUM_1, conf1.mode, 0, 0, 0); //No need of bu
}

// --- NEW: I2C Scanner Function ---
//Because we use two I2C buses we can use this function to scan both buses separately without rewriting code again.
void i2c_scanner(i2c_port_t port) {
    //only check the known addresses for MPU6050
    uint8_t addrs[] = {0x68, 0x69};

    for (int i=0; i<2; i++){
        esp_err_t res = i2c_master_write_to_device(port, addrs[i], NULL, 0, 100 / portTICK_PERIOD_MS);
        if (res == ESP_OK) printf("Bus %d: Device found at 0x%02X\n", port, addrs[i]);
        else printf("Bus %d: Missing device at 0x%02X\n", port, addrs[i]);
    }
}

// --- Modified Read Function (Safe Version) ---
esp_err_t mpu6050_read_all(i2c_port_t port, uint8_t addr, mpu_data_t *result) { //returns a error code(esp_err_t), *result is a pointer to data structure.
    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    uint8_t buffer[14]; 

    // Check return value!
    esp_err_t ret = i2c_master_write_read_device(port, addr, &reg, 1, buffer, 14, 1000 / portTICK_PERIOD_MS);

    if (ret == ESP_OK) {
        result->acc_x  = (buffer[0] << 8) | buffer[1];
        result->acc_y  = (buffer[2] << 8) | buffer[3];
        result->acc_z  = (buffer[4] << 8) | buffer[5];
        result->gyro_x = (buffer[8] << 8) | buffer[9];
        result->gyro_y = (buffer[10] << 8) | buffer[11];
        result->gyro_z = (buffer[12] << 8) | buffer[13];
    } else {
        // Clear data if read fails so we don't print garbage
        memset(result, 0, sizeof(mpu_data_t));
    }
    return ret;
}

void mpu6050_wake(i2c_port_t port, uint8_t addr) {
    uint8_t data[2] = {MPU6050_PWR_MGMT_1, 0x00};
    i2c_master_write_to_device(port, addr, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

void app_main(void) {
    init_i2c_buses();

    // 1. Run Scanner First
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for power to stabilize
    i2c_scanner(I2C_NUM_0);
    i2c_scanner(I2C_NUM_1);

    // 2. Initialize sensors
    mpu6050_wake(I2C_NUM_0, 0x68);
    mpu6050_wake(I2C_NUM_0, 0x69);
    mpu6050_wake(I2C_NUM_1, 0x68);

    mpu_data_t s1, s2, s3;

    while (1) {
        esp_err_t ret1 = mpu6050_read_all(I2C_NUM_0, 0x68, &s1);
        esp_err_t ret2 = mpu6050_read_all(I2C_NUM_0, 0x69, &s2);
        esp_err_t ret3 = mpu6050_read_all(I2C_NUM_1, 0x68, &s3);

        printf("\n--- DATA ---\n");
        if (ret1 == ESP_OK) printf("S1 (0x68): Ax:%d\n", s1.acc_x);
        else printf("S1: DISCONNECTED\n");

        if (ret2 == ESP_OK) printf("S2 (0x69): Ax:%d\n", s2.acc_x);
        else printf("S2: DISCONNECTED\n");

        if (ret3 == ESP_OK) printf("S3 (Bus1): Ax:%d\n", s3.acc_x);
        else printf("S3: DISCONNECTED\n");

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
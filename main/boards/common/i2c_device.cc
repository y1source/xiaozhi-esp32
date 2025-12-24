#include "i2c_device.h"

#include <esp_log.h>
#include <cstring>
#include <vector>

#define TAG "I2cDevice"


I2cDevice::I2cDevice(i2c_master_bus_handle_t i2c_bus, uint8_t addr) {
    i2c_device_config_t i2c_device_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400 * 1000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &i2c_device_cfg, &i2c_device_));
    assert(i2c_device_ != NULL);
}

void I2cDevice::WriteReg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_, buffer, 2, 100));
}


/**
 * @brief 向I2C设备寄存器写入多个字节数据。内部使用临时缓冲区将寄存器地址和数据合并后一次性发送。
 * 
 * @param reg 要写入的目标寄存器地址
 * @param buffer 指向待写入数据缓冲区的指针
 * @param length 要写入的数据长度（字节数）
 */


void I2cDevice::WriteRegs(uint8_t reg, uint8_t* buffer, size_t length) {
    // 更安全的内存处理：使用vector避免内存分配问题
    std::vector<uint8_t> temp(length + 1);
    // 第一个字节是寄存器地址
    temp[0] = reg;
    // 将数据拷贝到临时缓冲区后续位置
    std::memcpy(temp.data() + 1, buffer, length);
    // 发送完整数据（寄存器地址+数据）
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_device_, temp.data(), temp.size(), 100));
}

uint8_t I2cDevice::ReadReg(uint8_t reg) {
    uint8_t buffer[1];
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, 1, 100));
    return buffer[0];
}

void I2cDevice::ReadRegs(uint8_t reg, uint8_t* buffer, size_t length) {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_device_, &reg, 1, buffer, length, 100));
}
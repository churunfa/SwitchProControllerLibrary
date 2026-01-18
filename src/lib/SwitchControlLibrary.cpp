//
// Created by churunfa on 2026/1/18.
//
//

#include "SwitchControlLibrary.h"

#include <cstring>
#include <iostream>

SwitchControlLibrary::SwitchControlLibrary() : switchReport{}, lastSwitchReport{} {
    resetAll();
    running = true;
    reportSize = sizeof(SwitchProReport);
    worker = std::thread(&SwitchControlLibrary::loop, this);
}

SwitchControlLibrary::~SwitchControlLibrary() {
    if (worker.joinable()) {
        // 必须等待线程结束
        worker.join();
    }
}

void SwitchControlLibrary::loop() {
    while (running) {
        if (port_name.empty()) {
            port_name = SerialPort::AutoDetectPort();
        }
        if (port_name.empty()) {
            std::cout << "[搜索中] 等待设备... \n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        if (!serial.IsConnected()) {
            if (!serial.Connect(port_name)) {
                std::cout << "[连接失败] 端口: " << port_name << std::endl;
                port_name.clear();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            std::cout << "[已连接] " << port_name << std::endl;
        }
        sendReport();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void SwitchControlLibrary::sendReport() {
    std::lock_guard<std::recursive_mutex> lock(reportMtx);

    if (memcmp(&lastSwitchReport, &switchReport, reportSize) == 0) {
        return;
    }
    // 先发头，失败就等下次重发
    if (!serial.Write(header, 2)) {
        std::cout << "header发送失败" << std::endl;
        return;
    }

    // 发送报表内容
    if (!serial.Write(&switchReport, reportSize)) {
        std::cout << "switchReport发送失败" << std::endl;
        return;
    }

    // 发送校验和
    uint8_t checkSum = 0;
    for (uint8_t i = 0; i < reportSize; i++) {
        checkSum ^= reinterpret_cast<uint8_t *>(&switchReport)[i];
    }
    if (!serial.Write(&checkSum, 1)) {
        std::cout << "checkSum发送失败" << std::endl;
        return;
    }
    // 更新lastSwitchReport
    memcpy(&lastSwitchReport, &switchReport, sizeof(SwitchProReport));
}


void SwitchControlLibrary::resetAll() {
    std::lock_guard<std::recursive_mutex> lock(reportMtx);

    std::memset(&switchReport, 0, sizeof(SwitchProReport));

    // 这些值一般不改
    switchReport.inputs.dummy = 0;
    switchReport.inputs.chargingGrip = 1;
    switchReport.inputs.buttonLeftSL = 1;
    switchReport.inputs.buttonLeftSR = 1;
    // 摇杆居中
    resetLeftAnalog();
    resetRightAnalog();
    // 体感只留重力
    resetIMU();
}

void SwitchControlLibrary::pressButton(const ButtonType button) {
    std::lock_guard<std::recursive_mutex> lock(reportMtx);

    auto* ptr = reinterpret_cast<uint8_t*>(&switchReport);
    const int byteIdx = button / 8;
    const int bitOffset = button % 8;
    ptr[byteIdx] |= 1 << bitOffset;  // 设置为 1

}

void SwitchControlLibrary::releaseButton(ButtonType button) {
    std::lock_guard<std::recursive_mutex> lock(reportMtx);

    auto* ptr = reinterpret_cast<uint8_t*>(&switchReport);
    const int byteIdx = button / 8;
    const int bitOffset = button % 8;
    ptr[byteIdx] &= ~(1 << bitOffset);  // 设置为 0
}


void SwitchControlLibrary::setIMU(int16_t accX, int16_t accY, int16_t accZ, int16_t gyroX, int16_t gyroY, int16_t gyroZ) {
    std::lock_guard<std::recursive_mutex> lock(reportMtx);

    // 先设置一样的，可能需要动态算一下
    for (auto & i : switchReport.imuData) {
        i.accX = accX;
        i.accY = accY;
        i.accZ = accZ;
        i.gyroX = gyroX;
        i.gyroY = gyroY;
        i.gyroZ = gyroZ;
    }
}

void SwitchControlLibrary::resetIMU() {
    setIMU(0, 0, -4096, 0, 0, 0);
}

void SwitchControlLibrary::setAnalogX(SwitchAnalog stick, const uint16_t x) {
    uint8_t *data = stick.data;
    data[0] = x & 0xFF;
    data[1] = (data[1] & 0xF0) | ((x >> 8) & 0x0F);
}

void SwitchControlLibrary::setAnalogY(SwitchAnalog stick, const uint16_t y) {
    uint8_t *data = stick.data;
    data[1] = (data[1] & 0x0F) | ((y & 0x0F) << 4);
    data[2] = (y >> 4) & 0xFF;
}

void SwitchControlLibrary::moveLeftAnalog(const uint16_t x, const uint16_t y) {
    std::lock_guard<std::recursive_mutex> lock(reportMtx);
    setAnalogX(switchReport.leftStick, x);
    setAnalogY(switchReport.leftStick, y);
}

void SwitchControlLibrary::resetLeftAnalog() {
    std::lock_guard<std::recursive_mutex> lock(reportMtx);
    moveLeftAnalog(2048, 2048);
}

void SwitchControlLibrary::moveRightAnalog(const uint16_t x, const uint16_t y) {
    std::lock_guard<std::recursive_mutex> lock(reportMtx);
    setAnalogX(switchReport.rightStick, x);
    setAnalogY(switchReport.rightStick, y);
}

void SwitchControlLibrary::resetRightAnalog() {
    std::lock_guard<std::recursive_mutex> lock(reportMtx);
    moveRightAnalog(2048, 2048);
}
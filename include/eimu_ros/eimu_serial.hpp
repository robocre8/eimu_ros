#pragma once

#include <libserial/SerialPort.h>

#include <iostream>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <thread>
#include <chrono>
#include <stdexcept>
#include <cmath>

/* =======================
   Protocol Definitions
   ======================= */

namespace eimu_serial
{

static constexpr uint8_t START_BYTE = 0xBB;

// Command IDs
static constexpr uint8_t READ_QUAT                  = 0x01;
static constexpr uint8_t READ_RPY                   = 0x02;
static constexpr uint8_t READ_RPY_VAR               = 0x03;
static constexpr uint8_t WRITE_RPY_VAR              = 0x04;
static constexpr uint8_t READ_ACC                   = 0x05;
static constexpr uint8_t READ_ACC_RAW               = 0x06;
static constexpr uint8_t READ_ACC_OFF               = 0x07;
static constexpr uint8_t WRITE_ACC_OFF              = 0x08;
static constexpr uint8_t READ_ACC_VAR               = 0x09;
static constexpr uint8_t WRITE_ACC_VAR              = 0x0A;
static constexpr uint8_t READ_GYRO                  = 0x0B;
static constexpr uint8_t READ_GYRO_RAW              = 0x0C;
static constexpr uint8_t READ_GYRO_OFF              = 0x0D;
static constexpr uint8_t WRITE_GYRO_OFF             = 0x0E;
static constexpr uint8_t READ_GYRO_VAR              = 0x0F;
static constexpr uint8_t WRITE_GYRO_VAR             = 0x10;
static constexpr uint8_t READ_MAG                   = 0x11;
static constexpr uint8_t READ_MAG_RAW               = 0x12;
static constexpr uint8_t READ_MAG_H_OFF             = 0x13;
static constexpr uint8_t WRITE_MAG_H_OFF            = 0x14;
static constexpr uint8_t READ_MAG_S_OFF0            = 0x15;
static constexpr uint8_t WRITE_MAG_S_OFF0           = 0x16;
static constexpr uint8_t READ_MAG_S_OFF1            = 0x17;
static constexpr uint8_t WRITE_MAG_S_OFF1           = 0x18;
static constexpr uint8_t READ_MAG_S_OFF2            = 0x19;
static constexpr uint8_t WRITE_MAG_S_OFF2           = 0x1A;
static constexpr uint8_t SET_I2C_ADDR               = 0x1B;
static constexpr uint8_t GET_I2C_ADDR               = 0x1C;
static constexpr uint8_t SET_FILTER_GAIN            = 0x1D;
static constexpr uint8_t GET_FILTER_GAIN            = 0x1E;
static constexpr uint8_t SET_FRAME_ID               = 0x1F;
static constexpr uint8_t GET_FRAME_ID               = 0x20;
static constexpr uint8_t RESET_PARAMS               = 0x21;
static constexpr uint8_t READ_QUAT_RPY              = 0x22;
static constexpr uint8_t READ_ACC_GYRO              = 0x23;
static constexpr uint8_t CLEAR_DATA_BUFFER          = 0x27;
static constexpr uint8_t READ_IMU_DATA              = 0x28;
static constexpr uint8_t SET_ACC_LPF_CUT_FREQ       = 0x29;
static constexpr uint8_t GET_ACC_LPF_CUT_FREQ       = 0x2A;
static constexpr uint8_t READ_LIN_ACC_RAW           = 0x2B;
static constexpr uint8_t READ_LIN_ACC               = 0x2C;

}


/* =======================
   EIMU Serial Client
   ======================= */
namespace eimu_serial
{

inline float round_to_dp(float value, int decimal_places) {
    const float multiplier = std::pow(10.0, decimal_places);
    return std::round(value * multiplier) / multiplier;
}

inline LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  case 460800:
    return LibSerial::BaudRate::BAUD_460800;
  case 921600:
    return LibSerial::BaudRate::BAUD_921600;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

}


namespace eimu_serial
{

class EIMUSerialClient
{
public:
    EIMUSerialClient() = default;
    ~EIMUSerialClient() { disconnect(); }

    /* ---------- Connection ---------- */

    void connect(const std::string& port,
                 int baudrate = 115200,
                 int timeout_ms = 100)
    {
        if (serial.IsOpen())
            serial.Close();

        serial.Open(port);

        serial.SetBaudRate(convert_baud_rate(baudrate));
        serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial.SetParity(LibSerial::Parity::PARITY_NONE);
        serial.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);

        timeout_ms_ = timeout_ms;

        // ---- Allow MCU to boot / flush startup bytes ----
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        // ---- Retry confirm motors ----
        constexpr int max_attempts = 10;

        for (int i = 0; i < max_attempts; ++i)
        {
            auto[success, _] = getWorldFrameId();
            if (success){
              std::cout << "EIMU Connected Successfully" << std::endl;
              return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        disconnect();
        throw std::runtime_error("EIMU could not connect, Please check connection and Try Again");
    }

    void disconnect()
    {
        if (serial.IsOpen())
            serial.Close();
    }

    bool connected() const
    {
        return serial.IsOpen();
    }

    /* ---------- High-Level API ---------- */

    std::tuple<bool, std::vector<float>> readQuat() { return read_data4(READ_QUAT); }
    std::tuple<bool, std::vector<float>> readRPY() { return read_data3(READ_RPY); }
    std::tuple<bool, std::vector<float>> readGyro() { return read_data3(READ_GYRO); }
    std::tuple<bool, std::vector<float>> readLinearAcc() { return read_data3(READ_LIN_ACC); }

    std::tuple<bool, std::vector<float>> readAccGyro() { return read_data6(READ_ACC_GYRO); }
    std::tuple<bool, std::vector<float>> readImuData() { return read_data9(READ_IMU_DATA); }

    std::tuple<bool, std::vector<float>> readRPYVariance(){ return read_data3(READ_RPY_VAR); }
    std::tuple<bool, std::vector<float>> readGyroVariance(){ return read_data3(READ_GYRO_VAR); }
    std::tuple<bool, std::vector<float>> readAccVariance(){ return read_data3(READ_ACC_VAR); }

    std::tuple<bool, std::vector<float>> readMag() { return read_data3(READ_MAG); }
    std::tuple<bool, std::vector<float>> readLinearAccRaw(){ return read_data3(READ_LIN_ACC_RAW); }
    std::tuple<bool, std::vector<float>> readAcc() { return read_data3(READ_ACC); }

    void setWorldFrameId(int frame_id) { write_data1(SET_FRAME_ID, (float)frame_id); }
    std::tuple<bool, float> getWorldFrameId(){ return read_data1(GET_FRAME_ID); }

    void setFilterGain(float gain) { write_data1(SET_FILTER_GAIN, gain); }
    std::tuple<bool, float> getFilterGain(){ return read_data1(GET_FILTER_GAIN); }

    void setI2cAddress(int address) { write_data1(SET_I2C_ADDR, (float)address); }
    std::tuple<bool, float> getI2cAddress(){ return read_data1(GET_I2C_ADDR); }

    void setAccFilterCF(float cf) { write_data1(SET_ACC_LPF_CUT_FREQ, cf); }
    std::tuple<bool, float> getAccFilterCF(){ return read_data1(GET_ACC_LPF_CUT_FREQ); }

    bool resetParams(){ 
      bool success;
      std::tie(success, std::ignore) = read_data1(RESET_PARAMS);
      return success;
    }

    bool clearDataBuffer(){ 
      bool success;
      std::tie(success, std::ignore) = read_data1(CLEAR_DATA_BUFFER);
      return success;
    }

    void writeRPYVariance(float r, float p, float y) { write_data3(WRITE_RPY_VAR, r, p, y); }

    std::tuple<bool, std::vector<float>> readAccRaw() { return read_data3(READ_ACC_RAW); }
    std::tuple<bool, std::vector<float>> readAccOffset(){ return read_data3(READ_ACC_OFF); }
    void writeAccOffset(float ax, float ay, float az) { write_data3(WRITE_ACC_OFF, ax, ay, az); }
    void writeAccVariance(float ax, float ay, float az) { write_data3(WRITE_ACC_VAR, ax, ay, az); }

    std::tuple<bool, std::vector<float>> readGyroRaw() { return read_data3(READ_GYRO_RAW); }
    std::tuple<bool, std::vector<float>> readGyroOffset(){ return read_data3(READ_GYRO_OFF); }
    void writeGyroOffset(float gx, float gy, float gz) { write_data3(WRITE_GYRO_OFF, gx, gy, gz); }
    void writeGyroVariance(float gx, float gy, float gz) { write_data3(WRITE_GYRO_VAR, gx, gy, gz); }

    std::tuple<bool, std::vector<float>> readMagRaw() { return read_data3(READ_MAG_RAW); }
    std::tuple<bool, std::vector<float>> readMagHardOffset(){ return read_data3(READ_MAG_H_OFF); }
    void writeMagHardOffset(float mx, float my, float mz) { write_data3(WRITE_MAG_H_OFF, mx, my, mz); }
    std::tuple<bool, std::vector<float>> readMagSoftOffset0(){ return read_data3(READ_MAG_S_OFF0); }
    void writeMagSoftOffset0(float mx, float my, float mz) { write_data3(WRITE_MAG_S_OFF0, mx, my, mz); }
    std::tuple<bool, std::vector<float>> readMagSoftOffset1(){ return read_data3(READ_MAG_S_OFF1); }
    void writeMagSoftOffset1(float mx, float my, float mz) { write_data3(WRITE_MAG_S_OFF1, mx, my, mz); }
    std::tuple<bool, std::vector<float>> readMagSoftOffset2(){ return read_data3(READ_MAG_S_OFF2); }
    void writeMagSoftOffset2(float mx, float my, float mz) { write_data3(WRITE_MAG_S_OFF2, mx, my, mz); }

private:
    LibSerial::SerialPort serial;
    int timeout_ms_;

    /* ---------- Packet Helpers ---------- */

    void flush_rx()
    {
        if (!serial.IsOpen()) return;
        try {
            serial.FlushInputBuffer();  // clears RX
        } catch (...) {
        }
    }

    void flush_tx()
    {
        if (!serial.IsOpen()) return;
        try {
            serial.DrainWriteBuffer();  // clears TX
        } catch (...) {
        }
    }

    void sendPacket(uint8_t cmd,
                    const std::vector<uint8_t>& payload = {})
    {
        if (!serial.IsOpen()) {
          throw std::runtime_error("Serial port not connected");
        }
        flush_rx();
        
        std::vector<uint8_t> packet;
        packet.reserve(4 + payload.size());

        packet.push_back(START_BYTE);
        packet.push_back(cmd);
        packet.push_back(static_cast<uint8_t>(payload.size()));
        packet.insert(packet.end(), payload.begin(), payload.end());

        uint8_t checksum = 0;
        for (uint8_t b : packet)
            checksum += b;

        packet.push_back(checksum);

        serial.Write(packet);
        serial.DrainWriteBuffer();
    }

    std::pair<bool, std::vector<float>>
    readFloats(size_t count)
    {
        const size_t bytes_needed = count * sizeof(float);
        std::vector<uint8_t> buf(bytes_needed);

        try {
            serial.Read(buf, bytes_needed, timeout_ms_);
            if (buf.size() != bytes_needed){
              flush_rx();
              return {false, std::vector<float>(count, 0.0f)};
            }
        }
        catch (const LibSerial::ReadTimeout&) {
            flush_rx();   // critical for stream resync
            return {false, std::vector<float>(count, 0.0f)};
        }
        catch (...) {
            flush_rx();
            return {false, std::vector<float>(count, 0.0f)};
        }

        std::vector<float> values(count);
        std::memcpy(values.data(), buf.data(), bytes_needed);
        return {true, values};
    }

    /* ---------- Generic Data ---------- */

    void write_data1(uint8_t cmd, float val, uint8_t pos = 0)
    {
        std::vector<uint8_t> payload(1 + sizeof(float));
        payload[0] = pos;
        std::memcpy(&payload[1], &val, sizeof(float));
        sendPacket(cmd, payload);
    }

    void write_data3(uint8_t cmd, float a, float b, float c)
    {
        std::vector<uint8_t> payload(3 * sizeof(float));
        std::memcpy(&payload[0], &a, sizeof(float));
        std::memcpy(&payload[4], &b, sizeof(float));
        std::memcpy(&payload[8], &c, sizeof(float));
        sendPacket(cmd, payload);
    }

    void write_data4(uint8_t cmd, float a, float b, float c, float d)
    {
        std::vector<uint8_t> payload(4 * sizeof(float));
        std::memcpy(&payload[0], &a, sizeof(float));
        std::memcpy(&payload[4], &b, sizeof(float));
        std::memcpy(&payload[8], &c, sizeof(float));
        std::memcpy(&payload[12], &d, sizeof(float));
        sendPacket(cmd, payload);
    }

    std::tuple<bool, float>
    read_data1(uint8_t cmd, uint8_t pos = 0)
    {
        float dummy = 0.0f;
        std::vector<uint8_t> payload(1 + sizeof(float));
        payload[0] = pos;
        std::memcpy(&payload[1], &dummy, sizeof(float));

        sendPacket(cmd, payload);

        auto [ok, vals] = readFloats(1);
        return {ok, round_to_dp(vals[0],6)};
    }

    std::tuple<bool, std::vector<float>>
    read_data3(uint8_t cmd)
    {
        sendPacket(cmd);
        auto [ok, vals] = readFloats(3);
        std::vector<float> value(3);
        value[0] = round_to_dp(vals[0],6);
        value[1] = round_to_dp(vals[1],6);
        value[2] = round_to_dp(vals[2],6);
        return {ok, value};
    }

    std::tuple<bool, std::vector<float>>
    read_data4(uint8_t cmd)
    {
        sendPacket(cmd);
        auto [ok, vals] = readFloats(4);
        std::vector<float> value(4);
        value[0] = round_to_dp(vals[0],6);
        value[1] = round_to_dp(vals[1],6);
        value[2] = round_to_dp(vals[2],6);
        value[3] = round_to_dp(vals[3],6);
        return {ok, value};
    }

    std::tuple<bool, std::vector<float>>
    read_data6(uint8_t cmd)
    {
        sendPacket(cmd);
        auto [ok, vals] = readFloats(6);
        std::vector<float> value(6);
        value[0] = round_to_dp(vals[0],6);
        value[1] = round_to_dp(vals[1],6);
        value[2] = round_to_dp(vals[2],6);
        value[3] = round_to_dp(vals[3],6);
        value[4] = round_to_dp(vals[4],6);
        value[5] = round_to_dp(vals[5],6);
        return {ok, value};
    }

    std::tuple<bool, std::vector<float>>
    read_data9(uint8_t cmd)
    {
        sendPacket(cmd);
        auto [ok, vals] = readFloats(9);
        std::vector<float> value(9);
        value[0] = round_to_dp(vals[0],6);
        value[1] = round_to_dp(vals[1],6);
        value[2] = round_to_dp(vals[2],6);
        value[3] = round_to_dp(vals[3],6);
        value[4] = round_to_dp(vals[4],6);
        value[5] = round_to_dp(vals[5],6);
        value[6] = round_to_dp(vals[6],6);
        value[7] = round_to_dp(vals[7],6);
        value[8] = round_to_dp(vals[8],6);
        return {ok, value};
    }
};

}
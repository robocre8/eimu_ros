#ifndef EIMU_HPP
#define EIMU_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <tuple>
#include <chrono>
#include <thread>
#include <cstring>  // memcpy
#include <stdexcept> // For standard exception types
#include <cmath> // Required for std::round, std::ceil, std::floor
#include <libserial/SerialPort.h>

double round_to_dp(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::round(value * multiplier) / multiplier;
}

// Serial Protocol Command IDs -------------
const uint8_t START_BYTE = 0xBB;
const uint8_t READ_QUAT = 0x01;
const uint8_t READ_RPY = 0x02;
const uint8_t READ_RPY_VAR = 0x03;
const uint8_t READ_ACC = 0x05;
const uint8_t READ_ACC_VAR = 0x09;
const uint8_t READ_GYRO = 0x0B;
const uint8_t READ_GYRO_VAR = 0x0F;
const uint8_t READ_MAG = 0x11;
const uint8_t SET_FILTER_GAIN = 0x1D;
const uint8_t GET_FILTER_GAIN = 0x1E;
const uint8_t SET_FRAME_ID = 0x1F;
const uint8_t GET_FRAME_ID = 0x20;
const uint8_t READ_QUAT_RPY = 0x22;
const uint8_t READ_ACC_GYRO = 0x23;
const uint8_t CLEAR_DATA_BUFFER = 0x27;
const uint8_t READ_IMU_DATA = 0x28;
const uint8_t READ_LIN_ACC_RAW = 0x2B;
const uint8_t READ_LIN_ACC = 0x2C;
//---------------------------------------------

LibSerial::BaudRate convert_baud_rate(int baud_rate)
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


class EIMU
{

public:
  EIMU() = default;

  void connect(const std::string &serial_device, int32_t baud_rate = 57600, int32_t timeout_ms = 10)
  {
    try {
      timeout_ms_ = timeout_ms;
      serial_conn_.Open(serial_device);
      serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
      serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    } catch (const LibSerial::OpenFailed&) {
        std::cerr << "Failed to open serial port!" << std::endl;
    }
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  bool clearDataBuffer()
  {
    bool success;
    std::tie(success, std::ignore) = read_data1(CLEAR_DATA_BUFFER);
    return success;
  }

  void setWorldFrameId(int id)
  {
    write_data1(SET_FRAME_ID, (float)id);
  }

  std::tuple<bool, int> getWorldFrameId()
  {
    bool success; float frame_id;
    std::tie(success, frame_id) = read_data1(GET_FRAME_ID);
    return std::make_tuple(success, (int)frame_id);
  }

  void setFilterGain(float gain)
  {
    write_data1(SET_FILTER_GAIN, gain);
  }

  std::tuple<bool, float> getFilterGain()
  {
    bool success; float gain;
    std::tie(success, gain) = read_data1(GET_FILTER_GAIN);
    return std::make_tuple(success, gain);
  }

  std::tuple<bool, float, float, float, float> readQuat()
  {
    bool success; float qw, qx, qy, qz;
    std::tie(success, qw, qx, qy, qz) = read_data4(READ_QUAT);
    qw = round_to_dp(qw, 6);
    qx = round_to_dp(qx, 6);
    qy = round_to_dp(qy, 6);
    qz = round_to_dp(qz, 6);
    return std::make_tuple(success, qw, qx, qy, qz);
  }

  std::tuple<bool, float, float, float> readLinearAcc()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = read_data3(READ_LIN_ACC);
    x = round_to_dp(x, 6);
    y = round_to_dp(y, 6);
    z = round_to_dp(z, 6);
    return std::make_tuple(success, x, y, z);
  }

  // std::tuple<bool, float, float, float> readAcc()
  // {
  //   bool success; float x, y, z;
  //   std::tie(success, x, y, z) = read_data3(READ_ACC);
  //   x = round_to_dp(x, 6);
  //   y = round_to_dp(y, 6);
  //   z = round_to_dp(z, 6);
  //   return std::make_tuple(success, x, y, z);
  // }

  std::tuple<bool, float, float, float> readGyro()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = read_data3(READ_GYRO);
    x = round_to_dp(x, 6);
    y = round_to_dp(y, 6);
    z = round_to_dp(z, 6);
    return std::make_tuple(success, x, y, z);
  }

  std::tuple<bool, float, float, float> readRPY()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = read_data3(READ_RPY);
    x = round_to_dp(x, 6);
    y = round_to_dp(y, 6);
    z = round_to_dp(z, 6);
    return std::make_tuple(success, x, y, z);
  }

  // std::tuple<bool, float, float, float> readMag()
  // {
  //   bool success; float x, y, z;
  //   std::tie(success, x, y, z) = read_data3(READ_MAG);
  //   x = round_to_dp(x, 6);
  //   y = round_to_dp(y, 6);
  //   z = round_to_dp(z, 6);
  //   return std::make_tuple(success, x, y, z);
  // }

  std::tuple<bool, float, float, float> readAccVariance()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = read_data3(READ_ACC_VAR);
    x = round_to_dp(x, 6);
    y = round_to_dp(y, 6);
    z = round_to_dp(z, 6);
    return std::make_tuple(success, x, y, z);
  }

  std::tuple<bool, float, float, float> readGyroVariance()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = read_data3(READ_GYRO_VAR);
    x = round_to_dp(x, 6);
    y = round_to_dp(y, 6);
    z = round_to_dp(z, 6);
    return std::make_tuple(success, x, y, z);
  }

  std::tuple<bool, float, float, float> readRPYVariance()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = read_data3(READ_RPY_VAR);
    x = round_to_dp(x, 6);
    y = round_to_dp(y, 6);
    z = round_to_dp(z, 6);
    return std::make_tuple(success, x, y, z);
  }

  std::tuple<bool, float, float, float, float, float, float> readAccGyro()
  {
    bool success; float ax, ay, az, gx, gy, gz;
    std::tie(success, ax, ay, az, gx, gy, gz) = read_data6(READ_ACC_GYRO);
    ax = round_to_dp(ax, 6);
    ay = round_to_dp(ay, 6);
    az = round_to_dp(az, 6);
    gx = round_to_dp(gx, 6);
    gy = round_to_dp(gy, 6);
    gz = round_to_dp(gz, 6);
    return std::make_tuple(success, ax, ay, az, gx, gy, gz);
  }

  std::tuple<bool, float, float, float, float, float, float, float, float, float> readImuData()
  {
    bool success; float r, p, y, ax, ay, az, gx, gy, gz;
    std::tie(success, r, p, y, ax, ay, az, gx, gy, gz) = read_data9(READ_IMU_DATA);
    r = round_to_dp(r, 6);
    p = round_to_dp(p, 6);
    y = round_to_dp(y, 6);
    ax = round_to_dp(ax, 6);
    ay = round_to_dp(ay, 6);
    az = round_to_dp(az, 6);
    gx = round_to_dp(gx, 6);
    gy = round_to_dp(gy, 6);
    gz = round_to_dp(gz, 6);
    return std::make_tuple(success, r, p, y, ax, ay, az, gx, gy, gz);
  }


private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;

  uint8_t calcChecksum(const std::vector<uint8_t>& packet) {
    uint32_t sum = 0;
    for (auto b : packet) sum += b;
    return sum & 0xFF;
  }

  void send_packet_without_payload(uint8_t cmd) {
    uint8_t len = 0;
    std::vector<uint8_t> packet = {START_BYTE, cmd, len}; // no payload
    uint8_t checksum = calcChecksum(packet);
    packet.push_back(checksum);
    serial_conn_.Write(packet);
    // serial_conn_.DrainWriteBuffer();
  }

  void send_packet_with_payload(uint8_t cmd, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> packet = {START_BYTE, cmd, (uint8_t)payload.size()};
    packet.insert(packet.end(), payload.begin(), payload.end());
    uint8_t checksum = calcChecksum(packet);
    packet.push_back(checksum);
    serial_conn_.Write(packet);
    // serial_conn_.DrainWriteBuffer();
  }

  std::tuple<bool, float> read_packet1() {
    std::vector<uint8_t> payload;
    float val;
    try
    {
      serial_conn_.Read(payload, 4, timeout_ms_);
      if (payload.size() < 4) {
        // std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 1 values" << std::endl;
        return std::make_tuple(false, 0.0);
      }
      std::memcpy(&val, payload.data(), sizeof(float)); // little-endian assumed
      return std::make_tuple(true, val);
    }
    catch(const LibSerial::ReadTimeout &e)
    {
      // std::cerr << "[LIB SERIAL ERROR]: ReadTimeout" << std::endl;
      return std::make_tuple(false, 0.0);
    }
  }

  std::tuple<bool, float, float, float> read_packet3() {
    std::vector<uint8_t> payload;
    float val0, val1, val2;
    try
    {
      serial_conn_.Read(payload, 12, timeout_ms_);
      if (payload.size() < 12) {
        // std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 3 values" << std::endl;
        return std::make_tuple(false, 0.0, 0.0, 0.0);
      }
      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      return std::make_tuple(true, val0, val1, val2);
    }
    catch(const LibSerial::ReadTimeout &e)
    {
      // std::cerr << "[LIB SERIAL ERROR]: ReadTimeout" << std::endl;
      return std::make_tuple(false, 0.0, 0.0, 0.0);
    }
  }

  std::tuple<bool, float, float, float, float> read_packet4() {
    std::vector<uint8_t> payload;
    float val0, val1, val2, val3;
    try
    {
      serial_conn_.Read(payload, 16, timeout_ms_);
      if (payload.size() < 16) {
        // std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 4 values" << std::endl;
        return std::make_tuple(false, 0.0, 0.0, 0.0, 0.0);
      }
      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
      return std::make_tuple(true, val0, val1, val2, val3);
    }
    catch(const LibSerial::ReadTimeout &e)
    {
      // std::cerr << "[LIB SERIAL ERROR]: ReadTimeout" << std::endl;
      return std::make_tuple(false, 0.0, 0.0, 0.0, 0.0);
    }  
  }

  std::tuple<bool, float, float, float, float, float, float> read_packet6() {
    std::vector<uint8_t> payload;
    float val0, val1, val2, val3, val4, val5;
    try
    {
      serial_conn_.Read(payload, 24, timeout_ms_);
      if (payload.size() < 24) {
        // std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 6 values" << std::endl;
        return std::make_tuple(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      }
      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
      std::memcpy(&val4, payload.data() + 16, sizeof(float));
      std::memcpy(&val5, payload.data() + 20, sizeof(float));
      return std::make_tuple(true, val0, val1, val2, val3, val4, val5);
    }
    catch(const LibSerial::ReadTimeout &e)
    {
      // std::cerr << "[LIB SERIAL ERROR]: ReadTimeout" << std::endl;
      return std::make_tuple(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
  }

  std::tuple<bool, float, float, float, float, float, float, float, float, float> read_packet9() {
    std::vector<uint8_t> payload;
    float val0, val1, val2, val3, val4, val5, val6, val7, val8;
    try
    {
      serial_conn_.Read(payload, 36, timeout_ms_);
      if (payload.size() < 36) {
        // std::cerr << "[EPMC SERIAL ERROR]: Timeout while reading 9 values" << std::endl;
        return std::make_tuple(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      }
      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
      std::memcpy(&val4, payload.data() + 16, sizeof(float));
      std::memcpy(&val5, payload.data() + 20, sizeof(float));
      std::memcpy(&val6, payload.data() + 24, sizeof(float));
      std::memcpy(&val7, payload.data() + 28, sizeof(float));
      std::memcpy(&val8, payload.data() + 32, sizeof(float));
      return std::make_tuple(true, val0, val1, val2, val3, val4, val5, val6, val7, val8);
    }
    catch(const LibSerial::ReadTimeout &e)
    {
      // std::cerr << "[LIB SERIAL ERROR]: ReadTimeout" << std::endl;
      return std::make_tuple(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
  }

  // ------------------- High-Level Wrappers -------------------
  void write_data1(uint8_t cmd, float val, uint8_t pos=100) {
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      std::memcpy(&payload[1], &val, sizeof(float));
      send_packet_with_payload(cmd, payload);
  }

  void write_data3(uint8_t cmd, float a, float b, float c) {
      std::vector<uint8_t> payload(3 * sizeof(float));
      std::memcpy(&payload[0],  &a, sizeof(float));
      std::memcpy(&payload[4],  &b, sizeof(float));
      std::memcpy(&payload[8],  &c, sizeof(float));
      send_packet_with_payload(cmd, payload);
  }

  std::tuple<bool, float> read_data1(uint8_t cmd, uint8_t pos=100) {
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      send_packet_with_payload(cmd, payload);
      bool success; float val;
      std::tie(success, val) = read_packet1();
      return std::make_tuple(success, val);
  }

  std::tuple<bool, float, float, float> read_data3(uint8_t cmd) {
      send_packet_without_payload(cmd);
      bool success; float a, b, c;
      std::tie(success, a, b, c) = read_packet3();
      return std::make_tuple(success, a, b, c);
  }

  std::tuple<bool, float, float, float, float> read_data4(uint8_t cmd) {
      send_packet_without_payload(cmd);
      bool success; float a, b, c, d;
      std::tie(success, a, b, c, d) = read_packet4();
      return std::make_tuple(success, a, b, c, d);
  }

  std::tuple<bool, float, float, float, float, float, float> read_data6(uint8_t cmd) {
      send_packet_without_payload(cmd);
      bool success; float a, b, c, d, e, f;
      std::tie(success, a, b, c, d, e, f) = read_packet6();
      return std::make_tuple(success, a, b, c, d, e, f);
  }

  std::tuple<bool, float, float, float, float, float, float, float, float, float> read_data9(uint8_t cmd) {
      send_packet_without_payload(cmd);
      bool success; float a, b, c, d, e, f, g, h, i;
      std::tie(success, a, b, c, d, e, f, g, h, i) = read_packet9();
      return std::make_tuple(success, a, b, c, d, e, f, g, h, i);
  }

};

#endif
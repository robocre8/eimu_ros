#ifndef EIMU_HPP
#define EIMU_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <tuple>
#include <chrono>
#include <thread>
#include <cstring>  // memcpy
#include <libserial/SerialPort.h>

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
const uint8_t GET_FILTER_GAIN = 0x1E;
const uint8_t SET_FRAME_ID = 0x1F;
const uint8_t GET_FRAME_ID = 0x20;
const uint8_t READ_QUAT_RPY = 0x22;
const uint8_t READ_ACC_GYRO = 0x23;
const uint8_t CLEAR_DATA_BUFFER = 0x27;
const uint8_t READ_IMU_DATA = 0x28;
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

  void connect(const std::string &serial_device, int32_t baud_rate = 115200, int32_t timeout_ms = 100)
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

  int setWorldFrameId(int id)
  {
    float res = write_data1(SET_FRAME_ID, 0, (float)id);
    return (int)res;
  }

  int getWorldFrameId()
  {
    float id = read_data1(GET_FRAME_ID, 0);
    return (int)id;
  }

  float getFilterGain()
  {
    float gain = read_data1(GET_FILTER_GAIN, 0);
    return gain;
  }

  bool readAcc(float &x, float &y, float &z)
  {
    return read_data3(READ_ACC, x, y, z);
  }

  bool readAccVariance(float &x, float &y, float &z)
  {
    return read_data3(READ_ACC_VAR, x, y, z);
  }
  
  bool readGyro(float &x, float &y, float &z)
  {
    return read_data3(READ_GYRO, x, y, z);
  }

  bool readGyroVariance(float &x, float &y, float &z)
  {
    return read_data3(READ_GYRO_VAR, x, y, z);
  }

  bool readRPY(float &x, float &y, float &z)
  {
    return read_data3(READ_RPY, x, y, z);
  }

  bool readRPYVariance(float &x, float &y, float &z)
  {
    return read_data3(READ_RPY_VAR, x, y, z);
  }

  bool readQuat(float &qw, float &qx, float &qy, float &qz)
  {
    return read_data4(READ_QUAT, qw, qx, qy, qz);
  }

  bool readMag(float &x, float &y, float &z)
  {
    return read_data3(READ_MAG, x, y, z);
  }

  bool readQuatRPY(float &qw, float& qx, float &qy, float &qz, float &r, float& p, float &y)
  {
    float dummy_data;
    return read_data8(READ_QUAT_RPY, qw, qx, qy, qz, r, p, y, dummy_data);
  }

  bool readAccGyro(float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
  {
    return read_data6(READ_ACC_GYRO, ax, ay, az, gx, gy, gz);
  }

  bool readImuData(float &r, float &p, float &y, float &ax, float &ay, float &az, float &gx, float &gy, float &gz)
  {
    return read_data9(READ_IMU_DATA, r, p, y, ax, ay, az, gx, gy, gz);
  }

  int clearDataBuffer()
  {
    float res = write_data1(CLEAR_DATA_BUFFER, 0, 0.0);
    return (int)res;
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
      std::vector<uint8_t> packet = {START_BYTE, cmd, 0}; // no payload
      uint8_t checksum = calcChecksum(packet);
      packet.push_back(checksum);
      serial_conn_.Write(packet);
  }

  void send_packet_with_payload(uint8_t cmd, const std::vector<uint8_t>& payload) {
      std::vector<uint8_t> packet = {START_BYTE, cmd, (uint8_t)payload.size()};
      packet.insert(packet.end(), payload.begin(), payload.end());
      uint8_t checksum = calcChecksum(packet);
      packet.push_back(checksum);
      serial_conn_.Write(packet);
  }

  bool read_packet1(float &val) {
      std::vector<uint8_t> payload;
      try {
        serial_conn_.Read(payload, 4, timeout_ms_);
        if (payload.size() < 4) {
          std::cerr << "[EPMC SERIAL COMM]: Incomplete packet — received only "
                    << payload.size() << " bytes instead of 4." << std::endl;
          return false;
        }
      } catch (const LibSerial::ReadTimeout&) {
          std::cerr << "[EPMC SERIAL COMM]: Timeout while reading packet1" << std::endl;
          return false;
      }
      std::memcpy(&val, payload.data(), sizeof(float)); // little-endian assumed
      return true;
  }

  bool read_packet2(float &val0, float &val1) {
      std::vector<uint8_t> payload;
      try {
        serial_conn_.Read(payload, 8, timeout_ms_);
        if (payload.size() < 8) {
          std::cerr << "[EPMC SERIAL COMM]: Incomplete packet — received only "
                    << payload.size() << " bytes instead of 8." << std::endl;
          return false;
        }
      } catch (const LibSerial::ReadTimeout&) {
          std::cerr << "[EPMC SERIAL COMM]: Timeout while reading packet2" << std::endl;
          return false;
      }
      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      return true;
  }

  bool read_packet3(float &val0, float &val1, float &val2) {
      std::vector<uint8_t> payload;
      try {
        serial_conn_.Read(payload, 12, timeout_ms_);
        if (payload.size() < 12) {
          std::cerr << "[EPMC SERIAL COMM]: Incomplete packet — received only "
                    << payload.size() << " bytes instead of 12." << std::endl;
          serial_conn_.FlushIOBuffers();  // clears stale bytes
          return false;
        }
      } catch (const LibSerial::ReadTimeout&) {
          std::cerr << "[EPMC SERIAL COMM]: Timeout while reading packet3" << std::endl;
          return false;
      }

      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      return true;
  }

  bool read_packet4(float &val0, float &val1, float &val2, float &val3) {
      std::vector<uint8_t> payload;
      try {
        serial_conn_.Read(payload, 16, timeout_ms_);
        if (payload.size() < 16) {
          std::cerr << "[EPMC SERIAL COMM]: Incomplete packet — received only "
                    << payload.size() << " bytes instead of 16." << std::endl;
          serial_conn_.FlushIOBuffers();  // clears stale bytes
          return false;
        }
      } catch (const LibSerial::ReadTimeout&) {
          std::cerr << "[EPMC SERIAL COMM]: Timeout while reading packet4" << std::endl;
          return false;
      }

      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
      return true;
  }

  bool read_packet6(float &val0, float &val1, float &val2, float &val3, float &val4, float &val5) {
      std::vector<uint8_t> payload;
      try {
        serial_conn_.Read(payload, 24, timeout_ms_);
        if (payload.size() < 24) {
          std::cerr << "[EPMC SERIAL COMM]: Incomplete packet — received only "
                    << payload.size() << " bytes instead of 24." << std::endl;
          serial_conn_.FlushIOBuffers();  // clears stale bytes
          return false;
        }
      } catch (const LibSerial::ReadTimeout&) {
          std::cerr << "[EPMC SERIAL COMM]: Timeout while reading packet6" << std::endl;
          return false;
      }

      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
      std::memcpy(&val4, payload.data() + 16, sizeof(float));
      std::memcpy(&val5, payload.data() + 20, sizeof(float));
      return true;
  }


  bool read_packet8(float &val0, float &val1, float &val2, float &val3, float &val4, float &val5, float &val6, float &val7) {
      std::vector<uint8_t> payload;
      try {
        serial_conn_.Read(payload, 32, timeout_ms_);
        if (payload.size() < 32) {
          std::cerr << "[EPMC SERIAL COMM]: Incomplete packet — received only "
                    << payload.size() << " bytes instead of 32." << std::endl;
          serial_conn_.FlushIOBuffers();  // clears stale bytes
          return false;
        }
      } catch (const LibSerial::ReadTimeout&) {
          std::cerr << "[EPMC SERIAL COMM]: Timeout while reading packet8" << std::endl;
          return false;
      }

      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
      std::memcpy(&val4, payload.data() + 16, sizeof(float));
      std::memcpy(&val5, payload.data() + 20, sizeof(float));
      std::memcpy(&val6, payload.data() + 24, sizeof(float));
      std::memcpy(&val7, payload.data() + 28, sizeof(float));
      return true;
  }

  bool read_packet9(float &val0, float &val1, float &val2, float &val3, float &val4, float &val5, float &val6, float &val7, float &val8) {
      std::vector<uint8_t> payload;
      try {
        serial_conn_.Read(payload, 36, timeout_ms_);
        if (payload.size() < 36) {
          std::cerr << "[EPMC SERIAL COMM]: Incomplete packet — received only "
                    << payload.size() << " bytes instead of 36." << std::endl;
          serial_conn_.FlushIOBuffers();  // clears stale bytes
          return false;
        }
      } catch (const LibSerial::ReadTimeout&) {
          std::cerr << "[EPMC SERIAL COMM]: Timeout while reading packet9" << std::endl;
          return false;
      }

      std::memcpy(&val0, payload.data() + 0, sizeof(float));
      std::memcpy(&val1, payload.data() + 4, sizeof(float));
      std::memcpy(&val2, payload.data() + 8, sizeof(float));
      std::memcpy(&val3, payload.data() + 12, sizeof(float));
      std::memcpy(&val4, payload.data() + 16, sizeof(float));
      std::memcpy(&val5, payload.data() + 20, sizeof(float));std::memcpy(&val7, payload.data() + 28, sizeof(float));
      std::memcpy(&val6, payload.data() + 24, sizeof(float));
      std::memcpy(&val7, payload.data() + 28, sizeof(float));
      std::memcpy(&val8, payload.data() + 28, sizeof(float));
      return true;
  }

  // ------------------- High-Level Wrappers -------------------
  float write_data1(uint8_t cmd, uint8_t pos, float val) {
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      std::memcpy(&payload[1], &val, sizeof(float));
      send_packet_with_payload(cmd, payload);
      float data;
      if (!read_packet1(data)) {
        // std::cerr << "EPMC SERIAL COMM: Failed to read packet!" << std::endl;
        return 0.0;
      }
      return data;
  }

  float read_data1(uint8_t cmd, uint8_t pos) {
      float zero = 0.0f;
      std::vector<uint8_t> payload(sizeof(uint8_t) + sizeof(float));
      payload[0] = pos;
      std::memcpy(&payload[1], &zero, sizeof(float));
      send_packet_with_payload(cmd, payload);
      float val;
      if (!read_packet1(val)) {
        // std::cerr << "EPMC SERIAL COMM: Failed to read packet!" << std::endl;
        return 0.0;
      }
      return val;
  }

  void write_data2(uint8_t cmd, float a, float b) {
      std::vector<uint8_t> payload(2 * sizeof(float));
      std::memcpy(&payload[0],  &a, 4);
      std::memcpy(&payload[4],  &b, 4);
      send_packet_with_payload(cmd, payload);
  }

  bool read_data2(uint8_t cmd, float &a, float &b) {
      send_packet_without_payload(cmd);
      if (read_packet2(a, b)) 
        return true;
      else 
        return false;
  }

  void write_data3(uint8_t cmd, float a, float b, float c) {
      std::vector<uint8_t> payload(3 * sizeof(float));
      std::memcpy(&payload[0],  &a, 4);
      std::memcpy(&payload[4],  &b, 4);
      std::memcpy(&payload[8],  &c, 4);
      send_packet_with_payload(cmd, payload);
  }

  bool read_data3(uint8_t cmd, float &a, float &b, float &c) {
      send_packet_without_payload(cmd);
      if (read_packet3(a, b, c)) 
        return true;
      else 
        return false;
  }

  void write_data4(uint8_t cmd, float a, float b, float c, float d) {
      std::vector<uint8_t> payload(4 * sizeof(float));
      std::memcpy(&payload[0],  &a, 4);
      std::memcpy(&payload[4],  &b, 4);
      std::memcpy(&payload[8],  &c, 4);
      std::memcpy(&payload[12], &d, 4);
      send_packet_with_payload(cmd, payload);
  }

  bool read_data4(uint8_t cmd, float &a, float &b, float &c, float &d) {
      send_packet_without_payload(cmd);
      if (read_packet4(a, b, c, d)) 
        return true;
      else 
        return false;
  }

  bool read_data6(uint8_t cmd, float &a, float &b, float &c, float &d, float &e, float &f) {
      send_packet_without_payload(cmd);
      if (read_packet6(a, b, c, d, e, f)) 
        return true;
      else 
        return false;
  }

  bool read_data8(uint8_t cmd, float &a, float &b, float &c, float &d, float &e, float &f, float &g, float &h) {
      send_packet_without_payload(cmd);
      if (read_packet8(a, b, c, d, e, f, g, h)) 
        return true;
      else 
        return false;
  }

  bool read_data9(uint8_t cmd, float &a, float &b, float &c, float &d, float &e, float &f, float &g, float &h, float &i) {
      send_packet_without_payload(cmd);
      if (read_packet9(a, b, c, d, e, f, g, h, i)) 
        return true;
      else 
        return false;
  }

};

#endif
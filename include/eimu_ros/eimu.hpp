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
const int READ_RPY = 10;
const int READ_RPY_VAR = 11;
const int WRITE_RPY_VAR = 12;

const int READ_GYRO = 13;
const int READ_GYRO_RAW = 14;
const int READ_GYRO_OFF = 15;
const int WRITE_GYRO_OFF = 16;
const int READ_GYRO_VAR = 17;
const int WRITE_GYRO_VAR = 18;

const int READ_ACC = 19;
const int READ_ACC_RAW = 20;
const int READ_ACC_OFF = 21;
const int WRITE_ACC_OFF = 22;
const int READ_ACC_VAR = 23;
const int WRITE_ACC_VAR = 24;
const int READ_LIN_ACC_RAW = 25;
const int READ_LIN_ACC = 26;
const int SET_ACC_LPF_CUT_FREQ = 27;
const int GET_ACC_LPF_CUT_FREQ = 28;

const int READ_MAG = 29;
const int READ_MAG_RAW = 30;
const int READ_MAG_H_OFF = 31;
const int WRITE_MAG_H_OFF = 32;
const int READ_MAG_S_OFF0 = 33;
const int WRITE_MAG_S_OFF0 = 34;
const int READ_MAG_S_OFF1 = 35;
const int WRITE_MAG_S_OFF1 = 36;
const int READ_MAG_S_OFF2 = 37;
const int WRITE_MAG_S_OFF2 = 38;

const int SET_I2C_ADDR = 39;
const int GET_I2C_ADDR = 40;
const int SET_FILTER_GAIN = 41;
const int GET_FILTER_GAIN = 42;
const int SET_FRAME_ID = 43;
const int GET_FRAME_ID = 44;
const int RESET = 45;
const int CLEAR = 46;
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
      serial_conn_.FlushIOBuffers();
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
    std::tie(success, std::ignore, std::ignore, std::ignore) = recv((float)CLEAR);
    return success;
  }

  void setWorldFrameId(int id)
  {
    send((float)SET_FRAME_ID, 0.0, (float)id);
  }

  std::tuple<bool, int> getWorldFrameId()
  {
    bool success; float frame_id;
    std::tie(success, frame_id, std::ignore, std::ignore) = recv((float)GET_FRAME_ID);
    return std::make_tuple(success, (int)frame_id);
  }

  void setFilterGain(float gain)
  {
    send((float)SET_FILTER_GAIN, 0.0, gain);
  }

  std::tuple<bool, float> getFilterGain()
  {
    bool success; float gain;
    std::tie(success, gain, std::ignore, std::ignore) = recv((float)GET_FILTER_GAIN);
    return std::make_tuple(success, gain);
  }

  std::tuple<bool, float, float, float> readLinearAcc()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = recv((float)READ_LIN_ACC);
    return std::make_tuple(success, x, y, z);
  }

  std::tuple<bool, float, float, float> readGyro()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = recv((float)READ_GYRO);
    return std::make_tuple(success, x, y, z);
  }

  std::tuple<bool, float, float, float> readRPY()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = recv((float)READ_RPY);
    return std::make_tuple(success, x, y, z);
  }

  std::tuple<bool, float, float, float> readAccVariance()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = recv((float)READ_ACC_VAR);
    return std::make_tuple(success, x, y, z);
  }

  std::tuple<bool, float, float, float> readGyroVariance()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = recv((float)READ_GYRO_VAR);
    return std::make_tuple(success, x, y, z);
  }

  std::tuple<bool, float, float, float> readRPYVariance()
  {
    bool success; float x, y, z;
    std::tie(success, x, y, z) = recv((float)READ_RPY_VAR);
    return std::make_tuple(success, x, y, z);
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;

  void send(float cmd, float arg1=0.0, float arg2=0.0, float arg3=0.0)
  {
    std::ostringstream ss;
    ss << cmd << " " << round_to_dp(arg1,6) << " " << round_to_dp(arg2,6) << " " << round_to_dp(arg3,6) << "\r";

    serial_conn_.Write(ss.str());
  }

  std::tuple<bool, float, float, float> recv(float cmd, float arg1=0.0)
  {
    bool success;
    float data1, data2, data3;
    try {
      // Send request
      send(cmd, arg1);

      // Read response line (terminated by '\n')
      std::string line;
      serial_conn_.ReadLine(line, '\n');

      // Convert using strtof (robust & locale-safe)
      char* ptr = line.data();
      char* end;

      data1 = strtof(ptr, &end);
      if (ptr == end) {
        success = false;
        data1 = 0.0f;
        data2 = 0.0f;
        data3 = 0.0f;
        return std::make_tuple(success, data1, data2, data3);
      }

      data2 = strtof(end, &end);
      if (ptr == end){
        success = false;
        data1 = 0.0f;
        data2 = 0.0f;
        data3 = 0.0f;
        return std::make_tuple(success, data1, data2, data3);
      }

      data3 = strtof(end, &end);
      if (ptr == end){
        success = false;
        data1 = 0.0f;
        data2 = 0.0f;
        data3 = 0.0f;
        return std::make_tuple(success, data1, data2, data3);
      }

      success = true;
      return std::make_tuple(success, round_to_dp(data1,6), round_to_dp(data2,6), round_to_dp(data3,6));
    }
    catch (...) {
      success = false;
      data1 = 0.0f;
      data2 = 0.0f;
      data3 = 0.0f;
      // serial_conn_.FlushIOBuffers();
      return std::make_tuple(success, data1, data2, data3);
    }
  }


};

#endif
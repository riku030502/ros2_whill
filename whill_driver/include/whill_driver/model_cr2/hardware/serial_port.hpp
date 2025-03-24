// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    serial_port.hpp
 * @brief   Definitions of serial port driver
 */
#ifndef WHILL_DRIVER_MODEL_CR2_HARDWARE_SERIAL_PORT_H_
#define WHILL_DRIVER_MODEL_CR2_HARDWARE_SERIAL_PORT_H_

#include <sys/epoll.h>
#include <termios.h>

#include <string>

namespace whill_driver
{
namespace model_cr2
{
namespace hardware
{

constexpr int kMaxEvents = 10;

class SerialPort
{
public:
  int Open(std::string port);
  void Close();
  void Reflesh();
  int Send(uint8_t buf[], int len);
  int Receive(uint8_t buf[], int len);

private:
  int fd_;
  struct termios old_tio_;
  struct epoll_event ev_;
  struct epoll_event events_[kMaxEvents];
  int epollfd_;

  bool CanRead();
};

}  // namespace hardware
}  // namespace model_cr2
}  // namespace whill_driver

#endif  // WHILL_DRIVER_MODEL_CR2_HARDWARE_SERIAL_PORT_H_

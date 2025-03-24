// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    serial_port.cpp
 * @brief   Functions of serial port driver
 */
#include "whill_driver/model_cr2/hardware/serial_port.hpp"

#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace whill_driver
{
namespace model_cr2
{
namespace hardware
{

int SerialPort::Open(std::string port)
{
  // setting for UART
  if (!(fd_ = open(port.c_str(), O_RDWR))) {
    fprintf(stderr, "[SerialPort] open() error.\n");
    return -1;
  }
  ioctl(fd_, TCGETS, &old_tio_);
  struct termios tio = old_tio_;
  tio.c_cflag = (B38400 | CS8 | CLOCAL | CREAD | CSTOPB);
  tio.c_iflag = (IGNPAR);
  tio.c_oflag = 0;
  tio.c_lflag = 0;  // non canonical mode
  ioctl(fd_, TCSETS, &tio);

  // setting for epoll
  if ((epollfd_ = epoll_create1(0)) < 0) {
    fprintf(stderr, "[SerialPort] epoll_create1() error.\n");
    return -1;
  }
  ev_.events = EPOLLIN;
  ev_.data.fd = fd_;
  if (epoll_ctl(epollfd_, EPOLL_CTL_ADD, fd_, &ev_) == -1) {
    fprintf(stderr, "[SerialPort] epoll_ctl() error.\n");
    return -1;
  }
  return 1;
}

void SerialPort::Close()
{
  ioctl(fd_, TCSETS, &old_tio_);
  close(fd_);
  fprintf(stderr, "SerialPort::Close() called.\n");
}

void SerialPort::Reflesh()
{
  tcflush(fd_, TCIFLUSH);
}

int SerialPort::Send(uint8_t buf[], int len)
{
  if (write(fd_, buf, len) != len) {
    fprintf(stderr, "[SerialPort] write() error. command_id:[%02x]\n", buf[2]);
    return -1;
  }
  return 1;
}

int SerialPort::Receive(uint8_t buf[], int len)
{
  if (!CanRead()) {return -1;}

  return read(fd_, buf, len);
}

bool SerialPort::CanRead()
{
  int nfds = epoll_wait(epollfd_, events_, kMaxEvents, 0);
  if (nfds < 1) {
    return false;  // no data
  }

  for (int i = 0; i < nfds; ++i) {
    if (events_[i].data.fd == fd_) {
      return true;
    }
  }
  return false;
}

}  // namespace hardware
}  // namespace model_cr2
}  // namespace whill_driver

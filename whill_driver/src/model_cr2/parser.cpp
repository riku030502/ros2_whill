// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    parser.cpp
 * @brief   Functions of WHILL State Data Parsing
 */
#include "whill_driver/model_cr2/parser.hpp"

#include <memory>
#include <string.h>

namespace whill_driver
{
namespace model_cr2
{

Parser::Parser()
{
  Clear();
}

Parser::~Parser()
{
}

void Parser::Clear()
{
  memset(buf_, 0, sizeof(buf_));
  state_ = ParseState::kWaitProtocolSign;
  index_ = 0;
  data_length_ = 0;
}

uint8_t Parser::Checksum(uint8_t buf[], int len)
{
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < len; ++i) {
    checksum ^= buf[i];
  }
  return checksum;
}

int Parser::Parse(uint8_t buf[], int target_len, uint8_t payload[])
{
  int payload_len = 0;

  for (int i = 0; i < target_len; ++i) {
    switch (state_) {
      case ParseState::kWaitProtocolSign:
        if (buf[i] == kProtocolSign) {
          buf_[index_++] = buf[i];
          state_ = ParseState::kWaitDataLength;
        }
        break;
      case ParseState::kWaitDataLength:
        if ((buf[i] > 0) && (buf[i] <= kDatasetMaxSize - 2)) {
          buf_[index_++] = buf[i];
          data_length_ = buf[i];
          state_ = ParseState::kWaitPayload;
        } else {
          Clear();
        }
        break;
      case ParseState::kWaitPayload:
        if (index_ < data_length_) {
          buf_[index_++] = buf[i];
        } else if (index_ >= data_length_) {
          // last payload
          buf_[index_++] = buf[i];
          state_ = ParseState::kWaitChecksum;
        } else {
          Clear();
        }
        break;
      case ParseState::kWaitChecksum:
        if (buf[i] == Checksum(buf_, index_)) {
          payload_len = data_length_ - 1;
          memcpy(payload, &buf_[2], payload_len);
        }
        Clear();
        break;

      default:
        Clear();
        break;
    }
  }
  return payload_len;
}

}  // namespace model_cr2
}  // namespace whill_driver

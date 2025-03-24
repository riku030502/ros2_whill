// Copyright (c) 2024 WHILL, Inc.
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

/**
 * @file    parser.hpp
 * @brief   Definitions of WHILL State Data Parsing
 */
#ifndef WHILL_DRIVER_MODEL_CR2_PARSER_H_
#define WHILL_DRIVER_MODEL_CR2_PARSER_H_

#include <cstdint>

namespace whill_driver
{
namespace model_cr2
{

/**
 * This state indicates the next byte of data to be received.
 */
enum class ParseState
{
  kWaitProtocolSign = 0,
  kWaitDataLength,
  kWaitPayload,
  kWaitChecksum,
};

/**
 * Maximum size of entire WHILL state dataset packet.
 */
constexpr uint8_t kDatasetMaxSize = 64;

/**
 * The value of protocol sign of packets.
 */
constexpr uint8_t kProtocolSign = 0xAF;

/**
 * This class analyzes packets and extracts the payload.
 * It does not have any send and receive functionality.
 */
class Parser
{
public:
  Parser();
  ~Parser();

  /**
   * The Parse function processes a buffer to extract a payload based on a specific protocol sign, data
   * length, and checksum.
   *
   * @param buf The `buf` parameter is an array of `uint8_t` type, which is used to store the data
   * that needs to be parsed.
   * @param target_len The `target_len` parameter in the `Parse` function represents the length of the
   * `buf` array that is being parsed. It is used to iterate over the elements of the `buf` array and
   * determine the parsing logic based on the current state of the parser.
   * @param payload The `payload` parameter in the `Parse` function is an output parameter where the
   * parsed data will be stored. It is an array of `uint8_t` type that will hold the extracted
   * payload data after parsing the input buffer `buf`.
   *
   * @return The `Parse` function returns an integer value representing the length of the payload that
   * was successfully parsed from the input buffer.
   */
  int Parse(uint8_t target_buf[], int target_len, uint8_t payload[]);

  /**
   * The Checksum function calculates the XOR checksum of an array of bytes.
   *
   * @param buf An array of uint8_t values representing the data for which the checksum needs to be
   * calculated.
   * @param len The `len` parameter in the `Checksum` function represents the length of the `buf` array,
   * which is the number of elements in the array that need to be included in the checksum calculation.
   *
   * @return The function `Checksum` is returning a `uint8_t` value, which is the calculated checksum of
   * the input buffer `buf` with the specified length `len`.
   */
  uint8_t Checksum(uint8_t buf[], int len);

private:
  uint8_t buf_[kDatasetMaxSize];
  ParseState state_;
  int index_;
  int data_length_;

  void Clear();
};

}  // namespace model_cr2
}  // namespace whill_driver

#endif  // WHILL_DRIVER_MODEL_CR2_PARSER_H_

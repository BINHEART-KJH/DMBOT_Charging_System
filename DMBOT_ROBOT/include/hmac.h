#ifndef HMAC_H
#define HMAC_H

#include <stdint.h>
#include <stddef.h>

class HMAC {
public:
  HMAC();
  void init(const uint8_t* key, size_t keyLength);
  void update(const uint8_t* data, size_t length);
  void finalize(uint8_t* hmacResult, size_t resultLength);

private:
  static const size_t blockSize = 64;
  uint8_t o_key_pad[blockSize];
  uint8_t i_key_pad[blockSize];
};

#endif
#include "hmac.h"
#include "sha256.h"
#include <string.h>

HMAC::HMAC() {}

void HMAC::init(const uint8_t* key, size_t keyLength) {
  uint8_t keyBlock[blockSize];
  memset(keyBlock, 0, blockSize);

  if (keyLength > blockSize) {
    sha256(key, keyLength, keyBlock);
  } else {
    memcpy(keyBlock, key, keyLength);
  }

  for (size_t i = 0; i < blockSize; i++) {
    o_key_pad[i] = keyBlock[i] ^ 0x5c;
    i_key_pad[i] = keyBlock[i] ^ 0x36;
  }
}

void HMAC::update(const uint8_t* data, size_t length) {
  uint8_t innerData[blockSize + length];
  memcpy(innerData, i_key_pad, blockSize);
  memcpy(innerData + blockSize, data, length);

  uint8_t innerHash[32];
  sha256(innerData, blockSize + length, innerHash);

  uint8_t outerData[blockSize + 32];
  memcpy(outerData, o_key_pad, blockSize);
  memcpy(outerData + blockSize, innerHash, 32);

  sha256(outerData, blockSize + 32, innerHash);

  memcpy(i_key_pad, innerHash, 32); // store result temporarily
}

void HMAC::finalize(uint8_t* hmacResult, size_t resultLength) {
  memcpy(hmacResult, i_key_pad, resultLength);
}

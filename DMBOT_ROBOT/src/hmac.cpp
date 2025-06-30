#include "hmac.h"
#include "sha256.h"
#include <string.h>
#include <stdio.h>

void generateHMAC_SHA256(const char* message, const char* key, char* hexOut) {
  const size_t blockSize = 64;
  uint8_t keyBlock[blockSize];
  memset(keyBlock, 0, blockSize);

  size_t keyLen = strlen(key);
  if (keyLen > blockSize) {
    SHA256 sha;
    sha.update((const uint8_t*)key, keyLen);
    sha.finalize(keyBlock);
  } else {
    memcpy(keyBlock, key, keyLen);
  }

  uint8_t o_key_pad[blockSize];
  uint8_t i_key_pad[blockSize];

  for (size_t i = 0; i < blockSize; i++) {
    o_key_pad[i] = keyBlock[i] ^ 0x5c;
    i_key_pad[i] = keyBlock[i] ^ 0x36;
  }

  // Inner hash
  uint8_t innerHash[32];
  SHA256 shaInner;
  shaInner.update(i_key_pad, blockSize);
  shaInner.update((const uint8_t*)message, strlen(message));
  shaInner.finalize(innerHash);

  // Outer hash
  SHA256 shaOuter;
  shaOuter.update(o_key_pad, blockSize);
  shaOuter.update(innerHash, 32);
  uint8_t hmacResult[32];
  shaOuter.finalize(hmacResult);

  // HEX 출력
  for (int i = 0; i < 32; i++) {
    sprintf(&hexOut[i * 2], "%02x", hmacResult[i]);
  }
  hexOut[64] = '\0';
}
#ifndef SHA256_H
#define SHA256_H

#include <stdint.h>
#include <stddef.h>

class SHA256 {
public:
    SHA256();
    void reset();
    void update(const uint8_t* data, size_t len);
    void finalize(uint8_t hash[32]);  // 32 bytes = 256 bits

private:
    void transform(const uint8_t* chunk);

    uint8_t buffer[64];     // 512-bit chunk buffer
    uint32_t state[8];      // hash state (8 words)
    uint64_t bitlen;        // total message length in bits
    size_t bufferLen;       // current length of buffer
};

#endif
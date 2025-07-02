#include "sha256.h"
#include <Arduino.h>
#include <mbedtls/sha256.h>

void sha256(const uint8_t* data, size_t len, uint8_t* outHash) {
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts_ret(&ctx, 0); // 0 = SHA-256, 1 = SHA-224
  mbedtls_sha256_update_ret(&ctx, data, len);
  mbedtls_sha256_finish_ret(&ctx, outHash);
  mbedtls_sha256_free(&ctx);
}

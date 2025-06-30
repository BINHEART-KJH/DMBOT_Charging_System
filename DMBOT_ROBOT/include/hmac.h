#ifndef HMAC_H
#define HMAC_H

#include <stdint.h>
#include <stddef.h>

void generateHMAC_SHA256(const char* message, const char* key, char* hexOut);

#endif

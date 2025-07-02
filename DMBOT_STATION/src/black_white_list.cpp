#include <Arduino.h>
#include <string.h>
#include "black_white_list.h"

#define MAX_BLACKLIST 50
#define MAX_WHITELIST 20

char blacklist[MAX_BLACKLIST][18];  // "XX:XX:XX:XX:XX:XX" + null
int blacklistCount = 0;

char whitelist[MAX_WHITELIST][18];
int whitelistCount = 0;

void initLists() {
  blacklistCount = 0;
  whitelistCount = 0;
}

void addToBlacklist(const char* mac) {
  if (blacklistCount < MAX_BLACKLIST) {
    char upperMac[18];
    strncpy(upperMac, mac, 18);

    for (int i = 0; i < 17; i++) {
      upperMac[i] = toupper(upperMac[i]);
    }

    strncpy(blacklist[blacklistCount], upperMac, 18);
    blacklistCount++;
  }
}

void addToWhitelist(const char* mac) {
  if (whitelistCount < MAX_WHITELIST) {
    char upperMac[18];
    strncpy(upperMac, mac, 18);

    for (int i = 0; i < 17; i++) {
      upperMac[i] = toupper(upperMac[i]);
    }

    strncpy(whitelist[whitelistCount], upperMac, 18);
    whitelistCount++;
  }
}

bool isInBlacklist(const char* mac) {
  char upperMac[18];
  strncpy(upperMac, mac, 18);
  for (int i = 0; i < 17; i++) {
    upperMac[i] = toupper(upperMac[i]);
  }

  for (int i = 0; i < blacklistCount; ++i) {
    if (strncmp(upperMac, blacklist[i], 17) == 0) return true;
  }
  return false;
}

bool isWhitelisted(const char* mac) {
  char upperMac[18];
  strncpy(upperMac, mac, 18);
  for (int i = 0; i < 17; i++) {
    upperMac[i] = toupper(upperMac[i]);
  }

  for (int i = 0; i < whitelistCount; ++i) {
    if (strncmp(upperMac, whitelist[i], 17) == 0) return true;
  }
  return false;
}
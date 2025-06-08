#include "blacklist.h"
#include <Arduino.h>
#include <string.h>

#define MAX_BLACKLIST 50
#define BLOCK_DURATION_MS 300000  // 5분 블록

struct BlacklistEntry {
  char mac[18];
  unsigned long timestamp;
};

BlacklistEntry blacklist[MAX_BLACKLIST];
int blacklistIndex = 0;

bool isBlacklisted(const char* mac) {
  unsigned long now = millis();
  for (int i = 0; i < MAX_BLACKLIST; ++i) {
    if (strcmp(blacklist[i].mac, mac) == 0 &&
        (now - blacklist[i].timestamp) < BLOCK_DURATION_MS) {
      return true;
    }
  }
  return false;
}

void addToBlacklist(const char* mac) {
  strncpy(blacklist[blacklistIndex].mac, mac, sizeof(blacklist[blacklistIndex].mac));
  blacklist[blacklistIndex].mac[sizeof(blacklist[blacklistIndex].mac) - 1] = '\0';
  blacklist[blacklistIndex].timestamp = millis();
  blacklistIndex = (blacklistIndex + 1) % MAX_BLACKLIST;
}
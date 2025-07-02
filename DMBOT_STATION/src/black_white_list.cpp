#include <Arduino.h>
#include <string.h>
#include "black_white_list.h"

#define MAX_BLACKLIST 50
#define MAX_WHITELIST 20

char blacklist[MAX_BLACKLIST][18];  // "XX:XX:XX:XX:XX:XX" + null
int blacklistCount = 0;
int blacklistIndex = 0; // 링버퍼용 인덱스

char whitelist[MAX_WHITELIST][18];
int whitelistCount = 0;

void initLists() {
  blacklistCount = 0;
  whitelistCount = 0;
  blacklistIndex = 0;
}

// 블랙리스트 중복 확인
bool isBlacklisted(const char* mac) {
  for (int i = 0; i < blacklistCount; ++i) {
    if (strncmp(mac, blacklist[i], 17) == 0) return true;
  }
  return false;
}

void addToBlacklist(const char* mac) {
  char upperMac[18];
  strncpy(upperMac, mac, 18);

  for (int i = 0; i < 17; i++) {
    upperMac[i] = toupper(upperMac[i]);
  }
  upperMac[17] = '\0';

  if (isBlacklisted(upperMac)) return;  // 중복이면 추가 안 함

  // 링버퍼 구조로 추가
  strncpy(blacklist[blacklistIndex], upperMac, 18);
  blacklist[blacklistIndex][17] = '\0'; // 명시적 null-terminate
  blacklistIndex = (blacklistIndex + 1) % MAX_BLACKLIST;

  if (blacklistCount < MAX_BLACKLIST) {
    blacklistCount++;
  }

  Serial.print("🚫 블랙리스트 등록됨: ");
  Serial.println(upperMac);
}

void addToWhitelist(const char* mac) {
  if (whitelistCount < MAX_WHITELIST) {
    char upperMac[18];
    strncpy(upperMac, mac, 18);

    for (int i = 0; i < 17; i++) {
      upperMac[i] = toupper(upperMac[i]);
    }
    upperMac[17] = '\0';

    strncpy(whitelist[whitelistCount], upperMac, 18);
    whitelist[whitelistCount][17] = '\0';
    whitelistCount++;
  }
}

bool isInBlacklist(const char* mac) {
  char upperMac[18];
  strncpy(upperMac, mac, 18);
  for (int i = 0; i < 17; i++) {
    upperMac[i] = toupper(upperMac[i]);
  }
  upperMac[17] = '\0';

  return isBlacklisted(upperMac);
}

bool isWhitelisted(const char* mac) {
  char upperMac[18];
  strncpy(upperMac, mac, 18);
  for (int i = 0; i < 17; i++) {
    upperMac[i] = toupper(upperMac[i]);
  }
  upperMac[17] = '\0';

  for (int i = 0; i < whitelistCount; ++i) {
    if (strncmp(upperMac, whitelist[i], 17) == 0) return true;
  }
  return false;
}

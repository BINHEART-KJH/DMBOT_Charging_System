#pragma once

bool isBlacklisted(const char* mac);
void addToBlacklist(const char* mac);
void cleanupBlacklist();  // TTL 만료 항목 제거용 (optional)
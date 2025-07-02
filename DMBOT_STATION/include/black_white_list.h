#ifndef BLACK_WHITE_LIST_H
#define BLACK_WHITE_LIST_H

void initLists();
bool isInBlacklist(const char* mac);  // ✅ 이름 통일
bool isWhitelisted(const char* mac);
void addToBlacklist(const char* mac);
void addToWhitelist(const char* mac);

#endif

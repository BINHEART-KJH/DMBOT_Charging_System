#ifndef BLACK_WHITE_LIST_H
#define BLACK_WHITE_LIST_H

void initLists();
void addToBlacklist(const char* mac);
void addToWhitelist(const char* mac);
bool isInBlacklist(const char* mac);
bool isWhitelisted(const char* mac);

#endif
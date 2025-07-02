// robot_auth.h
#ifndef ROBOT_AUTH_H
#define ROBOT_AUTH_H

void generateHMAC_SHA256(const char* message, const char* key, char* hexOut);

#endif
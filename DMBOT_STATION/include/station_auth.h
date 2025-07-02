#ifndef STATION_AUTH_H
#define STATION_AUTH_H

#include <ArduinoBLE.h>

void setupAuthService();
void resetAuth();
bool processAuthToken();
bool isAuthenticated();

#endif

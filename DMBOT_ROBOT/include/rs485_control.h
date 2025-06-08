#pragma once

void setupRS485();
void processRS485Command();
bool isBLEScanRequested();
bool isBLEStopRequested();
void clearBLECommandFlags();
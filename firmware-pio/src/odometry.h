#pragma once
#include <Arduino.h>

void setupOdometry();
void updateOdometry();

void setCommandSigns(int8_t leftSign, int8_t rightSign);

float getOdomX();
float getOdomY();
float getOdomTheta();

unsigned long getLeftPulses();
unsigned long getRightPulses();

#ifndef __UTILS_H__
#define __UTILS_H__

#include <Arduino.h>

float DecodeFloat(uint8_t *data, int offset);
void ConvertFloatToBuffer(float value, uint8_t *v_buffer, int offset);


#endif

#include "Utils.h"

union _floatToChar {
  float f;
  uint8_t c[4];
} floatToChar_pro;

void ConvertFloatToBuffer(float value, uint8_t *v_buffer, int offset) {
  floatToChar_pro.f = value;
  v_buffer[offset++] = floatToChar_pro.c[0];
  v_buffer[offset++] = floatToChar_pro.c[1];
  v_buffer[offset++] = floatToChar_pro.c[2];
  v_buffer[offset++] = floatToChar_pro.c[3];
}

float DecodeFloat(uint8_t *data, int offset) {
  floatToChar_pro.c[0] = data[offset];
  floatToChar_pro.c[1] = data[offset+1];
  floatToChar_pro.c[2] = data[offset+2];
  floatToChar_pro.c[3] = data[offset+3];
  return floatToChar_pro.f;
}

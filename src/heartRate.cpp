#include "heartRate.h"

#define AVERAGING_SIZE 8

int32_t IR_AVG_Signal_Current = 0;
int32_t IR_AVG_Signal_Previous = 0;
int32_t IR_AVG_Derivative_Current = 0;
int32_t IR_AVG_Derivative_Previous = 0;
int32_t averageBuffer[AVERAGING_SIZE];
uint8_t averageBufferIndex = 0;
uint8_t averageValidSamples = 0;

void resetDetectBeat() {
  averageBufferIndex = 0;
  averageValidSamples = 0;
  for (uint8_t i = 0; i < AVERAGING_SIZE; i++) {
    averageBuffer[i] = 0;
  }
  IR_AVG_Signal_Current = 0;
  IR_AVG_Signal_Previous = 0;
  IR_AVG_Derivative_Current = 0;
  IR_AVG_Derivative_Previous = 0;
}

int32_t rollingAverage(int32_t sample) {
  averageBuffer[averageBufferIndex++] = sample;
  if (averageBufferIndex > averageValidSamples) {
    averageValidSamples = averageBufferIndex;
  }

  int32_t samplesAcc = 0;
  for (uint8_t i = 0; i < averageValidSamples; i++){
    samplesAcc += averageBuffer[i];
  }

  averageBufferIndex %= AVERAGING_SIZE;
  return samplesAcc / averageValidSamples;
}


bool detectBeat(int32_t sample)
{
  IR_AVG_Signal_Previous = IR_AVG_Signal_Current;
  IR_AVG_Derivative_Previous = IR_AVG_Derivative_Current;

  IR_AVG_Signal_Current = rollingAverage(sample);
  IR_AVG_Derivative_Current = IR_AVG_Signal_Previous - IR_AVG_Signal_Current;

  bool isFallingEdge = (IR_AVG_Derivative_Current < 0) && (IR_AVG_Derivative_Previous >= 0);
  bool isBeatDetected = isFallingEdge && (averageValidSamples == AVERAGING_SIZE);

  return isBeatDetected;
}

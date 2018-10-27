#include <Arduino.h>
#include <MAX30105.h>
#include <heartRate.h>

#define debug Serial

#define MAX30105_ADDRESS          0x57 //7-bit I2C Address
MAX30105 particleSensor;

#define HRT_SENSOR_UNCOVERED  0x00
#define HRT_SENSOR_COVERED    0x01
#define HRT_SENSOR_BEAT_RATE  0x02
#define HRT_SENSOR_MAGIC      0x03 //don't try me... really :)

#define RATE_AVERAGING_SIZE 5 //Increase this for more averaging. 4 is good.
#define IR_CUT_OFF 65535


void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);


  delay(1000);
  Serial.begin(115200);
  Serial.println("-- start HRT sensor --");
  Serial.flush();

  // Initialize sensor
  //  if (particleSensor.begin(Wire, I2C_SPEED_FAST, MAX30105_ADDRESS) == false)
  if (particleSensor.begin() == false)
  {
    Serial.println("MAX30101 not found.");
    while (1);
  }

  particleSensor.setup();

  particleSensor.setPulseAmplitudeRed(0x10); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  digitalWrite(LED_BUILTIN, HIGH);
}


void outputData(uint8_t i2cMessageType, long irValue, uint8_t beatRateAvg) {
  // debug.print("CMD=");
  // debug.print(i2cMessageType);
  Serial.write(i2cMessageType);
  if(i2cMessageType == HRT_SENSOR_COVERED) {
    // debug.print(", IR=");
    // debug.print((uint8_t) irValue);
    Serial.write((uint8_t) irValue);
  } else if(i2cMessageType == HRT_SENSOR_BEAT_RATE) {
    // debug.print(", AVG=");
    // debug.print((uint8_t) beatRateAvg);
    Serial.write((uint8_t) beatRateAvg);
  }
  // Serial.println();
  Serial.flush();
}


#define BEAT_LED_LENGTH_MS 100
unsigned long lastBeatTime = 0;
unsigned long deltaLastBeat = 0;
void builtinLightFeedback(uint8_t i2cMessageType, unsigned long deltaLastBeat) {
  if (i2cMessageType == HRT_SENSOR_BEAT_RATE) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    if (deltaLastBeat < BEAT_LED_LENGTH_MS ) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}


byte heartRates[RATE_AVERAGING_SIZE];
byte rateSpot = 0;

float beatsPerMinute;

uint8_t i2cMessageType = HRT_SENSOR_UNCOVERED;
long irValue = 0;
uint8_t beatRateAvg = 0;


void loop() {

  irValue = particleSensor.getIR();

  deltaLastBeat = millis() - lastBeatTime;

  if (irValue < IR_CUT_OFF) {
    i2cMessageType = HRT_SENSOR_UNCOVERED;
    beatRateAvg = 0;
  } else {

    if (checkForBeat(irValue) == true) {
      lastBeatTime = millis();

      beatsPerMinute = 60 / (deltaLastBeat / 1000.0);

      if (beatsPerMinute < 0x80 && beatsPerMinute > 40) {
        heartRates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_AVERAGING_SIZE; //Wrap variable

        //Take average of readings
        beatRateAvg = 0;
        for (byte x = 0 ; x < RATE_AVERAGING_SIZE ; x++)
          beatRateAvg += heartRates[x];

        beatRateAvg /= RATE_AVERAGING_SIZE;
      }
      i2cMessageType = HRT_SENSOR_BEAT_RATE;

    } else {
      i2cMessageType = HRT_SENSOR_COVERED;
    }

  }

  outputData(i2cMessageType, irValue, beatRateAvg);

  delay(40);

  builtinLightFeedback(i2cMessageType, deltaLastBeat);

}

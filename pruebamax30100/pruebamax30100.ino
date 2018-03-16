/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>
#include "MAX30100.h"
#include "MAX30100_PulseOximeter.h"
#include "freertos/event_groups.h"

#define REPORTING_PERIOD_MS     1000

/* define event bits */
#define TASK_1_BIT        ( 1 << 0 ) //1

// PulseOximeter is the higher level interface to the sensor
// it offers:
//  * beat detection reporting
//  * heart rate calculation
//  * SpO2 (oxidation level) calculation
MAX30100 sensor;
PulseOximeter ox;

uint32_t tsLastReport = 0;
int thresholdt = 50;
int thresholdu = 60;

/* create event group */
EventGroupHandle_t eg;

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
    Serial.println("Beat!");
}

void setup()
{
    Serial.begin(115200);
    Serial.print("Initializing MAX30100..");

    if (!sensor.begin()) {
        Serial.print("FAILED: ");

        uint8_t partId = sensor.getPartId();
        if (partId == 0xff) {
            Serial.println("I2C error");
        } else {
            Serial.print("wrong part ID 0x");
            Serial.print(partId, HEX);
            Serial.print(" (expected: 0x");
            Serial.println(EXPECTED_PART_ID, HEX);
        }
        // Stop here
        for(;;);
    } else {
        Serial.println("Success");
    }

    Serial.print("Enabling HR/SPO2 mode..");
    sensor.setMode(MAX30100_MODE_SPO2_HR);
    Serial.println("done.");

    Serial.print("Configuring LEDs biases to 50mA..");
    sensor.setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_50MA);
    Serial.println("done.");

    delay(1000);

    Serial.print("Lowering the current to 7.6mA..");
    sensor.setLedsCurrent(MAX30100_LED_CURR_7_6MA, MAX30100_LED_CURR_7_6MA);
    Serial.println("done.");

    uint32_t tsTempSampStart = millis();
    Serial.print("Sampling die temperature..");
    sensor.startTemperatureSampling();
    while(!sensor.isTemperatureReady()) {
        if (millis() - tsTempSampStart > 1000) {
            Serial.println("ERROR: timeout");
            // Stop here
            for(;;);
        }
    }

    float temperature = sensor.retrieveTemperature();
    Serial.print("done, temp=");
    Serial.print(temperature);
    Serial.println("C");

    delay(1000);

    Serial.print("Shutting down..");
    sensor.shutdown();
    Serial.println("done.");

    eg = xEventGroupCreate();
        // The default current for the IR LED is 50mA and it could be changed
    //   by uncommenting the following line. Check MAX30100_Registers.h for all the
    //   available options.
    //pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    //pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);

    // Register a callback for the beat detection
    ox.setOnBeatDetectedCallback(onBeatDetected);
    printf("\n ESP32 Touch Interrupt \n");
    touchAttachInterrupt(T0, gotTouch, threshold);
}

          
void loop()
{
    // Make sure to call update as fast as possible
    /* wait forever until event bit of task 1 is set */
    EventBits_t xbit = xEventGroupWaitBits(eg, TASK_1_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    Serial.print("Loop has even bit: ");
    Serial.println(xbit);
    ox.update();

    // Asynchronously dump heart rate and oxidation levels to the serial
    // For both, a value of 0 means "invalid"
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        Serial.print("Heart rate:");
        Serial.print(ox.getHeartRate());
        Serial.print("bpm / SpO2:");
        Serial.print(ox.getSpO2());
        Serial.println("%");

        tsLastReport = millis();
    }
    if (touchRead(T0) > thresholdu) {
      Serial.println(touchRead(T0));  // get value using T0
      uxBits = xEventGroupClearBits(eg,TASK_1_BIT);
      delay(500);
      touchAttachInterrupt(T0, gotTouch, delay(500));
      delay(500);
     }
    
}

void gotTouch(){
  Serial.println("Touched\n");
  BaseType_t xHigherPriorityTaskWoken;
  Serial.print("Resuming normal operation..");
  sensor.resume();
  delay(500);
  Serial.println("done.");
  sensor.resetFifo();
  printf("\n ESP32 Event TASK_1_BIT \n");
  xEventGroupSetBitsFromISR(eg,TASK_1_BIT, &xHigherPriorityTaskWoken);
  printf("\n ESP32 Touch Interrupt\n");
  touchAttachInterrupt(T0, gotTouch, 10);
}


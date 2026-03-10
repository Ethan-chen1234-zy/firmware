/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"
#include "nrf.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "SPI.h"
#include "configuration.h"


const uint32_t g_ADigitalPinMap[] = {
    // P0
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,

    // P1
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47};

void initVariant()
{
    // LED1 & LED2
    pinMode(PIN_LED1, OUTPUT);
    ledOff(PIN_LED1);

    pinMode(PIN_LED2, OUTPUT);
    ledOff(PIN_LED2);

    // 3V3 Power Rail
    pinMode(PIN_3V3_EN, OUTPUT);
    digitalWrite(PIN_3V3_EN, HIGH);
}

void earlyInitVariant() {
    // Prevent spi0 from being used.
    pinMode(INTERNAL_SX126X_POWER_EN, OUTPUT);
    digitalWrite(INTERNAL_SX126X_POWER_EN, LOW);
    pinMode(INTERNAL_SX126X_RESET, OUTPUT);
    digitalWrite(INTERNAL_SX126X_RESET, HIGH);
    pinMode(INTERNAL_SX126X_CS, OUTPUT);
    digitalWrite(INTERNAL_SX126X_CS, HIGH);
    pinMode(INTERNAL_SX126X_MISO, INPUT_PULLDOWN);
    pinMode(INTERNAL_SX126X_MOSI, INPUT_PULLDOWN);
    pinMode(INTERNAL_SX126X_SCK, INPUT_PULLDOWN);

    delay(10);
    
}


void lateInitVariant() {
    LOG_INFO("Initializing and sleeping internal SPI0 SX1262 for RAK13302");
    
    // 1. Configure internal SX1262_CS pins
    pinMode(INTERNAL_SX126X_CS, OUTPUT);
    digitalWrite(INTERNAL_SX126X_CS, HIGH);
    
    // 2. Reset internal SX1262
    pinMode(INTERNAL_SX126X_RESET, OUTPUT);
    digitalWrite(INTERNAL_SX126X_RESET, LOW);
    delay(100);
    digitalWrite(INTERNAL_SX126X_RESET, HIGH);
    delay(200);
    
    // 3. Initialize SPI
    SPI.setPins(INTERNAL_SX126X_MISO, INTERNAL_SX126X_SCK, INTERNAL_SX126X_MOSI);
    SPI.begin();
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

    // 4. Ensure entering STDBY_RC mode
    digitalWrite(INTERNAL_SX126X_CS, LOW);
    SPI.transfer(0x80); // STDBY command
    SPI.transfer(0x00); // STDBY_RC mode
    digitalWrite(INTERNAL_SX126X_CS, HIGH);
    delay(10);

    // 5. Clear all IRQ status
    digitalWrite(INTERNAL_SX126X_CS, LOW);
    SPI.transfer(0x02); // ClearIrqStatus
    SPI.transfer(0xFF);  //IrqMask low byte
    SPI.transfer(0xFF);  //IrqMask high byte
    digitalWrite(INTERNAL_SX126X_CS, HIGH);
    delay(10);
    
    // 6. Disable all IRQs
    digitalWrite(INTERNAL_SX126X_CS, LOW);
    SPI.transfer(0x08); // SetDioIrqParams
    for(int i=0; i<8; i++) SPI.transfer(0x00); 
    digitalWrite(INTERNAL_SX126X_CS, HIGH);
    delay(10);

    // 7. SLEEP command
    digitalWrite(INTERNAL_SX126X_CS, LOW);
    SPI.transfer(0x84); // SLEEP command
    SPI.transfer(0x00); // Cold start, do not retain configuration
    digitalWrite(INTERNAL_SX126X_CS, HIGH);
    delay(500);
    SPI.endTransaction();
    SPI.end();

    // 8. Final GPIO configuration for low power consumption
    pinMode(INTERNAL_SX126X_MISO, INPUT);
    pinMode(INTERNAL_SX126X_MOSI, INPUT);
    pinMode(INTERNAL_SX126X_SCK, INPUT);
    pinMode(INTERNAL_SX126X_BUSY, INPUT);
    pinMode(INTERNAL_SX126X_DIO1, INPUT);
    pinMode(INTERNAL_SX126X_POWER_EN, OUTPUT);
    digitalWrite(INTERNAL_SX126X_POWER_EN, LOW);

    pinMode(39, OUTPUT); // P1.07, 可能的 TXEN
    digitalWrite(39, LOW);
    pinMode(37, OUTPUT); // P1.05, 可能的 RXEN 或与POWER_EN冲突
    digitalWrite(37, LOW);
    LOG_INFO("Post-sleep BUSY pin state: %d", digitalRead(INTERNAL_SX126X_BUSY));
    LOG_INFO("Internal SX1262 put to sleep");

}


/* DHT11 Temperature and Humidity Sensor
 * based on sample code:
 * http://www.dfrobot.com/wiki/index.php?title=DHT11_Temperature_and_Humidity_Sensor_(SKU:_DFR0067)
 * <cccp at c-base dot org>
 */

#include <sysinit.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "basic/basic.h"
#include "basic/config.h"

#include "lcd/render.h"
#include "lcd/print.h"

#define PIN_DHT11    RB_SPI_SS2
#define PIN_DHT11_IO IOCON_PIO2_8

#define OUT(x)  do {gpioSetValue(PIN_DHT11, x);} while (0)

struct sensordata {
    uint8_t humidity;
    uint8_t temperature;
};

void init(void) {
    gpioSetPullup(&PIN_DHT11_IO, gpioPullupMode_Inactive);
    gpioSetDir(PIN_DHT11, gpioDirection_Output);
    OUT(1);
    // LCD
    lcdClear();
    lcdRefresh();
}

void delay_us(int microsecs) {
  uint32_t delay = microsecs * ((CFG_CPU_CCLK / 100000) / 80);
  while (delay > 0)
  {
    __asm volatile ("nop");
    delay--;
  }
}

bool dht11_read_byte(uint8_t *out)
{
    int bit;
    int cnt;
    uint8_t result = 0;
    for(bit = 0; bit < 8; bit++)
    {
        cnt = 0;
        while (!gpioGetValue(PIN_DHT11)) {
            if (cnt++ > 1000) {
                return false;
            }
        }
        delay_us(30);
        if (gpioGetValue(PIN_DHT11) != 0 ) {
            result |= (1 << (7 - bit));
        }
        cnt = 0;
        while (gpioGetValue(PIN_DHT11)) {
            if (cnt++ > 1000) {
                return false;
            }
        }
    }
    *out = result;
    return true;
}

int read_sensor(struct sensordata *out)
{
    uint8_t raw_data[5];
    uint8_t checksum_calc;
    int i;

    // ugly for now, maybe use interrupts?
    gpioSetDir(PIN_DHT11, gpioDirection_Output);
    OUT(0);
    delay_us(18000);
    OUT(1);
    delay_us(1);
    gpioSetDir(PIN_DHT11, gpioDirection_Input);

    delay_us(40);

    if (gpioGetValue(PIN_DHT11)) {
        return -1;
    }
    delay_us(80);

    if (!gpioGetValue(PIN_DHT11)) {
        return -2;
    }
    delay_us(80);
    checksum_calc = 0;
    for (i = 0; i < sizeof(raw_data); i++) {
        if (!dht11_read_byte(&raw_data[i]))
            return -3;
    }
    for (i = 0; i < 4; i++) {
        checksum_calc += raw_data[i];
    }
    if (raw_data[4] != checksum_calc) {
        return -5;
    }
    out->humidity    = raw_data[0];
    out->temperature = raw_data[2];

    gpioSetDir(PIN_DHT11, gpioDirection_Output);
    OUT(1);

    return 0;
}

void main_dht11(void) {
    int res;
    struct sensordata data;
    int count = 0;
    init();
    while (true) {
        lcdPrintInt(count);
        count += 1;
        lcdPrint(":");
        lcdRefresh();
        res = read_sensor(&data);
        if (res != 0) {
            lcdPrint("Error ");
            lcdPrintInt(res);
        } else {
            lcdPrintInt(data.temperature);
            lcdPrint(",");
            lcdPrintInt(data.humidity);
        }
        lcdNl();
        lcdRefresh();
        delayms(80);
    }
}

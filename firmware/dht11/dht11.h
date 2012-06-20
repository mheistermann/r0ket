#include <sysinit.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "basic/basic.h"
#include "basic/config.h"

#define PIN_DHT11    RB_SPI_SS2
#define PIN_DHT11_IO IOCON_PIO2_8

struct dht11_data {
    uint8_t humidity;
    uint8_t temperature;
};

void dht11_init(void);
int dht11_read(struct dht11_data *out);

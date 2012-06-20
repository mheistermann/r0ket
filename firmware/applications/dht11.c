#include <sysinit.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "dht11/dht11.h"
#include "lcd/render.h"
#include "lcd/print.h"

void main_dht11(void) {
    int res;
    struct dht11_data data;
    int count = 0;
    dht11_init();
    while (true) {
        lcdPrintInt(count);
        count += 1;
        lcdPrint(":");
        lcdRefresh();
        res = dht11_read(&data);
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

/* slider-frickelei
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

#include "funk/nrf24l01p.h"

#define MY_PIN_A RB_SPI_SS0
#define MY_PIN_B RB_SPI_SS1



struct __attribute__ ((packed)) {
    char magic[4];
    uint8_t left, right;
    uint8_t crc[2];
} buf;

struct NRF_CFG nrf_config = {
    .channel = 80,
    .txmac = "\x1\x2\x3\x2\x7",
    .nrmacs = 1,
    .mac0 =  "\x1\x2\x3\x2\x7",
    .maclen = {sizeof(buf)},
};

void init(void) {
    // LEDs
    gpioSetDir(MY_PIN_A, gpioDirection_Output);
    gpioSetDir(MY_PIN_B, gpioDirection_Output);
    //off();

    // LCD
    lcdClear();
    lcdRefresh();

    // nrf
    nrf_init();
    nrf_set_strength(3);
    nrf_config_set(&nrf_config);
    nrf_rcv_pkt_start();
}

uint32_t slider_value(uint8_t ch) {
    uint32_t val;
    val = adcRead(ch);
    return val;
}


#define MAGIC "PLOP"

void remote_led(uint8_t left, uint8_t right) {
    buf.left = left;
    buf.right = right;
    lcdPrint("sending ");
    lcdPrintInt(left);
    lcdPrintInt(right);
    lcdNl();
    lcdRefresh();
    nrf_snd_pkt_crc_encr(sizeof(buf), (uint8_t *)&buf, NULL);
}

void pitcher(void) {
    uint32_t slider[2];
    memcpy(buf.magic, MAGIC, sizeof(buf.magic));
    lcdClear(); lcdRefresh();
    lcdPrintln("pitching...");
    while(true) {
        slider[0] = slider_value(2);
        slider[1] = slider_value(3);

        remote_led(1, 1);
        delayms(slider[1]);

        remote_led(0, 0);
        delayms(slider[0]);
    }
}

void catcher(void) {
    int len;
    lcdClear(); lcdRefresh();
    //nrf_rcv_pkt_start();
    while (true) {
        lcdPrint("wait."); lcdRefresh();
        //do {
            //len = nrf_rcv_pkt_poll_dec(sizeof(buf), (uint8_t *)&buf, NULL);
        //} while (len<=0);
        len = nrf_rcv_pkt_time_encr(1000, sizeof(buf), (uint8_t *)&buf, NULL);
        lcdPrint("got."); lcdRefresh();
        if (len != sizeof(buf)) {
            lcdPrintInt(len); lcdNl(); lcdRefresh();
            continue;
        }
        if (memcmp(buf.magic, MAGIC, sizeof(buf.magic)) != 0) {
            lcdPrintln("magic"); lcdNl(); lcdRefresh();
            continue;
        }
        lcdPrintln("yay, output time"); lcdNl(); lcdRefresh();
        gpioSetValue(MY_PIN_A, !!buf.left);
        gpioSetValue(MY_PIN_B, !!buf.right);
    }
}

static const struct MENU mainmenu = {
    .title = "slider",
    .entries = {
        {"pitcher", pitcher},
        {"catcher", catcher},
        {NULL, NULL},
    }
};

void main_slider(void) {
    init();
    handleMenu(&mainmenu);
}

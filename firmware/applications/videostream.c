/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <mh at sponc dot de> wrote this file. As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return - Martin Heistermann
 * ----------------------------------------------------------------------------
 */

/* Experimental monochrome video streaming to r0kets.
 * Two modes of operation:
 *   - a r0ket receives a video stream via usb(-cdc) and displays it
 *   - r0ket A forwards the usb video stream using the RF link to
 *     r0ket B, which will then display the video
 *
 * Host-side USB video streaming code is in r0ket/tools/videostream
 *
 * TODO:
 *  - do less in usb interrupts
 *  - cdcuser.c: store data into buf, then call Custom_CDC_BulkOut with
 *    the number of bytes. Do not keep "recvbuf" here
 */

#include <stdint.h>
#include <string.h>
#include "usbcdc/usb.h"
#include "usbcdc/usbcore.h"
#include "usbcdc/usbhw.h"
#include "usbcdc/cdcuser.h"
#include "usbcdc/cdc_buf.h"
#include <sysinit.h>
#include "basic/basic.h"
#include "lcd/print.h"
#include "lcd/render.h"
#include "lcd/allfonts.h"
#include "lcd/display.h"
#include "string.h"
#include "usbcdc/util.h"
#include "usbcdc/cdcuser.h"
#include "funk/nrf24l01p.h"

#if CFG_USBMSC
#error "MSC is defined, change firmware/core/projectconfig.h"
#endif

#if !CFG_USBCDC
#error "CDC is not defined, change firmware/core/projectconfig.h"
#endif

#define LCDBUF_MAX (RESX * RESY_B)

#define PART_SIZE 26

int recvbuf_len;
struct {
    uint8_t magic[3];
    uint8_t part;
    uint8_t data[PART_SIZE];
    uint8_t crc[2];
}  __attribute__((packed)) recvbuf;

struct NRF_CFG nrf_config = {
    .channel = 80,
    .txmac = "\x1\x2\x3\x2\x7",
    .nrmacs = 1,
    .mac0 =  "\x1\x2\x3\x2\x7",
    .maclen = {32},
};

bool pkt_okay() {
    if (recvbuf_len != 4 + PART_SIZE) {
        puts("l");
        return false;
    }
    if (recvbuf.magic[0] != 'M'
            || recvbuf.magic[1] != 'X'
            || recvbuf.magic[2] != 'N') {
        printf("m(%c%c%c)",
                recvbuf.magic[0],
                recvbuf.magic[1],
                recvbuf.magic[2]);
        return false;
    }
    return true;
}

void output_recvbuf() {
    int len = sizeof(recvbuf.data);
    int offset = recvbuf.part * PART_SIZE;
    if (len > LCDBUF_MAX - offset) {
        len = LCDBUF_MAX - offset;
    }
    memcpy(&lcdBuffer[offset], recvbuf.data, len);
    if (len == LCDBUF_MAX - offset) {
        puts("!");
    } else {
        puts (".");
    }
}

void bulkout_direct (void) {
    recvbuf_len = USB_ReadEP(CDC_DEP_OUT, (uint8_t *)&recvbuf);
    if (!pkt_okay())
        return;
    output_recvbuf();
}

void direct(void) {
    lcdClear();
    lcdPrintln("receiving");
    lcdPrintln("usb -> display");
    lcdDisplay();
    Custom_CDC_BulkOut = bulkout_direct;
    usbCDCInit();
    while(true) {
        lcdRefresh();
        //delayms_queue(1000);
    }

}

void bulkout_forward(void) {
    recvbuf_len = USB_ReadEP(CDC_DEP_OUT, (uint8_t *)&recvbuf);
    if (!pkt_okay())
        return;
    nrf_snd_pkt_crc_encr(sizeof(recvbuf), (uint8_t*)&recvbuf, NULL);
    output_recvbuf();
}
void nrf_setup(void) {
    nrf_init();
    nrf_set_strength(3);
    nrf_config_set(&nrf_config);
}

void forward(void) {
    lcdClear();
    lcdPrintln("forwarding");
    lcdPrintln("usb -> funk");
    lcdDisplay();
    Custom_CDC_BulkOut = bulkout_forward;
    usbCDCInit();
    nrf_setup();
    while(true) {delayms_queue(1000);}
}

void recv(void) {
    lcdClear();
    lcdPrintln("receiving");
    lcdPrintln("funk -> display");
    lcdDisplay();

    nrf_setup();
    nrf_rcv_pkt_start();
    while(true) {
        recvbuf_len = nrf_rcv_pkt_poll_dec(sizeof(recvbuf),
                (uint8_t*)&recvbuf, NULL);
        /*
        recvbuf_len -= 2;
        if (!pkt_okay())
            return false;
            */
        output_recvbuf();
    }
}


static const struct MENU mainmenu = {
    .title = "videostream",
    .entries = {
        {"direct", direct},
        {"forward", forward},
        {"recv", recv},
        {NULL, NULL},
    }
};

void main_videostream(void) {
    while(true) {
        Custom_CDC_BulkOut = 0;
        handleMenu(&mainmenu);
    }
}



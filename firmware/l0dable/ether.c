/* etherr0ket-frickelei
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


#include "enc28j60/enc28j60.h"
#include "enc28j60/net.h"
#include "funk/nrf24l01p.h"

#include "usetable.h"


static const struct MENU mainmenu;

void init(void);

void ram(void) {
    init();
    while(true) {
        handleMenu(&mainmenu);
    }
}

struct NRF_CFG nrf_config = {
    .channel = 80,
    .txmac = "\x1\x2\x3\x2\x7",
    .nrmacs = 1,
    .mac0 =  "\x1\x2\x3\x2\x7",
    .maclen = {32},
};

void nrf_setup(void) {
    nrf_set_strength(3);
    nrf_config_set(&nrf_config);
}



void init() {
    //lcdPrintln("init..."); lcdDisplay();

    initMAC();

    lcdPrintln("init done"); lcdDisplay();
    delayms_queue(500);
}

void dump_packet() {
    lcdClear();
    lcdPrintInt(ether_len);
    lcdNl();
    for(int offset = 0; offset < ether_len; offset++) {
        if (offset % 6 == 0) {
            lcdNl();
        }
        lcdPrintCharHex(ether_buf[offset]);
    }
    lcdDisplay();
}

void dump() {
    uint8_t key;
    //lcdPrintln("sniff!");
    //lcdDisplay();
    do {
        key = getInput();
        MACRead();
        dump_packet();
    } while(key != BTN_ENTER);
}



uint16_t ipid = 0;

typedef struct __attribute__((packed)){
    struct eth_hdr eth;
    struct ip_hdr ip;
    struct udp_hdr udp;
    char payload[0];
} packet;

void udp() {
    packet *p = (packet*)ether_buf;
    uint8_t key;

    //memset(ether_buf, 0x42, ETHER_BUFSIZE);
    ether_len = sizeof(*p) + sizeof(key);

    memcpy(p->eth.dest, "\0foobar", 6);
    memcpy(&p->eth.src, "\0foobas", 6);
    p->eth.type = HTONS(UIP_ETHTYPE_IP);
    p->ip.vhl = 0x45;
    p->ip.tos = 0;
    p->ip.len[0] = 0;
    p->ip.len[1] = ether_len; // TODO: htons?
    p->ip.ipoffset[0] = 0;
    p->ip.ipoffset[1] = 0;
    p->ip.ttl = 0xf0;
    p->ip.proto = UIP_PROTO_UDP;
    memcpy(p->ip.srcipaddr, "AAAA", 4);
    memcpy(p->ip.destipaddr, "BBBB", 4);

    p->udp.srcport = 0xdead;
    p->udp.destport = 0xbeef;
    p->udp.udplen = HTONS(sizeof(p->udp) + 1);



    do {
        key = getInput();

        p->payload[0] = key;
        p->ip.ipid[0] = ipid >> 8;
        p->ip.ipid[1] = ipid & 0xff;
        ipid++;

        p->ip.ipchksum = 0; // TODO XXX
        p->udp.udpchksum = 0; // TODO ggf

        lcdPrint("xm");
        lcdPrintInt(ether_len); lcdDisplay();
        lcdNl();
        MACWrite();
        delayms_queue(1000);
    } while(key != BTN_ENTER);
}



uint8_t nrf_to_ether() {
    ether_len = nrf_rcv_pkt_poll_dec(ETHER_BUFSIZE, ether_buf, NULL);

    if (ether_len <= 2) {
        return 0;
    }
    ether_len -= 2; // skip crc
    MACWrite();
    return 1;
}

uint8_t ether_to_nrf() {
    ether_len = MACRead();
    if (ether_len <= 0) {
        return 0;
    }
    // TODO: cut off packet at right position, exclude padding/crc
    nrf_snd_pkt_crc_encr(ether_len, ether_buf, NULL);
    return 1;
}

void ghettowifi() {
    uint8_t key;
    int count_ne = 0;
    int count_en = 0;


    nrf_setup();
    nrf_rcv_pkt_start();

    do {
        count_ne += nrf_to_ether();
        count_en += ether_to_nrf();


        lcdClear();
        lcdPrintInt(count_ne);
        lcdNl();
        lcdPrintInt(count_en);
        lcdDisplay();
        key = getInput();
    } while(key != BTN_ENTER);
    //nrf_off();
}

static const struct MENU mainmenu = {
    .title = "etherr0ket x",
    .entries = {
        {"init", init},
        {"dump", dump},
        {"udp", udp},
        {"ghettowifi", ghettowifi},
        {NULL, NULL},
    }
};



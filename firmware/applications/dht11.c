#include <sysinit.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "dht11/dht11.h"

#include "enc28j60/enc28j60.h"
#include "enc28j60/net.h"

#include "lcd/render.h"
#include "lcd/print.h"

struct __attribute__((packed)) my_payload {
    uint32_t id;
    int8_t dht11_status;
    uint8_t temperature;
    uint8_t humidity;
    uint32_t voltage;
};

struct __attribute__((packed)) packet {
    struct eth_hdr eth;
    struct ip_hdr ip;
    struct udp_hdr udp;
    struct my_payload payload;
};

void main_dht11(void) {
    struct packet *p = (struct packet*) ether_buf;
    struct my_payload *pl = &p->payload;

    uint16_t ipid = 0;
    int res;
    struct dht11_data data;

    dht11_init();
    initMAC();


    ether_len = sizeof(*p);

    memcpy(p->eth.dest, "\0foobar", 6);
    memcpy(&p->eth.src, "\0foobas", 6);
    p->eth.type = HTONS(UIP_ETHTYPE_IP);
    p->ip.vhl = 0x45;
    p->ip.tos = 0;
    p->ip.len[0] = 0;
    p->ip.len[1] = ether_len;
    p->ip.ipoffset[0] = 0;
    p->ip.ipoffset[1] = 0;
    p->ip.ttl = 0xf0;
    p->ip.proto = UIP_PROTO_UDP;
    memcpy(p->ip.srcipaddr, "AAAA", 4);
    memcpy(p->ip.destipaddr, "BBBB", 4);

    p->udp.srcport = 0xdead;
    p->udp.destport = 0xbeef;
    p->udp.udplen = HTONS(sizeof(p->udp) + sizeof(struct my_payload));

    p->payload.id = 0;

    data.temperature = 0xff;
    data.humidity = 0xff;

    while (true) {
        VoltageCheck();
        res = dht11_read(&data);

        pl->id++;
        pl->dht11_status = res;
        pl->temperature = data.temperature;
        pl->humidity = data.humidity;
        pl->voltage = GetVoltage();

        p->ip.ipid[0] = ipid >> 8;
        p->ip.ipid[1] = ipid & 0xff;
        ipid++;

        p->ip.ipchksum = 0; // TODO XXX
        p->udp.udpchksum = 0; // TODO ggf

        MACWrite();


        lcdClear();

        lcdPrint("Id: ");
        lcdPrintInt(pl->id);
        lcdNl();

        lcdPrint("Status: ");
        lcdPrintInt(pl->dht11_status);
        lcdNl();

        lcdPrint("Temp: ");
        lcdPrintInt(pl->temperature);
        lcdPrint("C");
        lcdNl();

        lcdPrint("Hum: ");
        lcdPrintInt(pl->humidity);
        lcdPrint("%");
        lcdNl();

        lcdPrint("Batt: ");
        lcdPrintInt(pl->voltage);
        lcdPrint("mV");

        lcdRefresh();
        delayms(500);
    }
}

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
    uint8_t proto_version;
    uint32_t count;
    int8_t dht11_status;
    uint8_t temperature;
    uint8_t humidity;
    uint16_t voltage;
};

struct __attribute__((packed)) packet {
    struct eth_hdr eth;
    struct ip_hdr ip;
    struct udp_hdr udp;
    struct my_payload payload;
};

uint16_t checksum(uint16_t startval, void *addr, uint16_t bytes)
{ // code based on code from rfc 1071
    /* Compute Internet Checksum for "bytes" bytes
     *         beginning at location "addr".
     */
    uint32_t sum = startval;

    while( bytes > 1 )  {
        /*  This is the inner loop */
        sum += * ((uint16_t*) addr);
        bytes -= 2;
        addr += 2;
    }

    /*  Add left-over byte, if any */
    if( bytes > 0 )
        sum += * (unsigned char *) addr;

    /*  Fold 32-bit sum to 16 bits */
    while (sum>>16)
        sum = (sum & 0xffff) + (sum >> 16);

    return ~(uint16_t)sum;
}

/*********** start  uip code *************/

void main_dht11(void) {
    struct packet *p = (struct packet*) ether_buf;
    struct my_payload *pl = &p->payload;
    const uint16_t udplen = sizeof(struct udp_hdr) + sizeof(struct my_payload);

    //uint16_t chksum = 0;
    uint16_t ipid = 0;
    int res = 0;
    struct dht11_data data;

    dht11_init();
    initMAC();

    // LCD
    lcdClear();
    lcdPrintln ("dht11-ip sensor");
    lcdRefresh();


    ether_len = sizeof(*p);

    memcpy(p->eth.dest, "\x1c\xbd\xb9\xa9\xd6\x4e", 6);
    memcpy(&p->eth.src, "\0CC_CP", 6);
    p->eth.type = HTONS16(UIP_ETHTYPE_IP);
    p->ip.vhl = 0x45;
    p->ip.tos = 0;
    p->ip.len[0] = 0;
    p->ip.len[1] = ether_len - sizeof(struct eth_hdr);
    p->ip.ipoffset[0] = 0;
    p->ip.ipoffset[1] = 0;
    p->ip.ttl = 0xf0;
    p->ip.proto = UIP_PROTO_UDP;
    p->ip.srcipaddr = HTONS32(0x0a0a00aa);
    //p->ip.destipaddr = HTONS32(0x0a0a0003);
    p->ip.destipaddr = HTONS32(0xb009b264);

    p->udp.srcport = 0xdead;
    p->udp.destport = 0xbeef;
    p->udp.udplen = HTONS16(udplen);

    p->payload.count = 0;
    p->payload.proto_version = 0;

    data.temperature = 0xff;
    data.humidity = 0xff;

    while (true) {
        lcdRefresh();

        VoltageCheck();

        lcdRefresh();
        res = dht11_read(&data);

        lcdClear();
        lcdRefresh();
        pl->count++;
        pl->dht11_status = res;
        pl->temperature = data.temperature;
        pl->humidity = data.humidity;
        pl->voltage = GetVoltage();

        // TODO: HMAC or similar

        p->ip.ipid[0] = ipid >> 8;
        p->ip.ipid[1] = ipid & 0xff;
        ipid++;

        p->ip.ipchksum = 0;
        p->ip.ipchksum = checksum(0, &p->ip, sizeof(struct ip_hdr));
        p->udp.udpchksum = 0;
        /* TODO
        chksum = checksum(HTONS16(udplen) + UIP_PROTO_UDP, &p->udp, udplen);
        p->udp.udpchksum = checksum(chksum, &p->ip.srcipaddr, sizeof(uint32_t) * 2);

        if (p->udp.udpchksum == 0)
            p->udp.udpchksum = 0xffff;
            */

        MACWrite();


        lcdClear();

        lcdPrint("cnt: ");
        lcdPrintInt(pl->count);
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

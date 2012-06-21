// packet structures and defines taken from / based on uip 1.0

#define UIP_ETHTYPE_ARP 0x0806
#define UIP_ETHTYPE_IP  0x0800
#define UIP_ETHTYPE_IP6 0x86dd
#define UIP_PROTO_ICMP  1
#define UIP_PROTO_TCP   6
#define UIP_PROTO_UDP   17
#define UIP_PROTO_ICMP6 58

#define HTONS(n) (uint16_t)((((uint16_t) (n)) << 8) | (((uint16_t) (n)) >> 8))

struct __attribute__ ((packed)) eth_hdr {
  uint8_t dest[6];
  uint8_t src[6];
  uint16_t type;
};

struct __attribute__((packed)) ip_hdr {
    /* IP header. */
    uint8_t vhl,
         tos,
         len[2],
         ipid[2],
         ipoffset[2],
         ttl,
         proto;
    uint16_t ipchksum;
    uint16_t srcipaddr[2],
          destipaddr[2];
};

struct __attribute__((packed)) udp_hdr {
    /* UDP header. */
    uint16_t srcport,
          destport;
    uint16_t udplen;
    uint16_t udpchksum;
};



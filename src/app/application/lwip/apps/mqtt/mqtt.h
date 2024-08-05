/*
 * mqtt.h
 *
 *  Created on: 23. 10. 2013
 *      Author: hp
 */

#ifndef MQTT_H_
#define MQTT_H_

#include "dhcp.h"
#include "dns.h"
#include "etharp.h"
#include "init.h"
#include "loopif.h"
#include "lwip_sys.h"
#include "netif.h"
#include "opt.h"
#include "pbuf.h"
#include "stats.h"
#include "tcp.h"
#include "udp.h"

#include <stdint.h>

#define MQTTCONNECT   1 << 4
#define MQTTPUBLISH   3 << 4
#define MQTTSUBSCRIBE 8 << 4

#define MAX_PACKET_SIZE 128

#define KEEPALIVE 5000

typedef struct Mqtt Mqtt;

typedef void (*msgReceived)(Mqtt *this, uint8_t *topic, uint8_t topicLen, uint8_t *data, uint32_t dataLen);

struct Mqtt {
    struct ip_addr server;
    uint16_t port;
    uint8_t connected;
    uint8_t autoConnect;
    struct tcp_pcb *pcb;
    uint32_t lastActivity;
    msgReceived msgReceivedCallback;
    char *deviceId;
    uint8_t pollAbortCounter;
};

Mqtt mqtt;

typedef struct MqttFixedHeader {
    uint8_t header;
    uint8_t remainingLength;
} MqttFixedHeader;

#define MQTT_MSGT_PINGREQ  (12 << 4)
#define MQTT_MSGT_PINGRESP (13 << 4)
#define MQTT_MSGT_PUBLISH  (3 << 4)
#define MQTT_MSGT_CONACK   (2 << 4)

#define MQTT_PINGREQ_HEADER (MQTT_MSGT_PINGREQ)

void mqttInit(Mqtt *mqtt, struct ip_addr serverIp, int port, msgReceived fn, char *devId);
uint8_t mqttConnect(Mqtt *this);
uint8_t mqttPublish(Mqtt *this, char *pub_topic, char *msg);
uint8_t mqttDisconnect(Mqtt *this);
uint8_t mqttSubscribe(Mqtt *this, char *topic);
uint8_t mqttLive(Mqtt *this);
void mqttDisconnectForced(Mqtt *this);

#endif /* MQTT_H_ */

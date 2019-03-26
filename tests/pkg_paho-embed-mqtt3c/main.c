/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       MQTT Packet Serialization library package test application
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>

#include "MQTTPacket.h"

static void print_buffer(const uint8_t *buffer, size_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        printf(" %02x", buffer[i]);
    }
}

int main(void)
{
    puts("MQTT Serialization test\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;

    uint8_t buf[200];
    int buflen = sizeof(buf);

    MQTTString topicString = MQTTString_initializer;
    char* payload = "mypayload";
    int payloadlen = strlen(payload);
    int len = 0;
    char *host = "m2m.eclipse.org";
    int port = 1883;

    printf("Sending to hostname %s port %d\n", host, port);

    data.clientID.cstring = "me";
    data.keepAliveInterval = 20;
    data.cleansession = 1;
    data.username.cstring = "testuser";
    data.password.cstring = "testpassword";
    data.MQTTVersion = 4;

    len = MQTTSerialize_connect((unsigned char *)buf, buflen, &data);
    puts("MQTT Serialize connect:");
    print_buffer(buf, len);
    puts("");
    topicString.cstring = "mytopic";
    len += MQTTSerialize_publish((unsigned char *)(buf + len), buflen - len, 0, 0, 0, 0, topicString, (unsigned char *)payload, payloadlen);
    puts("Add MQTT Serialize publish:");
    print_buffer(buf, len);
    puts("");
    len += MQTTSerialize_disconnect((unsigned char *)(buf + len), buflen - len);
    puts("Add MQTT Serialize disconnect:");
    print_buffer(buf, len);
    puts("");
    return 0;
}

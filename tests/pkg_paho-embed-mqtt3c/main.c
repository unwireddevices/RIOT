/*
 * Copyright (C) 2017 Kaspar Schleiser <kaspar@schleiser.de>
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
 * @brief       minmea GPS NMEA parser library package test application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include <stdio.h>

#include "MQTTPacket.h"

int main(void)
{
    puts("START");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    // MQTTString topicString = MQTTString_initializer;

    data.clientID.cstring = (char *)"Center2M";
    data.keepAliveInterval = 20;
    data.cleansession = 1;
    data.MQTTVersion = 3;
    data.username.cstring = (char *)"dew00001";
    data.password.cstring = (char *)"kjz&450#1A";

    printf("%s\n", (char *)&data);

    return 0;
}
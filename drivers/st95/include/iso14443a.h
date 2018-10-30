/*
 * Copyright (C) 2018 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup
 * @ingroup
 * @brief
 * @{
 * @file		iso14443a.h
 * @brief       protocol ISO14443A for ST95
 * @author      Mikhail Perkov
 */

#ifndef ISO14443A_H
#define ISO14443A_H

#ifdef __cplusplus
extern "C" {
#endif

#define ST95_TX_RATE_106                0
#define ST95_RX_RATE_106                0
#define ST95_TX_RATE_212                1
#define ST95_RX_RATE_212                1
#define ST95_TX_RATE_424                2
#define ST95_RX_RATE_424                2
#define ST95_TX_RATE_14443A             ST95_TX_RATE_106
#define ST95_RX_RATE_14443A             ST95_RX_RATE_106

/* Anticollison levels (commands) */
#define ISO14443A_SELECT_LVL1   0x93
#define ISO14443A_SELECT_LVL2	0x95
#define ISO14443A_SELECT_LVL3	0x97

/* ATQ FLAG */
#define ISO14443A_ATQA_SINGLE	0
#define	ISO14443A_ATQA_DOUBLE	1
#define ISO14443A_ATQA_TRIPLE	2
/* UID Sizes */
#define ISO14443A_UID_SINGLE	4
#define ISO14443A_UID_DOUBLE	7
#define ISO14443A_UID_TRIPLE	10


int get_uid_14443a(const st95_t * dev, uint8_t * length_uid, uint8_t * uid, uint8_t * sak);

#ifdef __cplusplus
}
#endif

#endif /* ISO14443A_H */
/** @} */

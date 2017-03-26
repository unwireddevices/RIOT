/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
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
 * @file		ls-regions.h
 * @brief       Describes LoRa region-dependent frequency channels
 * @author      Eugene Ponomarev
 */
#ifndef LS_REGIONS_H_
#define LS_REGIONS_H_

typedef struct {
	char region[10];

	uint8_t num_channels;
	uint32_t channels[5];
} ls_region_t;

#define LS_UNI_NUM_REGIONS 2

static const ls_region_t regions[LS_UNI_NUM_REGIONS] = {
		{ "Europe 868", 3,
			{
				868100000,
				868300000,
				868500000
			}
		},

		{ "Russia 868", 2,
			{
					868850000,
					869050000
			}
		}
};

#endif /* LS_REGIONS_H_ */

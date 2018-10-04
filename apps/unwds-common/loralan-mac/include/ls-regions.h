/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
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

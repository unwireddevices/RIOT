/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx128x
 * @{
 * @file
 * @brief       Semtech SX128X ranging correction
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 */

#ifndef __SX128X_RANGING_CORRECT_H__
#define __SX128X_RANGING_CORRECT_H__

#include <stdint.h>

#include "ranging_correct_type.h"
#include "ranging_correction_sf5_bw0400.h"
#include "ranging_correction_sf6_bw0400.h"
#include "ranging_correction_sf7_bw0400.h"
#include "ranging_correction_sf8_bw0400.h"
#include "ranging_correction_sf9_bw0400.h"
#include "ranging_correction_sf10_bw0400.h"
#include "ranging_correction_sf5_bw0800.h"
#include "ranging_correction_sf6_bw0800.h"
#include "ranging_correction_sf7_bw0800.h"
#include "ranging_correction_sf8_bw0800.h"
#include "ranging_correction_sf9_bw0800.h"
#include "ranging_correction_sf10_bw0800.h"
#include "ranging_correction_sf5_bw1600.h"
#include "ranging_correction_sf6_bw1600.h"
#include "ranging_correction_sf7_bw1600.h"
#include "ranging_correction_sf8_bw1600.h"
#include "ranging_correction_sf9_bw1600.h"
#include "ranging_correction_sf10_bw1600.h"

#ifdef __cplusplus
extern "C" {
#endif

const int32_t *ranging_correction_per_sf_bw_gain[6][3] = {
    { &ranging_correction_sf5_bw0400[0],  &ranging_correction_sf5_bw0800[0],  &ranging_correction_sf5_bw1600[0] },
    { &ranging_correction_sf6_bw0400[0],  &ranging_correction_sf6_bw0800[0],  &ranging_correction_sf6_bw1600[0] },
    { &ranging_correction_sf7_bw0400[0],  &ranging_correction_sf7_bw0800[0],  &ranging_correction_sf7_bw1600[0] },
    { &ranging_correction_sf8_bw0400[0],  &ranging_correction_sf8_bw0800[0],  &ranging_correction_sf8_bw1600[0] },
    { &ranging_correction_sf9_bw0400[0],  &ranging_correction_sf9_bw0800[0],  &ranging_correction_sf9_bw1600[0] },
    { &ranging_correction_sf10_bw0400[0], &ranging_correction_sf10_bw0800[0], &ranging_correction_sf10_bw1600[0] },
};

const sx128x_ranging_correction_polynomes_t *ranging_correction_polynomes_per_sf_bw[6][3] = {
    { &correction_ranging_polynome_sf5_bw0400,  &correction_ranging_polynome_sf5_bw0800,  &correction_ranging_polynome_sf5_bw1600 },
    { &correction_ranging_polynome_sf6_bw0400,  &correction_ranging_polynome_sf6_bw0800,  &correction_ranging_polynome_sf6_bw1600 },
    { &correction_ranging_polynome_sf7_bw0400,  &correction_ranging_polynome_sf7_bw0800,  &correction_ranging_polynome_sf7_bw1600 },
    { &correction_ranging_polynome_sf8_bw0400,  &correction_ranging_polynome_sf8_bw0800,  &correction_ranging_polynome_sf8_bw1600 },
    { &correction_ranging_polynome_sf9_bw0400,  &correction_ranging_polynome_sf9_bw0800,  &correction_ranging_polynome_sf9_bw1600 },
    { &correction_ranging_polynome_sf10_bw0400, &correction_ranging_polynome_sf10_bw0800, &correction_ranging_polynome_sf10_bw1600 },
};

#ifdef __cplusplus
}
#endif

#endif /* __SX128X_RANGING_CORRECT_H__ */
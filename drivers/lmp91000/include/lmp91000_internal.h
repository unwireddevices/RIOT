/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_lmp91000
 * @{
 *
 * @file
 * @brief       Definition for the LMP91000 Sensor AFE System
 * 
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */
#ifndef _LMP91000_INTERNAL_H
#define _LMP91000_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    LMP91000 registers map
 * @{
 */
#define LMP91000_STATUS                     (0x00)      /**< */
#define LMP91000_LOCK                       (0x01)      /**< */
#define LMP91000_TIACN                      (0x10)      /**< */
#define LMP91000_REFCN                      (0x11)      /**< */
#define LMP91000_MODECN                     (0x12)      /**< */
/**
 * @}
 */

/**
 * @name    LMP91000 masks for register STATUS
 * @{
 */
#define LMP91000_MASK_STATUS_READY          (0x01)      /**< */
/**
 * @}
 */

/**
 * @name    LMP91000 masks for register LOCK
 * @{
 */
#define LMP91000_MASK_LOCK_PROTECT          (0x01)      /**< */
/**
 * @}
 */

/**
 * @name    LMP91000 masks for register TIACN
 * @{
 */
#define LMP91000_MASK_TIACN_RLOAD           (0x03)      /**< */
#define LMP91000_MASK_TIACN_TIA_GAIN        (0x1C)      /**< */
/**
 * @}
 */


/**
 * @name    LMP91000 masks for register REFCN
 * @{
 */
#define LMP91000_MASK_REFCN_BIAS            (0x0F)      /**< */
#define LMP91000_MASK_REFCN_BIAS_SIGN       (0x10)      /**< */
#define LMP91000_MASK_REFCN_INT_Z           (0x60)      /**< */
#define LMP91000_MASK_REFCN_REF_SOURCE      (0x80)      /**< */
/**
 * @}
 */

/**
 * @name    LMP91000 masks for register MODECN 
 * @{
 */
#define LMP91000_MASK_MODECN_OP_MODE        (0x07)      /**< */
#define LMP91000_MASK_MODECN_FET_SHORT      (0x80)      /**< */
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _LMP91000_INTERNAL_H */
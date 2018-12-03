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
#define LMP91000_STATUS                     (0x00)      /**< Status Register (Address 0x00)            */
#define LMP91000_LOCK                       (0x01)      /**< Protection Register (Address 0x01)        */
#define LMP91000_TIACN                      (0x10)      /**< TIA Control Register (Address 0x10)       */
#define LMP91000_REFCN                      (0x11)      /**< Reference Control Register (Address 0x11) */
#define LMP91000_MODECN                     (0x12)      /**< Mode Control Register (Address 0x12)      */
/**
 * @}
 */

/**
 * @name    LMP91000 masks for register STATUS
 * @{
 */
#define LMP91000_MASK_STATUS_READY          (0x01)      /**< Mask bitfield: Status of Davice */      
/**
 * @}
 */

/**
 * @name    LMP91000 masks for register LOCK
 * @{
 */
#define LMP91000_MASK_LOCK_PROTECT          (0x01)      /**< Mask bitfield: Write protection */      
/**
 * @}
 */

/**
 * @name    LMP91000 masks for register TIACN
 * @{
 */
#define LMP91000_MASK_TIACN_RLOAD           (0x03)      /**< Mask bitfield: Rload selection                     */
#define LMP91000_MASK_TIACN_TIA_GAIN        (0x1C)      /**< Mask bitfield: TIA feedback resistance selection   */
/**
 * @}
 */


/**
 * @name    LMP91000 masks for register REFCN
 * @{
 */
#define LMP91000_MASK_REFCN_BIAS            (0x0F)      /**< Mask bitfield: BIAS selection                     */
#define LMP91000_MASK_REFCN_BIAS_SIGN       (0x10)      /**< Mask bitfield: Selection of the BIAS polarity     */
#define LMP91000_MASK_REFCN_INT_Z           (0x60)      /**< Mask bitfield: Internal zero selection            */
#define LMP91000_MASK_REFCN_REF_SOURCE      (0x80)      /**< Mask bitfield: Reference voltage source selection */
/**
 * @}
 */

/**
 * @name    LMP91000 masks for register MODECN 
 * @{
 */
#define LMP91000_MASK_MODECN_OP_MODE        (0x07)      /**< Mask bitfield: Mode of Operation selection */
#define LMP91000_MASK_MODECN_FET_SHORT      (0x80)      /**< Mask bitfield: Shorting FET feature        */
/**
 * @}
 */

/**
 * @brief  LMP91000 Properties
 *
 * @{ 
 */
#define LMP91000_I2C_TIMEOUT                (200)     /**< I2C Time out (ms), this is the maximum time needed by LMP91000 to ready state */
/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* _LMP91000_INTERNAL_H */
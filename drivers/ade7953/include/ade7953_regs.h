/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
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
 * @file		ade7953_regs.h
 * @brief       Register definition for ADE7953 device
 * @author      Mikhail Perkov
 */

#ifndef ADE7953_REGS_H
#define ADE7953_REGS_H


#ifdef __cplusplus
extern "C" {
#endif

/* Register Addresses */
// 8-bits Registers
#define ADE7953_SAGCYC_8        0x000   //  Sag line cycles
#define ADE7953_DISNOLOAD_8     0x001   //  No-load detection disable
#define ADE7953_LCYCMODE_8      0x004   //  Line cycle accumulation mode configuration
#define ADE7953_PGA_V_8         0x007   //  Voltage channel gain configuration (Bits[2:0])
#define ADE7953_PGA_IA_8        0x008   //  Current Channel A gain configuration (Bits[2:0])
#define ADE7953_PGA_IB_8        0x009   //  Current Channel B gain configuration (Bits[2:0])
#define ADE7953_WRITE_PROTECT_8 0x040   //  Write protection bits (Bits[2:0])
#define ADE7953_LAST_OP_8       0x0FD   //  Contains the type (read or write) of the last successful communication (0x35 = read; 0xCA = write)
#define ADE7953_UNLOCK_REG_8    0x0FE   //  Addres Unlock register for unlock Register 0x120 address
#define ADE7953_LAST_RWDATA_8   0x0FF   //  Contains the data from the last successful 8-bit register communication
#define ADE7953_VERSION_8       0x702   //  Contains the silicon version number
#define ADE7953_EX_REF_8        0x800   //  Reference input configuration: set to 0 for internal; set to 1 for external

// 16-bits Registers
#define ADE7953_ZXTOUT_16        0x100   //  Zero-crossing timeout
#define ADE7953_LINECYC_16       0x101   //  Number of half line cycles for line cycle energy accumulation mode
#define ADE7953_CONFIG_16        0x102   //  Configuration register
#define ADE7953_CF1DEN_16        0x103   //  CF1 frequency divider denominator. When modifying this register, two sequential write operations must be performed to ensure that the write is successful.
#define ADE7953_CF2DEN_16        0x104   //  CF2 frequency divider denominator. When modifying this register, two sequential write operations must be performed to ensure that the write is successful.
#define ADE7953_CFMODE_16        0x107   //  CF output selection
#define ADE7953_PHCALA_16        0x108   //  Phase calibration register (Current Channel A). This register is in sign magnitude format.
#define ADE7953_PHCALB_16        0x109   //  Phase calibration register (Current Channel B). This register is in sign magnitude format.
#define ADE7953_PFA_16           0x10A   //  Power factor (Current Channel A)
#define ADE7953_PFB_16           0x10B   //  Power factor (Current Channel B)
#define ADE7953_ANGLE_A_16       0x10C   //  Angle between the voltage input and the Current Channel A input
#define ADE7953_ANGLE_B_16       0x10D   //  Angle between the voltage input and the Current Channel B input
#define ADE7953_PERIOD_16        0x10E   //  Period register
#define ADE7953_ALT_OUTPUT_16    0x110   //  Alternative output functions (see Table 20)
#define ADE7953_SETUP_REG_16     0x120   //  This register should be set to 30h
#define ADE7953_LAST_ADD_16      0x1FE   //  Contains the address of the last successful communication
#define ADE7953_LAST_RWDATA_16   0x1FF   //  Contains the data from the last successful 16-bit register communication

// 24-Bit Registers
#define ADE7953_SAGLVL_24      0x200   // Sag voltage level
#define ADE7953_ACCMODE_24     0x201   // Accumulation mode
#define ADE7953_AP_NOLOAD_24   0x203   // Active power no-load level
#define ADE7953_VAR_NOLOAD_24  0x204   // Reactive power no-load level
#define ADE7953_VA_NOLOAD_24   0x205   // Apparent power no-load level
#define ADE7953_AVA_24         0x210   // Instantaneous apparent power (Current Channel A)
#define ADE7953_BVA_24         0x211   // Instantaneous apparent power (Current Channel B)
#define ADE7953_AWATT_24       0x212   // Instantaneous active power (Current Channel A)
#define ADE7953_BWATT_24       0x213   // Instantaneous active power (Current Channel B)
#define ADE7953_AVAR_24        0x214   // Instantaneous reactive power (Current Channel A)
#define ADE7953_BVAR_24        0x215   // Instantaneous reactive power (Current Channel B)
#define ADE7953_IA_24          0x216   // Instantaneous current (Current Channel A)
#define ADE7953_IB_24          0x217   // Instantaneous current (Current Channel B)
#define ADE7953_V_24           0x218   // Instantaneous voltage (voltage channel)
#define ADE7953_IRMSA_24       0x21A   // IRMS register (Current Channel A)
#define ADE7953_IRMSB_24       0x21B   // IRMS register (Current Channel B)
#define ADE7953_VRMS_24        0x21C   // VRMS register
#define ADE7953_AENERGYA_24    0x21E   // Active energy (Current Channel A)
#define ADE7953_AENERGYB_24    0x21F   // Active energy (Current Channel B)
#define ADE7953_RENERGYA_24    0x220   // Reactive energy (Current Channel A)
#define ADE7953_RENERGYB_24    0x221   // Reactive energy (Current Channel B)
#define ADE7953_APENERGYA_24   0x222   // Apparent energy (Current Channel A)
#define ADE7953_APENERGYB_24   0x223   // Apparent energy (Current Channel B)
#define ADE7953_OVLVL_24       0x224   // Overvoltage level
#define ADE7953_OILVL_24       0x225   // Overcurrent level
#define ADE7953_VPEAK_24       0x226   // Voltage channel peak
#define ADE7953_RSTVPEAK_24    0x227   // Read voltage peak with reset
#define ADE7953_IAPEAK_24      0x228   // Current Channel A peak
#define ADE7953_RSTIAPEAK_24   0x229   // Read Current Channel A peak with reset
#define ADE7953_IBPEAK_24      0x22A   // Current Channel B peak
#define ADE7953_RSTIBPEAK_24   0x22B   // Read Current Channel B peak with reset
#define ADE7953_IRQENA_24      0x22C   // Interrupt enable (Current Channel A)
#define ADE7953_IRQSTATA_24    0x22D   // Interrupt status (Current Channel A)
#define ADE7953_RSTIRQSTATA_24 0x22E   // Reset interrupt status (Current Channel A)
#define ADE7953_IRQENB_24      0x22F   // Interrupt enable (Current Channel B)
#define ADE7953_IRQSTATB_24    0x230   // Interrupt status (Current Channel B)
#define ADE7953_RSTIRQSTATB_24 0x231   // Reset interrupt status (Current Channel B)
#define ADE7953_AIGAIN_24      0x280   // Current channel gain (Current Channel A)
#define ADE7953_AVGAIN_24      0x281   // Voltage channel gain
#define ADE7953_AWGAIN_24      0x282   // Active power gain (Current Channel A)
#define ADE7953_AVARGAIN_24    0x283   // Reactive power gain (Current Channel A)
#define ADE7953_AVAGAIN_24     0x284   // Apparent power gain (Current Channel A)
#define ADE7953_AIRMSOS_24     0x286   // IRMS offset (Current Channel A)
#define ADE7953_VRMSOS_24      0x288   // VRMS offset
#define ADE7953_AWATTOS_24     0x289   // Active power offset correction (Current Channel A)
#define ADE7953_AVAROS_24      0x28A   // Reactive power offset correction (Current Channel A)
#define ADE7953_AVAOS_24       0x28B   // Apparent power offset correction (Current Channel A)ADE7953
#define ADE7953_BIGAIN_24      0x28C   // Current channel gain (Current Channel B) 
#define ADE7953_BVGAIN_24      0x28D   // This register should not be modified. 
#define ADE7953_BWGAIN_24      0x28E   // Active power gain (Current Channel B) 
#define ADE7953_BVARGAIN_24    0x28F   // Reactive power gain (Current Channel  B) 
#define ADE7953_BVAGAIN_24     0x290   // Apparent power gain (Current Channel B) 
#define ADE7953_BIRMSOS_24     0x292   // IRMS offset (Current Channel B) 
#define ADE7953_BWATTOS_24     0x295   // Active power offset correction (Current Channel B) 
#define ADE7953_BVAROS_24      0x296   // Reactive power offset correction (Current Channel B) 
#define ADE7953_BVAOS_24       0x297   // Apparent power offset correction (Current Channel B) 
#define ADE7953_LAST_RWDATA_24 0x2FF   //  Contains the data from the last successful 24-bit register communication

// 32-Bit Registers
#define ADE7953_SAGLVL_32      0x300   // Sag voltage level
#define ADE7953_ACCMODE_32     0x301   // Accumulation mod
#define ADE7953_AP_NOLOAD_32   0x303   // Active power no-load level
#define ADE7953_VAR_NOLOAD_32  0x304   // Reactive power no-load level
#define ADE7953_VA_NOLOAD_32   0x305   // Apparent power no-load level
#define ADE7953_AVA_32         0x310   // Instantaneous apparent power (Current Channel A)
#define ADE7953_BVA_32         0x311   // Instantaneous apparent power (Current Channel B)
#define ADE7953_AWATT_32       0x312   // Instantaneous active power (Current Channel A)
#define ADE7953_BWATT_32       0x313   // Instantaneous active power (Current Channel B)
#define ADE7953_AVAR_32        0x314   // Instantaneous reactive power (Current Channel A)
#define ADE7953_BVAR_32        0x315   // Instantaneous reactive power (Current Channel B)
#define ADE7953_IA_32          0x316   // Instantaneous current (Current Channel A)
#define ADE7953_IB_32          0x317   // Instantaneous current (Current Channel B)
#define ADE7953_V_32           0x318   // Instantaneous voltage (voltage channel)
#define ADE7953_IRMSA_32       0x31A   // IRMS register (Current Channel A)
#define ADE7953_IRMSB_32       0x31B   // IRMS register (Current Channel B)
#define ADE7953_VRMS_32        0x31C   // VRMS register
#define ADE7953_AENERGYA_32    0x31E   // Active energy (Current Channel A)
#define ADE7953_AENERGYB_32    0x31F   // Active energy (Current Channel B)
#define ADE7953_RENERGYA_32    0x320   // Reactive energy (Current Channel A)
#define ADE7953_RENERGYB_32    0x321   // Reactive energy (Current Channel B)
#define ADE7953_APENERGYA_32   0x322   // Apparent energy (Current Channel A)
#define ADE7953_APENERGYB_32   0x323   // Apparent energy (Current Channel B)
#define ADE7953_OVLVL_32       0x324   // Overvoltage level
#define ADE7953_OILVL_32       0x325   // Overcurrent level
#define ADE7953_VPEAK_32       0x326   // Voltage channel peak
#define ADE7953_RSTVPEAK_32    0x327   // Read voltage peak with reset
#define ADE7953_IAPEAK_32      0x328   // Current Channel A peak
#define ADE7953_RSTIAPEAK_32   0x329   // Read Current Channel A peak with reset
#define ADE7953_IBPEAK_32      0x32A   // Current Channel B peak
#define ADE7953_RSTIBPEAK_32   0x32B   // Read Current Channel B peak with reset
#define ADE7953_IRQENA_32      0x32C   // Interrupt enable (Current Channel A)
#define ADE7953_IRQSTATA_32    0x32D   // Interrupt status (Current Channel A)
#define ADE7953_RSTIRQSTATA_32 0x32E   // Reset interrupt status (Current Channel A)
#define ADE7953_IRQENB_32      0x32F   // Interrupt enable (Current Channel B)
#define ADE7953_IRQSTATB_32    0x330   // Interrupt status (Current Channel B)
#define ADE7953_RSTIRQSTATB_32 0x331   // Reset interrupt status (Current Channel B)
#define ADE7953_AIGAIN_32      0x380   // Current channel gain (Current Channel A)
#define ADE7953_AVGAIN_32      0x381   // Voltage channel gain
#define ADE7953_AWGAIN_32      0x382   // Active power gain (Current Channel A)
#define ADE7953_AVARGAIN_32    0x383   // Reactive power gain (Current Channel A)
#define ADE7953_AVAGAIN_32     0x384   // Apparent power gain (Current Channel A)
#define ADE7953_AIRMSOS_32     0x386   // IRMS offset (Current Channel A)
#define ADE7953_VRMSOS_32      0x388   // VRMS offset
#define ADE7953_AWATTOS_32     0x389   // Active power offset correction (Current Channel A)
#define ADE7953_AVAROS_32      0x38A   // Reactive power offset correction (Current Channel A)
#define ADE7953_AVAOS_32       0x38B   // Apparent power offset correction (Current Channel A)ADE7953
#define ADE7953_BIGAIN_32      0x38C   // Current channel gain (Current Channel B) 
#define ADE7953_BVGAIN_32      0x38D   // This register should not be modified. 
#define ADE7953_BWGAIN_32      0x38E   // Active power gain (Current Channel B) 
#define ADE7953_BVARGAIN_32    0x38F   // Reactive power gain (Current Channel  B) 
#define ADE7953_BVAGAIN_32     0x390   // Apparent power gain (Current Channel B) 
#define ADE7953_BIRMSOS_32     0x392   // IRMS offset (Current Channel B) 
#define ADE7953_BWATTOS_32     0x395   // Active power offset correction (Current Channel B) 
#define ADE7953_BVAROS_32      0x396   // Reactive power offset correction (Current Channel B) 
#define ADE7953_BVAOS_32       0x397   // Apparent power offset correction (Current Channel B) 
#define ADE7953_LAST_RWDATA_32 0x3FF   //  Contains the data from the last successful 24-bit register communication

// Register Interrupt enable (IRQENA) bits
#define ADE7953_IRQENA_AEHFA       0x000001     // interrupt when the active energy is half full (Current Channel A)
#define ADE7953_IRQENA_VAREHFA     0x000002     // interrupt when the reactive energy is half full (Current Channel A)
#define ADE7953_IRQENA_VAEHFA      0x000004     // interrupt when the apparent energy is half full (Current Channel A)
#define ADE7953_IRQENA_AEOFA       0x000008     // interrupt when the active energy has overflowed or underflowed (Current Channel A)
#define ADE7953_IRQENA_VAREOFA     0x000010      // interrupt when the reactive energy has overflowed or underflowed (Current Channel A)
#define ADE7953_IRQENA_VAEOFA      0x000020      // interrupt when the apparent energy has overflowed or underflowed (Current Channel A)
#define ADE7953_IRQENA_AP_NOLOADA  0x000040      // interrupt when the active power no-load condition is detected on Current Channel A
#define ADE7953_IRQENA_VAR_NOLOADA 0x000080      // interrupt when the reactive power no-load condition is detected on Current Channel A
#define ADE7953_IRQENA_VA_NOLOADA  0x000100      // interrupt when the apparent power no-load condition is detected on Current Channel A
#define ADE7953_IRQENA_APSIGN_A    0x000200      // interrupt when the sign of active energy has changed (Current Channel A)
#define ADE7953_IRQENA_VARSIGN_A   0x000400      // interrupt when the sign of reactive energy has changed (Current Channel A)
#define ADE7953_IRQENA_ZXTO_IA     0x000800      // interrupt when the zero crossing has been missing on Current Channel A for the length of time specified in the ZXTOUT register
#define ADE7953_IRQENA_ZXIA        0x001000      // interrupt when the current Channel A zero crossing occurs
#define ADE7953_IRQENA_OIA         0x002000      // interrupt when the current Channel A peak has exceeded the overcurrent threshold set in the OILVL register
#define ADE7953_IRQENA_ZXTO        0x004000      // interrupt when a zero crossing has been missing on the voltage channel for the length of time specified in the ZXTOUT register
#define ADE7953_IRQENA_ZXV         0x008000      // interrupt when the voltage channel zero crossing occurs
#define ADE7953_IRQENA_OV          0x010000      // interrupt when the voltage peak has exceeded the overvoltage threshold set in the OVLVL register
#define ADE7953_IRQENA_WSMP        0x020000      // interrupt when new waveform data is acquired
#define ADE7953_IRQENA_CYCEND      0x040000      // interrupt when it is the end of a line cycle accumulation period
#define ADE7953_IRQENA_SAG         0x080000      // interrupt when a sag event has occurred
#define ADE7953_IRQENA_RESET       0x100000      // This interrupt is always enabled and cannot be disabled
#define ADE7953_IRQENA_ADE_CRC     0x200000      // interrupt when the checksum has changed

// Register Interrupt Status (IRQSTATA) bits
#define ADE7953_IRQSTATA_AEHFA       0x000001     // interrupt when the active energy is half full (Current Channel A)
#define ADE7953_IRQSTATA_VAREHFA     0x000002     // interrupt when the reactive energy is half full (Current Channel A)
#define ADE7953_IRQSTATA_VAEHFA      0x000004     // interrupt when the apparent energy is half full (Current Channel A)
#define ADE7953_IRQSTATA_AEOFA       0x000008     // interrupt when the active energy has overflowed or underflowed (Current Channel A)
#define ADE7953_IRQSTATA_VAREOFA     0x000010      // interrupt when the reactive energy has overflowed or underflowed (Current Channel A)
#define ADE7953_IRQSTATA_VAEOFA      0x000020      // interrupt when the apparent energy has overflowed or underflowed (Current Channel A)
#define ADE7953_IRQSTATA_AP_NOLOADA  0x000040      // interrupt when the active power no-load condition is detected on Current Channel A
#define ADE7953_IRQSTATA_VAR_NOLOADA 0x000080      // interrupt when the reactive power no-load condition is detected on Current Channel A
#define ADE7953_IRQSTATA_VA_NOLOADA  0x000100      // interrupt when the apparent power no-load condition is detected on Current Channel A
#define ADE7953_IRQSTATA_APSIGN_A    0x000200      // interrupt when the sign of active energy has changed (Current Channel A)
#define ADE7953_IRQSTATA_VARSIGN_A   0x000400      // interrupt when the sign of reactive energy has changed (Current Channel A)
#define ADE7953_IRQSTATA_ZXTO_IA     0x000800      // interrupt when the zero crossing has been missing on Current Channel A for the length of time specified in the ZXTOUT register
#define ADE7953_IRQSTATA_ZXIA        0x001000      // interrupt when the current Channel A zero crossing occurs
#define ADE7953_IRQSTATA_OIA         0x002000      // interrupt when the current Channel A peak has exceeded the overcurrent threshold set in the OILVL register
#define ADE7953_IRQSTATA_ZXTO        0x004000      // interrupt when a zero crossing has been missing on the voltage channel for the length of time specified in the ZXTOUT register
#define ADE7953_IRQSTATA_ZXV         0x008000      // interrupt when the voltage channel zero crossing occurs
#define ADE7953_IRQSTATA_OV          0x010000      // interrupt when the voltage peak has exceeded the overvoltage threshold set in the OVLVL register

#ifdef __cplusplus
}
#endif

#endif /* ADE7953_REGS_H */
/** @} */

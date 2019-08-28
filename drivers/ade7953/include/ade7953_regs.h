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
#define SAGCYC_8        0x000   //  Sag line cycles
#define DISNOLOAD_8     0x001   //  No-load detection disable
#define LCYCMODE_8      0x004   //  Line cycle accumulation mode configuration
#define PGA_V_8         0x007   //  Voltage channel gain configuration (Bits[2:0])
#define PGA_IA_8        0x008   //  Current Channel A gain configuration (Bits[2:0])
#define PGA_IB_8        0x009   //  Current Channel B gain configuration (Bits[2:0])
#define WRITE_PROTECT_8 0x040   //  Write protection bits (Bits[2:0])
#define LAST_OP_8       0x0FD   //  Contains the type (read or write) of the last successful communication (0x35 = read; 0xCA = write)
#define UNLOCK_REG_8    0x0FE   //  Addres Unlock register for unlock Register 0x120 address
#define LAST_RWDATA_8   0x0FF   //  Contains the data from the last successful 8-bit register communication
#define VERSION_8       0x702   //  Contains the silicon version number
#define EX_REF_8        0x800   //  Reference input configuration: set to 0 for internal; set to 1 for external

// 16-bits Registers
#define ZXTOUT_16        0x100   //  Zero-crossing timeout
#define LINECYC_16       0x101   //  Number of half line cycles for line cycle energy accumulation mode
#define CONFIG_16        0x102   //  Configuration register
#define CF1DEN_16        0x103   //  CF1 frequency divider denominator. When modifying this register, two sequential write operations must be performed to ensure that the write is successful.
#define CF2DEN_16        0x104   //  CF2 frequency divider denominator. When modifying this register, two sequential write operations must be performed to ensure that the write is successful.
#define CFMODE_16        0x107   //  CF output selection
#define PHCALA_16        0x108   //  Phase calibration register (Current Channel A). This register is in sign magnitude format.
#define PHCALB_16        0x109   //  Phase calibration register (Current Channel B). This register is in sign magnitude format.
#define PFA_16           0x10A   //  Power factor (Current Channel A)
#define PFB_16           0x10B   //  Power factor (Current Channel B)
#define ANGLE_A_16       0x10C   //  Angle between the voltage input and the Current Channel A input
#define ANGLE_B_16       0x10D   //  Angle between the voltage input and the Current Channel B input
#define PERIOD_16        0x10E   //  Period register
#define ALT_OUTPUT_16    0x110   //  Alternative output functions (see Table 20)
#define SETUP_REG_16     0x120   //  This register should be set to 30h
#define LAST_ADD_16      0x1FE   //  Contains the address of the last successful communication
#define LAST_RWDATA_16   0x1FF   //  Contains the data from the last successful 16-bit register communication

// 24-Bit Registers
#define SAGLVL_24      0x200   // Sag voltage level
#define ACCMODE_24     0x201   // Accumulation mode
#define AP_NOLOAD_24   0x203   // Active power no-load level
#define VAR_NOLOAD_24  0x204   // Reactive power no-load level
#define VA_NOLOAD_24   0x205   // Apparent power no-load level
#define AVA_24         0x210   // Instantaneous apparent power (Current Channel A)
#define BVA_24         0x211   // Instantaneous apparent power (Current Channel B)
#define AWATT_24       0x212   // Instantaneous active power (Current Channel A)
#define BWATT_24       0x213   // Instantaneous active power (Current Channel B)
#define AVAR_24        0x214   // Instantaneous reactive power (Current Channel A)
#define BVAR_24        0x215   // Instantaneous reactive power (Current Channel B)
#define IA_24          0x216   // Instantaneous current (Current Channel A)
#define IB_24          0x217   // Instantaneous current (Current Channel B)
#define V_24           0x218   // Instantaneous voltage (voltage channel)
#define IRMSA_24       0x21A   // IRMS register (Current Channel A)
#define IRMSB_24       0x21B   // IRMS register (Current Channel B)
#define VRMS_24        0x21C   // VRMS register
#define AENERGYA_24    0x21E   // Active energy (Current Channel A)
#define AENERGYB_24    0x21F   // Active energy (Current Channel B)
#define RENERGYA_24    0x220   // Reactive energy (Current Channel A)
#define RENERGYB_24    0x221   // Reactive energy (Current Channel B)
#define APENERGYA_24   0x222   // Apparent energy (Current Channel A)
#define APENERGYB_24   0x223   // Apparent energy (Current Channel B)
#define OVLVL_24       0x224   // Overvoltage level
#define OILVL_24       0x225   // Overcurrent level
#define VPEAK_24       0x226   // Voltage channel peak
#define RSTVPEAK_24    0x227   // Read voltage peak with reset
#define IAPEAK_24      0x228   // Current Channel A peak
#define RSTIAPEAK_24   0x229   // Read Current Channel A peak with reset
#define IBPEAK_24      0x22A   // Current Channel B peak
#define RSTIBPEAK_24   0x22B   // Read Current Channel B peak with reset
#define IRQENA_24      0x22C   // Interrupt enable (Current Channel A)
#define IRQSTATA_24    0x22D   // Interrupt status (Current Channel A)
#define RSTIRQSTATA_24 0x22E   // Reset interrupt status (Current Channel A)
#define IRQENB_24      0x22F   // Interrupt enable (Current Channel B)
#define IRQSTATB_24    0x230   // Interrupt status (Current Channel B)
#define RSTIRQSTATB_24 0x231   // Reset interrupt status (Current Channel B)
#define AIGAIN_24      0x280   // Current channel gain (Current Channel A)
#define AVGAIN_24      0x281   // Voltage channel gain
#define AWGAIN_24      0x282   // Active power gain (Current Channel A)
#define AVARGAIN_24    0x283   // Reactive power gain (Current Channel A)
#define AVAGAIN_24     0x284   // Apparent power gain (Current Channel A)
#define AIRMSOS_24     0x286   // IRMS offset (Current Channel A)
#define VRMSOS_24      0x288   // VRMS offset
#define AWATTOS_24     0x289   // Active power offset correction (Current Channel A)
#define AVAROS_24      0x28A   // Reactive power offset correction (Current Channel A)
#define AVAOS_24       0x28B   // Apparent power offset correction (Current Channel A)ADE7953
#define LAST_RWDATA_24 0x2FF   //  Contains the data from the last successful 24-bit register communication

// 32-Bit Registers
#define SAGLVL_32      0x300   // Sag voltage level
#define ACCMODE_32     0x301   // Accumulation mod
#define AP_NOLOAD_32   0x303   // Active power no-load level
#define VAR_NOLOAD_32  0x304   // Reactive power no-load level
#define VA_NOLOAD_32   0x305   // Apparent power no-load level
#define AVA_32         0x310   // Instantaneous apparent power (Current Channel A)
#define BVA_32         0x311   // Instantaneous apparent power (Current Channel B)
#define AWATT_32       0x312   // Instantaneous active power (Current Channel A)
#define BWATT_32       0x313   // Instantaneous active power (Current Channel B)
#define AVAR_32        0x314   // Instantaneous reactive power (Current Channel A)
#define BVAR_32        0x315   // Instantaneous reactive power (Current Channel B)
#define IA_32          0x316   // Instantaneous current (Current Channel A)
#define IB_32          0x317   // Instantaneous current (Current Channel B)
#define V_32           0x318   // Instantaneous voltage (voltage channel)
#define IRMSA_32       0x31A   // IRMS register (Current Channel A)
#define IRMSB_32       0x31B   // IRMS register (Current Channel B)
#define VRMS_32        0x31C   // VRMS register
#define AENERGYA_32    0x31E   // Active energy (Current Channel A)
#define AENERGYB_32    0x31F   // Active energy (Current Channel B)
#define RENERGYA_32    0x320   // Reactive energy (Current Channel A)
#define RENERGYB_32    0x321   // Reactive energy (Current Channel B)
#define APENERGYA_32   0x322   // Apparent energy (Current Channel A)
#define APENERGYB_32   0x323   // Apparent energy (Current Channel B)
#define OVLVL_32       0x324   // Overvoltage level
#define OILVL_32       0x325   // Overcurrent level
#define VPEAK_32       0x326   // Voltage channel peak
#define RSTVPEAK_32    0x327   // Read voltage peak with reset
#define IAPEAK_32      0x328   // Current Channel A peak
#define RSTIAPEAK_32   0x329   // Read Current Channel A peak with reset
#define IBPEAK_32      0x32A   // Current Channel B peak
#define RSTIBPEAK_32   0x32B   // Read Current Channel B peak with reset
#define IRQENA_32      0x32C   // Interrupt enable (Current Channel A)
#define IRQSTATA_32    0x32D   // Interrupt status (Current Channel A)
#define RSTIRQSTATA_32 0x32E   // Reset interrupt status (Current Channel A)
#define IRQENB_32      0x32F   // Interrupt enable (Current Channel B)
#define IRQSTATB_32    0x330   // Interrupt status (Current Channel B)
#define RSTIRQSTATB_32 0x331   // Reset interrupt status (Current Channel B)
#define AIGAIN_32      0x380   // Current channel gain (Current Channel A)
#define AVGAIN_32      0x381   // Voltage channel gain
#define AWGAIN_32      0x382   // Active power gain (Current Channel A)
#define AVARGAIN_32    0x383   // Reactive power gain (Current Channel A)
#define AVAGAIN_32     0x384   // Apparent power gain (Current Channel A)
#define AIRMSOS_32     0x386   // IRMS offset (Current Channel A)
#define VRMSOS_32      0x388   // VRMS offset
#define AWATTOS_32     0x389   // Active power offset correction (Current Channel A)
#define AVAROS_32      0x38A   // Reactive power offset correction (Current Channel A)
#define AVAOS_32       0x38B   // Apparent power offset correction (Current Channel A)ADE7953
#define LAST_RWDATA_32 0x3FF   //  Contains the data from the last successful 24-bit register communication

// Register Interrupt enable (IRQENA) bits
#define IRQENA_AEHFA       0x000001     // interrupt when the active energy is half full (Current Channel A)
#define IRQENA_VAREHFA     0x000002     // interrupt when the reactive energy is half full (Current Channel A)
#define IRQENA_VAEHFA      0x000004     // interrupt when the apparent energy is half full (Current Channel A)
#define IRQENA_AEOFA       0x000008     // interrupt when the active energy has overflowed or underflowed (Current Channel A)
#define IRQENA_VAREOFA     0x000010      // interrupt when the reactive energy has overflowed or underflowed (Current Channel A)
#define IRQENA_VAEOFA      0x000020      // interrupt when the apparent energy has overflowed or underflowed (Current Channel A)
#define IRQENA_AP_NOLOADA  0x000040      // interrupt when the active power no-load condition is detected on Current Channel A
#define IRQENA_VAR_NOLOADA 0x000080      // interrupt when the reactive power no-load condition is detected on Current Channel A
#define IRQENA_VA_NOLOADA  0x000100      // interrupt when the apparent power no-load condition is detected on Current Channel A
#define IRQENA_APSIGN_A    0x000200      // interrupt when the sign of active energy has changed (Current Channel A)
#define IRQENA_VARSIGN_A   0x000400      // interrupt when the sign of reactive energy has changed (Current Channel A)
#define IRQENA_ZXTO_IA     0x000800      // interrupt when the zero crossing has been missing on Current Channel A for the length of time specified in the ZXTOUT register
#define IRQENA_ZXIA        0x001000      // interrupt when the current Channel A zero crossing occurs
#define IRQENA_OIA         0x002000      // interrupt when the current Channel A peak has exceeded the overcurrent threshold set in the OILVL register
#define IRQENA_ZXTO        0x004000      // interrupt when a zero crossing has been missing on the voltage channel for the length of time specified in the ZXTOUT register
#define IRQENA_ZXV         0x008000      // interrupt when the voltage channel zero crossing occurs
#define IRQENA_OV          0x010000      // interrupt when the voltage peak has exceeded the overvoltage threshold set in the OVLVL register
#define IRQENA_WSMP        0x020000      // interrupt when new waveform data is acquired
#define IRQENA_CYCEND      0x040000      // interrupt when it is the end of a line cycle accumulation period
#define IRQENA_SAG         0x080000      // interrupt when a sag event has occurred
#define IRQENA_RESET       0x100000      // This interrupt is always enabled and cannot be disabled
#define IRQENA_ADE_CRC     0x200000      // interrupt when the checksum has changed

// Register Interrupt Status (IRQSTATA) bits
#define IRQSTATA_AEHFA       0x000001     // interrupt when the active energy is half full (Current Channel A)
#define IRQSTATA_VAREHFA     0x000002     // interrupt when the reactive energy is half full (Current Channel A)
#define IRQSTATA_VAEHFA      0x000004     // interrupt when the apparent energy is half full (Current Channel A)
#define IRQSTATA_AEOFA       0x000008     // interrupt when the active energy has overflowed or underflowed (Current Channel A)
#define IRQSTATA_VAREOFA     0x000010      // interrupt when the reactive energy has overflowed or underflowed (Current Channel A)
#define IRQSTATA_VAEOFA      0x000020      // interrupt when the apparent energy has overflowed or underflowed (Current Channel A)
#define IRQSTATA_AP_NOLOADA  0x000040      // interrupt when the active power no-load condition is detected on Current Channel A
#define IRQSTATA_VAR_NOLOADA 0x000080      // interrupt when the reactive power no-load condition is detected on Current Channel A
#define IRQSTATA_VA_NOLOADA  0x000100      // interrupt when the apparent power no-load condition is detected on Current Channel A
#define IRQSTATA_APSIGN_A    0x000200      // interrupt when the sign of active energy has changed (Current Channel A)
#define IRQSTATA_VARSIGN_A   0x000400      // interrupt when the sign of reactive energy has changed (Current Channel A)
#define IRQSTATA_ZXTO_IA     0x000800      // interrupt when the zero crossing has been missing on Current Channel A for the length of time specified in the ZXTOUT register
#define IRQSTATA_ZXIA        0x001000      // interrupt when the current Channel A zero crossing occurs
#define IRQSTATA_OIA         0x002000      // interrupt when the current Channel A peak has exceeded the overcurrent threshold set in the OILVL register
#define IRQSTATA_ZXTO        0x004000      // interrupt when a zero crossing has been missing on the voltage channel for the length of time specified in the ZXTOUT register
#define IRQSTATA_ZXV         0x008000      // interrupt when the voltage channel zero crossing occurs
#define IRQSTATA_OV          0x010000      // interrupt when the voltage peak has exceeded the overvoltage threshold set in the OVLVL register

#ifdef __cplusplus
}
#endif

#endif /* ADE7953_REGS_H */
/** @} */

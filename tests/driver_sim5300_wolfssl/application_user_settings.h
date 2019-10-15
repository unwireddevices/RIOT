/*
 * example
 */
#ifndef APPLICATION_USER_SETTINGS_H
#define APPLICATION_USER_SETTINGS_H


#ifdef __cplusplus
extern "C" {
#endif

#include <time.h>

extern time_t get_epoch_time(struct tm *time);

#undef WOLFSSL_RIOT_OS
#define WOLFSSL_RIOT_OS

#undef NO_MAIN_DRIVER
#define NO_MAIN_DRIVER

#undef HAVE_ECC
#define HAVE_ECC

#undef TFM_TIMING_RESISTANT
#define TFM_TIMING_RESISTANT

#undef ECC_TIMING_RESISTANT
#define ECC_TIMING_RESISTANT

#undef WC_RSA_BLINDING
#define WC_RSA_BLINDING

#undef SINGLE_THREADED
#define SINGLE_THREADED

#undef NO_FILESYSTEM
#define NO_FILESYSTEM

#undef USE_CERT_BUFFERS_2048
#define USE_CERT_BUFFERS_2048

#undef WOLFSSL_HAVE_SP_RSA
#define WOLFSSL_HAVE_SP_RSA

#undef NO_ASN_TIME
#define NO_ASN_TIME

#define XTIME get_epoch_time

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_USER_SETTINGS_H */

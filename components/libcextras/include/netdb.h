#pragma once

#include_next "netdb.h"

#ifndef NI_NUMERICHOST
#define NI_NUMERICHOST 0x1
#endif

#ifndef IFF_UP
#define IFF_UP 0x1
#endif

#ifndef IFF_LOOPBACK
#define IFF_LOOPBACK 0x8
#endif

#ifndef NI_NUMERICSERV
#define NI_NUMERICSERV 0x8
#endif

#ifndef NI_DGRAM
#define NI_DGRAM 0x00000010
#endif

#ifndef EAI_BADFLAGS
#define EAI_BADFLAGS 3
#endif

#ifndef AF_UNIX
#define AF_UNIX 1
#endif

#ifdef __cplusplus
extern "C" {
#endif

const char *gai_strerror(int ecode);

#ifdef __cplusplus
}
#endif

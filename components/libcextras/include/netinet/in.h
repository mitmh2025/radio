#pragma once

#include_next <netinet/in.h>

// Taken from libc

#define IN6_IS_ADDR_UNSPECIFIED(a)                                             \
  (((const uint32_t *)(a))[0] == 0 && ((const uint32_t *)(a))[1] == 0 &&       \
   ((const uint32_t *)(a))[2] == 0 && ((const uint32_t *)(a))[3] == 0)

#define IN6_IS_ADDR_LOOPBACK(a)                                                \
  (((const uint32_t *)(a))[0] == 0 && ((const uint32_t *)(a))[1] == 0 &&       \
   ((const uint32_t *)(a))[2] == 0 && ((const uint32_t *)(a))[3] == htonl(1))

#define IN6_IS_ADDR_LINKLOCAL(a)                                               \
  ((((const uint32_t *)(a))[0] & htonl(0xffc00000)) == htonl(0xfe800000))

#define IN6_IS_ADDR_SITELOCAL(a)                                               \
  ((((const uint32_t *)(a))[0] & htonl(0xffc00000)) == htonl(0xfec00000))

#define IN6_IS_ADDR_V4MAPPED(a)                                                \
  ((((const uint32_t *)(a))[0] == 0) && (((const uint32_t *)(a))[1] == 0) &&   \
   (((const uint32_t *)(a))[2] == htonl(0xffff)))

#define IN6_IS_ADDR_V4COMPAT(a)                                                \
  ((((const uint32_t *)(a))[0] == 0) && (((const uint32_t *)(a))[1] == 0) &&   \
   (((const uint32_t *)(a))[2] == 0) &&                                        \
   (ntohl(((const uint32_t *)(a))[3]) > 1))

#define IN6_ARE_ADDR_EQUAL(a, b)                                               \
  ((((const uint32_t *)(a))[0] == ((const uint32_t *)(b))[0]) &&               \
   (((const uint32_t *)(a))[1] == ((const uint32_t *)(b))[1]) &&               \
   (((const uint32_t *)(a))[2] == ((const uint32_t *)(b))[2]) &&               \
   (((const uint32_t *)(a))[3] == ((const uint32_t *)(b))[3]))

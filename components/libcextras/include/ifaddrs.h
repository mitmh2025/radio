#pragma once

#include <sys/socket.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

// These should come from <net/if.h> but lwip doesn't have them
#define IFF_UP 0x1           /* interface is up */
#define IFF_BROADCAST 0x2    /* broadcast address valid */
#define IFF_LOOPBACK 0x8     /* is a loopback net */
#define IFF_POINTOPOINT 0x10 /* interface is point-to-point link */
#define IFF_RUNNING 0x40     /* resources allocated */

// This doesn't need to be complete; just good enough for kvswebrtc, which
// only looks at a handful of fields: ifa_addr, ifa_name, and ifa_flags (and
// there only IFF_LOOPBACK and IFF_RUNNING, which are always false and true,
// respectively)
struct ifaddrs {
  struct ifaddrs *ifa_next;  /* Pointer to next struct */
  char *ifa_name;            /* Interface name */
  u_int ifa_flags;           /* Interface flags */
  struct sockaddr *ifa_addr; /* Interface address */
};

int getifaddrs(struct ifaddrs **ifap);

void freeifaddrs(struct ifaddrs *ifp);

#ifdef __cplusplus
}
#endif

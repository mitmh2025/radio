#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define _UTSNAME_LENGTH 64

struct utsname {
  char sysname[_UTSNAME_LENGTH];  /* [XSI] Name of OS */
  char nodename[_UTSNAME_LENGTH]; /* [XSI] Name of this network node */
  char release[_UTSNAME_LENGTH];  /* [XSI] Release level */
  char version[_UTSNAME_LENGTH];  /* [XSI] Version level */
  char machine[_UTSNAME_LENGTH];  /* [XSI] Hardware type */
};

int uname(struct utsname *);

#ifdef __cplusplus
}
#endif

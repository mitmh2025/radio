#pragma once

#include_next "netdb.h"

#ifdef __cplusplus
extern "C" {
#endif

const char *gai_strerror(int ecode);

#ifdef __cplusplus
}
#endif

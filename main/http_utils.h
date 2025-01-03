#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t http_utils_percent_decode(const char *src, size_t src_len, char *dst,
                                    size_t dst_len);

#ifdef __cplusplus
}
#endif

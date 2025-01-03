#include "http_utils.h"

esp_err_t http_utils_percent_decode(const char *src, size_t src_len, char *dst,
                                    size_t dst_len) {
  size_t pos = 0;
  for (size_t i = 0; i < src_len; i++) {
    if (dst_len <= pos) {
      return ESP_ERR_NO_MEM;
    }

    if (src[i] == '+') {
      dst[pos++] = ' ';
      continue;
    }

    if (src[i] != '%') {
      dst[pos++] = src[i];
      continue;
    }

    if (i + 2 >= src_len) {
      return ESP_ERR_INVALID_ARG;
    }

    char hex[3] = {src[i + 1], src[i + 2], 0};
    dst[pos++] = (char)strtol(hex, NULL, 16);
    i += 2;
  }

  return ESP_OK;
}

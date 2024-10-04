#include "netdb.h"

const char *gai_strerror(int ecode) {
  // Error strings taken from macOS
  switch (ecode) {
  case EAI_NONAME:
    return "nodename nor servname provided, or not known";
  case EAI_SERVICE:
    return "servname not supported for ai_socktype";
  case EAI_FAIL:
    return "Non-recoverable failure in name resolution";
  case EAI_MEMORY:
    return "Memory allocation failure";
  case EAI_FAMILY:
    return "ai_family not supported";
  default:
    return "Unknown error";
  }
}

#include <string.h>
#include <sys/errno.h>
#include <sys/utsname.h>

#include "esp_idf_version.h"
#include "esp_netif.h"

int uname(struct utsname *name) {
  const char *hostname = "unknown";
  esp_netif_t *netif = esp_netif_get_default_netif();
  if (netif) {
    esp_netif_get_hostname(netif, &hostname);
  }
  strlcpy(name->sysname, "ESP32", _UTSNAME_LENGTH);
  strlcpy(name->release, esp_get_idf_version(), _UTSNAME_LENGTH);
  strlcpy(name->version, esp_get_idf_version(), _UTSNAME_LENGTH);
  strlcpy(name->machine, "xtensa", _UTSNAME_LENGTH);
  strlcpy(name->nodename, hostname, _UTSNAME_LENGTH);
  return -1;
}

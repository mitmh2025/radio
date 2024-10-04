#include "ifaddrs.h"
#include "errno.h"
#include "esp_err.h"
#include "esp_netif.h"

static esp_err_t _getifaddrs_cb(void *ctx) {
  esp_err_t err;
  struct ifaddrs **ifap = (struct ifaddrs **)ctx;
  esp_netif_t *netif = NULL;
  while ((netif = esp_netif_next_unsafe(netif)) != NULL) {
    struct ifaddrs *ifa = calloc(1, sizeof(struct ifaddrs));
    if (ifa == NULL) {
      freeifaddrs(*ifap);
      return ESP_ERR_NO_MEM;
    }

    ifa->ifa_next = *ifap;
    *ifap = ifa;

    esp_netif_ip_info_t ip_info;
    err = esp_netif_get_ip_info(netif, &ip_info);
    if (err != ESP_OK) {
      freeifaddrs(*ifap);
      return err;
    }

    struct sockaddr_in *addr = calloc(1, sizeof(struct sockaddr_in));
    if (!addr) {
      freeifaddrs(*ifap);
      return ESP_ERR_NO_MEM;
    }
    addr->sin_family = AF_INET;
    addr->sin_addr.s_addr = ip_info.ip.addr;
    ifa->ifa_addr = (struct sockaddr *)addr;

    ifa->ifa_name = calloc(1, NETIF_NAMESIZE);
    if (!ifa->ifa_name) {
      freeifaddrs(*ifap);
      return ESP_ERR_NO_MEM;
    }

    err = esp_netif_get_netif_impl_name(netif, ifa->ifa_name);
    if (err != ESP_OK) {
      freeifaddrs(*ifap);
      return err;
    }

    ifa->ifa_flags = esp_netif_is_netif_up(netif) ? IFF_UP | IFF_RUNNING : 0;
  }

  return ESP_OK;
}

int getifaddrs(struct ifaddrs **ifap) {
  esp_err_t err = esp_netif_tcpip_exec(_getifaddrs_cb, ifap);
  if (err != ESP_OK) {
    errno = ENOSYS;
    return -1;
  }
  return 0;
}

void freeifaddrs(struct ifaddrs *ifp) {
  while (ifp) {
    struct ifaddrs *next = ifp->ifa_next;
    if (ifp->ifa_addr) {
      free(ifp->ifa_addr);
    }
    if (ifp->ifa_name) {
      free(ifp->ifa_name);
    }
    free(ifp);
    ifp = next;
  }
}

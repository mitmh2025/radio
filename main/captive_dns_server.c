#include "captive_dns_server.h"
#include "main.h"

#include <net/if.h>
#include <stdatomic.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"

enum {
  DNS_TYPE_A = 1,
  DNS_TYPE_ANY = 255,
};

enum {
  DNS_CLASS_IN = 1,
};

struct __attribute__((packed)) dns_header {
  uint16_t id;
  union {
    struct {
      uint16_t rd : 1;     // recursion desired
      uint16_t tc : 1;     // truncated message
      uint16_t aa : 1;     // authoritative answer
      uint16_t opcode : 4; // message_type
      uint16_t qr : 1;     // query/response flag
      uint16_t rcode : 4;  // response code
      uint16_t z : 3;      // its z! reserved
      uint16_t ra : 1;     // recursion available
    };
    uint16_t flags;
  };
  uint16_t qdcount;
  uint16_t ancount;
  uint16_t nscount;
  uint16_t arcount;
};

struct __attribute__((packed)) dns_question_footer {
  uint16_t qtype;
  uint16_t qclass;
};

struct __attribute__((packed)) dns_answer_footer {
  uint16_t type;
  uint16_t class;
  uint32_t ttl;
  uint16_t rdlength;
};

static atomic_int sock = -1;

static int send_error(int s, uint16_t id, struct sockaddr_in *addr,
                      uint8_t rcode) {
  struct dns_header header = {
      .id = id,
      .qr = 1,
      .rcode = rcode,
  };

  return sendto(s, &header, sizeof(header), 0, (struct sockaddr *)addr,
                sizeof(*addr));
}

ssize_t dns_name_len(const char *name, size_t len) {
  size_t remaining = len;
  len = 0;
  while (remaining > 0) {
    if (*name >= 0xc0) {
      return len + 2;
    } else if (*name == 0) {
      return len + 1;
    } else {
      len += *name + 1;
      name += *name + 1;
      remaining -= *name + 1;
    }
  }

  return -1;
}

static void dns_server_task(void *arg) {
  int s = (int)arg;

  while (true) {
    // TODO: implement DNS server
    char packet[512] = {};
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    ssize_t len = recvfrom(s, packet, sizeof(packet), 0,
                           (struct sockaddr *)&addr, &addr_len);

    if (len < 0) {
      if (errno != ENOTCONN) {
        ESP_LOGE(RADIO_TAG, "Failed to receive DNS request: %d", errno);
      }
      break;
    }

    if (len < sizeof(struct dns_header)) {
      ESP_LOGE(RADIO_TAG, "Invalid DNS request: too short");
      continue;
    }

    char *buf = packet;

    struct dns_header *header = (struct dns_header *)buf;
    if (header->qr != 0) {
      // Ignore non-queries
      continue;
    }

    if (header->tc != 0) {
      send_error(s, header->id, &addr, 4); // Not Implemented
      continue;
    }

    if (ntohs(header->qdcount) != 1 || header->ancount != 0 ||
        header->nscount != 0 || header->arcount != 0 || header->opcode != 0) {
      send_error(s, header->id, &addr, 4); // Not Implemented
      continue;
    }

    buf += sizeof(struct dns_header);
    len -= sizeof(struct dns_header);

    char *name = buf;
    ssize_t name_len = dns_name_len(buf, len);
    if (name_len < 0) {
      send_error(s, header->id, &addr, 1); // Format Error
    }

    buf += name_len;
    len -= name_len;
    if (len < sizeof(struct dns_question_footer)) {
      send_error(s, header->id, &addr, 1); // Format Error
    }

    struct dns_question_footer *question_footer =
        (struct dns_question_footer *)buf;
    uint16_t qclass = ntohs(question_footer->qclass);
    uint16_t qtype = ntohs(question_footer->qtype);

    if (qclass != DNS_CLASS_IN) {
      send_error(s, header->id, &addr, 3); // Does Not Exist
      continue;
    }

    // We support A and ANY queries
    if (qtype != DNS_TYPE_A && qtype != DNS_TYPE_ANY) {
      send_error(s, header->id, &addr, 3); // Does Not Exist
      continue;
    }

    // OK we can build a reply now
    size_t reply_len = sizeof(struct dns_header) +
                       (name_len + sizeof(struct dns_question_footer)) +
                       (name_len + sizeof(struct dns_answer_footer) + 4);
    char reply[reply_len];
    memset(reply, 0, reply_len);
    struct dns_header *reply_header = (struct dns_header *)reply;
    reply_header->id = header->id;
    reply_header->qr = 1;
    reply_header->aa = 1;
    reply_header->qr = 1;
    reply_header->rd = 1;
    reply_header->ra = 1;
    reply_header->qdcount = htons(1);
    reply_header->ancount = htons(1);
    char *question_name = reply + sizeof(struct dns_header);
    memcpy(question_name, name, name_len);
    struct dns_question_footer *reply_question_footer =
        (struct dns_question_footer *)(question_name + name_len);
    memcpy(reply_question_footer, question_footer,
           sizeof(struct dns_question_footer));
    char *answer_name =
        ((char *)reply_question_footer) + sizeof(*reply_question_footer);
    memcpy(answer_name, name, name_len);
    struct dns_answer_footer *reply_answer_footer =
        (struct dns_answer_footer *)(answer_name + name_len);
    reply_answer_footer->class = htons(DNS_CLASS_IN);
    reply_answer_footer->type = htons(DNS_TYPE_A);
    reply_answer_footer->ttl = htonl(60);
    reply_answer_footer->rdlength = htons(4);

    esp_netif_ip_info_t info;
    esp_err_t err = esp_netif_get_ip_info(esp_netif_get_default_netif(), &info);
    if (err != ESP_OK) {
      ESP_LOGE(RADIO_TAG, "Failed to get IP info: %d", err);
      send_error(s, header->id, &addr, 2); // Server Failure
    }

    uint8_t *rdata =
        ((uint8_t *)reply_answer_footer) + sizeof(*reply_answer_footer);
    rdata[0] = ip4_addr1(&info.ip);
    rdata[1] = ip4_addr2(&info.ip);
    rdata[2] = ip4_addr3(&info.ip);
    rdata[3] = ip4_addr4(&info.ip);

    ssize_t ret =
        sendto(s, reply, reply_len, 0, (struct sockaddr *)&addr, addr_len);
    if (ret < 0) {
      ESP_LOGE(RADIO_TAG, "Failed to send DNS reply: %d", errno);
    }
  }

  vTaskDelete(NULL);
}

static void swap_socket(int s) {
  s = atomic_exchange(&sock, s);
  if (s >= 0) {
    close(s);
  }
}

esp_err_t captive_dns_server_start() {
  ESP_LOGI(RADIO_TAG, "Starting DNS server");
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  ESP_RETURN_ON_FALSE(s >= 0, ESP_FAIL, RADIO_TAG,
                      "Failed to create socket: %d", errno);

  struct sockaddr_in addr = {
      .sin_family = AF_INET,
      .sin_port = htons(53),
      .sin_addr =
          {
              .s_addr = htonl(INADDR_ANY),
          },
  };

  int err = bind(s, (struct sockaddr *)&addr, sizeof(addr));
  ESP_RETURN_ON_FALSE(err == 0, ESP_FAIL, RADIO_TAG,
                      "Failed to bind socket: %d", errno);

  swap_socket(s);

  ESP_RETURN_ON_FALSE(pdPASS == xTaskCreatePinnedToCore(dns_server_task,
                                                        "dns_server", 4096,
                                                        (void *)s, 18, NULL, 0),
                      ESP_FAIL, RADIO_TAG, "Failed to create DNS server task");

  return ESP_OK;
}

esp_err_t captive_dns_server_stop() {
  ESP_LOGI(RADIO_TAG, "Stopping DNS server");
  swap_socket(-1);
  return ESP_OK;
}

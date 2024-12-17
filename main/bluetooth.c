#include "bluetooth.h"
#include "main.h"
#include "things.h"

#include <math.h>
#include <stdint.h>
#include <sys/queue.h>

#include "esp_bt.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"

#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

struct bt_beacon {
  bluetooth_beacon_t beacon;
  TAILQ_ENTRY(bt_beacon) entries;
};

TAILQ_HEAD(bt_beacon_list, bt_beacon);

struct bt_subscriber {
  uint16_t major;
  bluetooth_beacon_callback_t callback;
  void *arg;
  TAILQ_ENTRY(bt_subscriber) entries;
};

TAILQ_HEAD(bt_subscriber_list, bt_subscriber);

// B7F8C378-248E-40E0-9157-8B517DA07783
static const ble_uuid128_t radio_uuid =
    BLE_UUID128_INIT(0xb7, 0xf8, 0xc3, 0x78, 0x24, 0x8e, 0x40, 0xe0, 0x91, 0x57,
                     0x8b, 0x51, 0x7d, 0xa0, 0x77, 0x83, );

const uint16_t BLUETOOTH_MAJOR_RADIO = 0x00001;
const uint16_t BLUETOOTH_MAJOR_RICKROLL = 0x0002;
const uint16_t BLUETOOTH_MAJOR_FUNAROUND = 0x0003;

static bool synced = false;

static SemaphoreHandle_t mutex = NULL;
static bluetooth_mode_t current_mode = BLUETOOTH_MODE_DEFAULT;
static uint16_t adv_major = UINT16_MAX;
static uint16_t adv_minor = UINT16_MAX;

static SemaphoreHandle_t beacon_mutex = NULL;
static struct bt_beacon_list beacons = TAILQ_HEAD_INITIALIZER(beacons);
static struct bt_subscriber_list subscribers =
    TAILQ_HEAD_INITIALIZER(subscribers);

static esp_timer_handle_t beacon_cleanup_timer = NULL;

static uint8_t mac[6] = {0};

static void telemetry_generator() {
  things_send_telemetry_string("bluetooth_mode", (const char *[]){
                                                     "default",
                                                     "disabled",
                                                     "aggressive",
                                                 }[current_mode]);

  // Generate a JSON string with all beacons that we can currently see in the
  // form:
  // [{"major": 1, "minor": 2, "rssi": -50, "txPower": -60, "lastSeen":
  // 1234567890}, ...]

  size_t beacon_count = 0;
  xSemaphoreTake(beacon_mutex, portMAX_DELAY);
  struct bt_beacon *beacon;
  TAILQ_FOREACH(beacon, &beacons, entries) { beacon_count++; }

  size_t json_len =
      2 /* [] */ +
      beacon_count *
          (strlen("{\"major\": 65535, \"minor\": 65535, \"rssi\": -128, "
                  "\"txPower\": -128, \"lastSeen\": 9223372036854775808},")) +
      1;
  char *json = malloc(json_len);
  if (!json) {
    ESP_LOGE(RADIO_TAG, "Failed to allocate memory for telemetry JSON");
    xSemaphoreGive(beacon_mutex);
    return;
  }

  char *json_ptr = json;
  size_t json_remaining = json_len;
  json_ptr += snprintf(json_ptr, json_remaining, "[");
  json_remaining -= 1;
  TAILQ_FOREACH(beacon, &beacons, entries) {
    json_ptr += snprintf(
        json_ptr, json_remaining,
        "{\"major\": %" PRIu16 ", \"minor\": %" PRIu16
        ", \"rssi\": %d, \"txPower\": %d, \"lastSeen\": %" PRId64 "}",
        beacon->beacon.major, beacon->beacon.minor, beacon->beacon.rssi,
        beacon->beacon.tx_power, beacon->beacon.last_seen);
    json_remaining -= strlen(json_ptr);
    if (TAILQ_NEXT(beacon, entries)) {
      json_ptr += snprintf(json_ptr, json_remaining, ",");
      json_remaining -= 1;
    }
  }
  json_ptr += snprintf(json_ptr, json_remaining, "]");
  xSemaphoreGive(beacon_mutex);

  things_send_telemetry_string("bluetooth_beacons", json);

  free(json);
}

// Must call while holding beacon_mutex
static void notify_subscriber(struct bt_subscriber *sub,
                              struct bt_beacon *newest) {
  struct bt_beacon *strongest = NULL;
  struct bt_beacon *beacon;
  TAILQ_FOREACH(beacon, &beacons, entries) {
    if (beacon->beacon.major == sub->major) {
      if (!strongest || beacon->beacon.rssi > strongest->beacon.rssi) {
        strongest = beacon;
      }
    }
  }
  sub->callback(&newest->beacon, &strongest->beacon, sub->arg);
}

// Must call while holding beacon_mutex
static void notify_subscribers(struct bt_beacon *newest) {
  struct bt_subscriber *sub;
  TAILQ_FOREACH(sub, &subscribers, entries) { notify_subscriber(sub, newest); }
}

static void beacon_cleanup(void *arg) {
  // Remove any beacon we haven't seen in the last 30 seconds
  int64_t now = esp_timer_get_time();
  xSemaphoreTake(beacon_mutex, portMAX_DELAY);
  struct bt_beacon *beacon, *tmp;
  TAILQ_FOREACH_SAFE(beacon, &beacons, entries, tmp) {
    if (now - beacon->beacon.last_seen > 30 * 1000 * 1000) {
      ESP_LOGD(RADIO_TAG, "Removing beacon: major=%" PRIu16 ", minor=%" PRIu16,
               beacon->beacon.major, beacon->beacon.minor);
      beacon->beacon.last_seen = 0;
      beacon->beacon.period = 0;
      beacon->beacon.rssi = INT8_MIN;
      notify_subscribers(beacon);
      TAILQ_REMOVE(&beacons, beacon, entries);
      free(beacon);
    }
  }
  xSemaphoreGive(beacon_mutex);
}

static void advertise() {
  if (!synced) {
    return;
  }

  int rc = ble_gap_adv_stop();
  if (rc != 0 && rc != BLE_HS_EALREADY) {
    ESP_LOGE(RADIO_TAG, "Failed to stop advertising: %d", rc);
    return;
  }

  if (current_mode == BLUETOOTH_MODE_DISABLED) {
    return;
  }

  if (adv_major == UINT16_MAX && adv_minor == UINT16_MAX) {
    return;
  }

  // TODO: figure out if there's a better setting for tx power
  rc = ble_ibeacon_set_adv_data((void *)&radio_uuid.value, adv_major, adv_minor,
                                -30);
  if (rc != 0) {
    ESP_LOGE(RADIO_TAG, "Failed to set iBeacon advertisement data: %d", rc);
    return;
  }

  struct ble_gap_adv_params adv_params = {};
  if (current_mode == BLUETOOTH_MODE_AGGRESSIVE) {
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(30);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(60);
  }
  rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params,
                         NULL, NULL);
  if (rc != 0) {
    ESP_LOGE(RADIO_TAG, "Failed to start advertising: %d", rc);
  }
}

static int gap_event(struct ble_gap_event *event, void *arg) {
  switch (event->type) {
  case BLE_GAP_EVENT_DISC: {
    struct ble_hs_adv_fields parsed_fields;
    int ret = ble_hs_adv_parse_fields(&parsed_fields, event->disc.data,
                                      event->disc.length_data);
    if (ret != 0) {
      ESP_LOGD(RADIO_TAG, "Failed to parse advertisement data: %d", ret);
      return 0;
    }

    // 25 bytes is the size of an iBeacon advertisement
    if (parsed_fields.mfg_data_len != 25) {
      return 0;
    }

    if (memcmp(parsed_fields.mfg_data, "\x4c\x00\x02\x15", 4) != 0) {
      return 0;
    }

    ble_uuid_any_t beacon_uuid;
    ble_uuid_init_from_buf(&beacon_uuid, parsed_fields.mfg_data + 4, 16);

    if (ble_uuid_cmp(&beacon_uuid.u, &radio_uuid.u) != 0) {
      return 0;
    }

    uint16_t major = get_be16(parsed_fields.mfg_data + 20);
    uint16_t minor = get_be16(parsed_fields.mfg_data + 22);
    int8_t tx_power = (int8_t)parsed_fields.mfg_data[24];

    xSemaphoreTake(beacon_mutex, portMAX_DELAY);
    struct bt_beacon *beacon = NULL;
    TAILQ_FOREACH(beacon, &beacons, entries) {
      if (beacon->beacon.major == major && beacon->beacon.minor == minor) {
        break;
      }
    }
    if (!beacon) {
      ESP_LOGD(RADIO_TAG,
               "Found radio beacon: major=%" PRIu16 ", minor=%" PRIu16
               " (rssi=%d, txPower=%d)",
               major, minor, event->disc.rssi, tx_power);

      beacon = calloc(1, sizeof(struct bt_beacon));
      if (!beacon) {
        ESP_LOGE(RADIO_TAG, "Failed to allocate memory for beacon");
        xSemaphoreGive(beacon_mutex);
        return 0;
      }
      beacon->beacon.major = major;
      beacon->beacon.minor = minor;
      beacon->beacon.rssi = event->disc.rssi;
      TAILQ_INSERT_TAIL(&beacons, beacon, entries);
    }

    beacon->beacon.rssi = (beacon->beacon.rssi + event->disc.rssi) / 2;
    beacon->beacon.tx_power = tx_power;
    int64_t now = esp_timer_get_time();
    if (beacon->beacon.period == 0 && beacon->beacon.last_seen != 0) {
      beacon->beacon.period = now - beacon->beacon.last_seen;
    } else {
      beacon->beacon.period =
          (beacon->beacon.period + now - beacon->beacon.last_seen) / 2;
    }
    beacon->beacon.last_seen = now;

    notify_subscribers(beacon);
    xSemaphoreGive(beacon_mutex);
    break;
  }
  default:
    break;
  }

  return 0;
};

static esp_err_t scan() {
  // Cancel any scan that is currently running
  int rc = ble_gap_disc_cancel();
  ESP_RETURN_ON_FALSE(rc == 0 || rc == BLE_HS_EALREADY, ESP_FAIL, RADIO_TAG,
                      "Unable to stop existing BlueTooth scan: %d", rc);

  if (current_mode == BLUETOOTH_MODE_DISABLED) {
    return ESP_OK;
  }

  uint16_t interval = current_mode == BLUETOOTH_MODE_AGGRESSIVE
                          ? BLE_GAP_SCAN_ITVL_MS(60)
                          : BLE_GAP_SCAN_SLOW_INTERVAL1;
  uint16_t window = BLE_GAP_SCAN_FAST_WINDOW;

  uint8_t own_addr_type;

  rc = ble_hs_id_infer_auto(0, &own_addr_type);
  ESP_RETURN_ON_FALSE(rc == 0, ESP_FAIL, RADIO_TAG,
                      "Failed to infer own address type: %d", rc);

  struct ble_gap_disc_params disc_params = {
      .filter_policy = BLE_HCI_SCAN_FILT_NO_WL,
      .filter_duplicates = 0,
      .passive = 1,
      .itvl = interval,
      .window = window,
  };

  rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, gap_event,
                    NULL);
  ESP_RETURN_ON_FALSE(rc == 0, ESP_FAIL, RADIO_TAG,
                      "Error initiating GAP discovery procedure; rc=%d\n", rc);

  return ESP_OK;
};

static void on_sync() {
  ESP_LOGD(RADIO_TAG, "Bluetooth initialized");

  int rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(RADIO_TAG, "Failed to ensure Bluetooth address: %d", rc);
    return;
  }

  synced = true;

  xSemaphoreTake(mutex, portMAX_DELAY);
  ESP_ERROR_CHECK_WITHOUT_ABORT(scan(0, 0));
  advertise();
  xSemaphoreGive(mutex);
}

static void bluetooth_task(void *param) {
  // This function will return only when nimble_port_stop() is executed
  nimble_port_run();

  nimble_port_freertos_deinit();
}

static void things_attr_cb(const char *key, const things_attribute_t *attr) {
  xSemaphoreTake(mutex, portMAX_DELAY);

  // Default to resetting to unset
  adv_major = UINT16_MAX;
  adv_minor = UINT16_MAX;

  switch (attr->type) {
  case THINGS_ATTRIBUTE_TYPE_INT:
    if (attr->value.i > 0xffffffff || attr->value.i < 0) {
      ESP_LOGW(RADIO_TAG, "Attribute %s is too large (treating as unset)", key);
      goto cleanup;
    }
    adv_major = (uint16_t)(attr->value.i >> 16);
    adv_minor = (uint16_t)(attr->value.i & 0xffff);
    break;
  case THINGS_ATTRIBUTE_TYPE_FLOAT:
    // Ugh this really should be an int, but the TB web interface seems to
    // always ship values as floats
    if (attr->value.f > 0xffffffff || attr->value.f < 0 ||
        isnan(attr->value.f) || isinf(attr->value.f) ||
        attr->value.f != (int)attr->value.f) {
      ESP_LOGW(RADIO_TAG, "Attribute %s is invalid (treating as unset)", key);
      goto cleanup;
    }
    adv_major = (uint16_t)((uint32_t)attr->value.f >> 16);
    adv_minor = (uint16_t)((uint32_t)attr->value.f & 0xffff);
    break;
  case THINGS_ATTRIBUTE_TYPE_UNSET:
    break;
  default:
    ESP_LOGW(RADIO_TAG,
             "Invalid attribute type for %s (type %d, treating as unset)", key,
             attr->type);
    break;
  }

cleanup:
  advertise();
  xSemaphoreGive(mutex);
}

esp_err_t bluetooth_init(void) {
  mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(mutex != NULL, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create mutex");

  beacon_mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(beacon_mutex != NULL, ESP_ERR_NO_MEM, RADIO_TAG,
                      "Failed to create beacon mutex");

  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .callback = beacon_cleanup,
                              .arg = NULL,
                              .dispatch_method = ESP_TIMER_TASK,
                              .name = "beacon_cleanup",
                          },
                          &beacon_cleanup_timer),
                      RADIO_TAG, "Failed to create beacon cleanup timer: %d",
                      err_rc_);
  ESP_RETURN_ON_ERROR(
      esp_timer_start_periodic(beacon_cleanup_timer, 5 * 1000 * 1000),
      RADIO_TAG, "Failed to start beacon cleanup timer: %d", err_rc_);

  ESP_RETURN_ON_ERROR(nimble_port_init(), RADIO_TAG,
                      "Failed to initialize NimBLE port: %d", err_rc_);
  ble_svc_gap_init();

  ble_hs_cfg.sync_cb = on_sync;
  ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

  ESP_RETURN_ON_ERROR(esp_read_mac(mac, ESP_MAC_BT), RADIO_TAG,
                      "Failed to read Bluetooth MAC address: %d", err_rc_);
  char device_name[13] = {0};
  snprintf(device_name, sizeof(device_name), "radio-%02x%02x%02x", mac[3],
           mac[4], mac[5]);
  int rc = ble_svc_gap_device_name_set(device_name);
  ESP_RETURN_ON_FALSE(rc == 0, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Failed to set device name: %d", rc);

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P21);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P21);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P21);

  nimble_port_freertos_init(bluetooth_task);

  things_subscribe_attribute("beacon_identity", things_attr_cb);

  things_register_telemetry_generator(telemetry_generator, "bluetooth", NULL);

  return ESP_OK;
}

esp_err_t bluetooth_set_mode(bluetooth_mode_t new_mode) {
  ESP_RETURN_ON_FALSE(new_mode < BLUETOOTH_MODE_MAX, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Invalid Bluetooth mode: %d", new_mode);
  ESP_RETURN_ON_FALSE(mutex != NULL, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Bluetooth not initialized");

  xSemaphoreTake(mutex, portMAX_DELAY);
  bluetooth_mode_t old_mode = current_mode;
  current_mode = new_mode;
  if (old_mode != new_mode) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(scan());
    advertise();
  }
  xSemaphoreGive(mutex);

  xSemaphoreTake(beacon_mutex, portMAX_DELAY);
  struct bt_beacon *beacon, *tmp;
  TAILQ_FOREACH_SAFE(beacon, &beacons, entries, tmp) {
    if (new_mode == BLUETOOTH_MODE_DISABLED) {
      TAILQ_REMOVE(&beacons, beacon, entries);
      free(beacon);
    } else {
      beacon->beacon.last_seen = 0;
      beacon->beacon.period = 0;
    }
  }
  xSemaphoreGive(beacon_mutex);

  return ESP_OK;
}

esp_err_t bluetooth_subscribe_beacon(uint16_t major,
                                     bluetooth_beacon_callback_t callback,
                                     void *arg) {
  ESP_RETURN_ON_FALSE(beacon_mutex != NULL, ESP_ERR_INVALID_STATE, RADIO_TAG,
                      "Bluetooth not initialized");
  ESP_RETURN_ON_FALSE(callback != NULL, ESP_ERR_INVALID_ARG, RADIO_TAG,
                      "Callback must not be NULL");

  xSemaphoreTake(beacon_mutex, portMAX_DELAY);
  struct bt_subscriber *sub = calloc(1, sizeof(struct bt_subscriber));
  if (!sub) {
    ESP_LOGE(RADIO_TAG, "Failed to allocate memory for subscriber");
    xSemaphoreGive(beacon_mutex);
    return ESP_ERR_NO_MEM;
  }

  sub->major = major;
  sub->callback = callback;
  sub->arg = arg;
  TAILQ_INSERT_TAIL(&subscribers, sub, entries);

  struct bt_beacon *newest = NULL, *beacon = NULL;
  TAILQ_FOREACH(beacon, &beacons, entries) {
    if (beacon->beacon.major == major &&
        (!newest || beacon->beacon.last_seen > newest->beacon.last_seen)) {
      newest = beacon;
    }
  }
  notify_subscriber(sub, newest);

  xSemaphoreGive(beacon_mutex);

  return ESP_OK;
}

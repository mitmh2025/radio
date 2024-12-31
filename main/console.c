#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "nvs_flash.h"

#include "calibration.h"
#include "console.h"
#include "led.h"
#include "main.h"
#include "station_funaround.h"
#include "station_pi.h"
#include "storage.h"
#include "things.h"
#include "wifi.h"

#include "board.h"

#include <fcntl.h>

#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_check.h"
#include "esp_console.h"
#include "esp_debug_helpers.h"
#include "esp_log.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "linenoise/linenoise.h"
#include "lwip/stats.h"
#include "soc/usb_serial_jtag_reg.h"

#include "esp_littlefs.h"

static TaskHandle_t console_task_handle = NULL;
static bool force_console = false;

static bool warning_required = true;

static void jtag_poll_timer_cb() {
  if (!usb_serial_jtag_is_connected()) {
    // print a warning the next time we receive a command
    warning_required = true;
  }
}

static void disable_console() {
  REG_SET_BIT(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_PHY_SEL);
}

static void enable_console() {
  REG_CLR_BIT(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_PHY_SEL);
}

static void things_en_console_cb(const char *key,
                                 const things_attribute_t *attr) {
  bool enable = false;
  switch (attr->type) {
  case THINGS_ATTRIBUTE_TYPE_UNSET:
    break;
  case THINGS_ATTRIBUTE_TYPE_BOOL:
    enable = attr->value.b;
    break;
  default:
    ESP_LOGW(RADIO_TAG, "Invalid type for en_console (defaulting to false): %d",
             attr->type);
  }

  if (enable) {
    enable_console();
  } else if (!force_console) {
    disable_console();
  }
}

static const char *WARNING_MESSAGE[] = {
    "=========================================================================="
    "======",
    "This is the USB serial console for the 2025 MIT Mystery Hunt radio. We "
    "have left",
    "it possible to enable and access this serial console exclusively for "
    "Death &",
    "Mayhem to be able to debug the radio. If you are participating in the "
    "Hunt and",
    "seeing this message, you have likely enabled it by accident. Please do "
    "not",
    "use it, and instead contact HQ for further instructions.",
    "=========================================================================="
    "======",
};

static void console_task(void *arg) {
  const char *prompt = LOG_COLOR_I "radio> " LOG_RESET_COLOR;

  if (!force_console) {
    disable_console();
  }

  while (true) {
    if (warning_required) {
      usb_serial_jtag_write_bytes("\n", 1, portMAX_DELAY);
      for (size_t i = 0;
           i < sizeof(WARNING_MESSAGE) / sizeof(WARNING_MESSAGE[0]); i++) {
        usb_serial_jtag_write_bytes(WARNING_MESSAGE[i],
                                    strlen(WARNING_MESSAGE[i]), portMAX_DELAY);
        usb_serial_jtag_write_bytes("\n", 1, portMAX_DELAY);
      }
      usb_serial_jtag_write_bytes("\n", 1, portMAX_DELAY);
      warning_required = false;
    }

    linenoiseSetDumbMode(linenoiseProbe() < 0);

    char *line = linenoise(prompt);
    if (line == NULL) {
      continue;
    }
    linenoiseHistoryAdd(line);

    int ret;
    esp_err_t err = esp_console_run(line, &ret);
    if (err == ESP_ERR_NOT_FOUND) {
      printf("Unrecognized command\n");
    } else if (err == ESP_ERR_INVALID_ARG) {
      // command was empty
    } else if (err == ESP_OK && ret != ESP_OK) {
      printf("Command returned non-zero error code: 0x%x\n", ret);
    } else if (err != ESP_OK) {
      printf("Internal error: %d\n", err);
    }
    linenoiseFree(line);
  }
}

static int restart_func(int argc, char **argv) {
  esp_restart();
  return 0;
}

static int panic_func(int argc, char **argv) {
  *((int *)0) = 0;
  return 0;
}

static struct {
  struct arg_str *caps;
  struct arg_end *end;
} heap_args;

static int heap_func(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&heap_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, heap_args.end, argv[0]);
    return 1;
  }

  uint32_t caps = MALLOC_CAP_DEFAULT;
  if (heap_args.caps->count == 1) {
    if (strcasecmp(heap_args.caps->sval[0], "internal") == 0) {
      caps = MALLOC_CAP_INTERNAL;
    } else if (strcasecmp(heap_args.caps->sval[0], "psram") == 0) {
      caps = MALLOC_CAP_SPIRAM;
    } else {
      arg_print_syntax(stderr, (void **)&heap_args, argv[0]);
      return 1;
    }
  }

  heap_caps_print_heap_info(caps);
  return 0;
}

static int trace_func(int argc, char **argv) {
  esp_err_t ret = esp_backtrace_print_all_tasks(10);
  if (ret != ESP_OK) {
    printf("Failed to print backtrace: %d\n", ret);
  }
  return 0;
}

static int tasks_func(int argc, char **argv) {
  const size_t bytes_per_task = 40; /* see vTaskList description */
  const size_t buffer_size = uxTaskGetNumberOfTasks() * bytes_per_task;
  char *task_list_buffer = malloc(buffer_size);
  if (task_list_buffer == NULL) {
    ESP_LOGE(RADIO_TAG, "failed to allocate buffer for vTaskList output");
    return 1;
  }
  fputs("Task Name\tStatus\tPrio\tHWM\tTask\tCore#\n", stdout);
  vTaskList(task_list_buffer);
  fputs(task_list_buffer, stdout);

  vTaskGetRunTimeStats(task_list_buffer);
  fputs("\nTask Name\tRuntime\t%CPU\n", stdout);
  fputs(task_list_buffer, stdout);

  free(task_list_buffer);
  return 0;
}

static int lwip_func(int argc, char **argv) {
  stats_display();
  return 0;
}

static int gpio_func(int argc, char **argv) {
  gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
  return 0;
}

static int intr_func(int argc, char **argv) {
  esp_intr_dump(NULL);
  return 0;
}

static int timers_func(int argc, char **argv) {
  esp_timer_dump(stdout);
  return 0;
}

static struct {
  struct arg_str *hostname;
  struct arg_str *token;
  struct arg_end *end;
} provision_args;

static int provision_func(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&provision_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, provision_args.end, argv[0]);
    return 1;
  }

  if (provision_args.hostname->count == 0 || provision_args.token->count == 0) {
    arg_print_syntax(stderr, (void **)&provision_args, argv[0]);
    return 1;
  }

  esp_err_t err = things_provision(provision_args.hostname->sval[0],
                                   provision_args.token->sval[0]);
  if (err != ESP_OK) {
    printf("Failed to provision device: %d\n", err);
    return 1;
  }

  return 0;
}

static int recalibrate_func(int argc, char **argv) {
  esp_err_t err = calibration_erase();
  if (err != ESP_OK) {
    printf("Failed to erase calibration: %d\n", err);
    return 1;
  }

  esp_restart();
  return 0;
}

static struct {
  struct arg_str *dir;
  struct arg_lit *long_format;
  struct arg_end *end;
} list_args;

static int list_func(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&list_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, list_args.end, argv[0]);
    return 1;
  }

  if (list_args.dir->count == 0) {
    arg_print_syntax(stderr, (void **)&list_args, argv[0]);
    return 1;
  }

  DIR *dir = opendir(list_args.dir->sval[0]);
  if (dir == NULL) {
    printf("Failed to open directory: %s\n", strerror(errno));
    return 1;
  }

  struct dirent *entry;
  while ((entry = readdir(dir)) != NULL) {
    if (list_args.long_format->count) {
      struct stat st;
      char path[PATH_MAX];
      snprintf(path, sizeof(path), "%s/%s", list_args.dir->sval[0],
               entry->d_name);
      if (stat(path, &st) == 0) {
        printf("%s\t%ld\t%s\n", entry->d_type == DT_DIR ? "d" : "-",
               entry->d_type == DT_DIR ? 0 : st.st_size, entry->d_name);
      }
    } else {
      printf("%s%s\n", entry->d_name, entry->d_type == DT_DIR ? "/" : "");
    }
  }

  closedir(dir);

  return 0;
}

static struct {
  struct arg_str *path;
  struct arg_end *end;
} cat_args;

static int cat_func(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&cat_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, cat_args.end, argv[0]);
    return 1;
  }

  if (cat_args.path->count == 0) {
    arg_print_syntax(stderr, (void **)&cat_args, argv[0]);
    return 1;
  }

  FILE *file = fopen(cat_args.path->sval[0], "r");
  if (file == NULL) {
    printf("Failed to open file: %s\n", strerror(errno));
    return 1;
  }

  char buffer[1024];
  size_t read;
  while ((read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
    fwrite(buffer, 1, read, stdout);
  }

  fclose(file);
  printf("\n");

  return 0;
}

static struct {
  struct arg_str *path;
  struct arg_end *end;
} rm_args;

static int rm_func(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&rm_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, rm_args.end, argv[0]);
    return 1;
  }

  if (rm_args.path->count == 0) {
    arg_print_syntax(stderr, (void **)&rm_args, argv[0]);
    return 1;
  }

  if (unlink(rm_args.path->sval[0]) != 0) {
    printf("Failed to remove file: %s\n", strerror(errno));
    return 1;
  }

  return 0;
}

static int df_func(int argc, char **argv) {
  size_t total, used;
  int ret = esp_littlefs_mountpoint_info(STORAGE_MOUNTPOINT, &total, &used);
  if (ret != ESP_OK) {
    printf("Failed to get storage info: %d\n", ret);
    return 1;
  }

  printf("Used:  %8zu bytes (%u%%)\n", used, (used * 100) / total);
  printf("Free:  %8zu bytes (%u%%)\n", total - used,
         ((total - used) * 100) / total);
  printf("Total: %8zu bytes\n", total);

  return 0;
}

static int gc_func(int argc, char **argv) {
  esp_err_t ret = esp_littlefs_gc_mountpoint(STORAGE_MOUNTPOINT);
  if (ret != ESP_OK) {
    printf("Failed to run GC: %d\n", ret);
    return 1;
  }

  return 0;
}

static int nvs_stats_func(int argc, char **argv) {
  nvs_stats_t stats;
  esp_err_t err = nvs_get_stats(NULL, &stats);
  if (err != ESP_OK) {
    printf("Failed to get NVS stats: %d\n", err);
    return 1;
  }

  printf("Used entries: %zu\n", stats.used_entries);
  printf("Free entries: %zu\n", stats.free_entries);
  printf("Available entries: %zu\n", stats.available_entries);
  printf("All entries: %zu\n", stats.total_entries);
  printf("Namespace count: %zu\n", stats.namespace_count);

  return 0;
}

static int nvs_ls_func(int argc, char **argv) {
  nvs_iterator_t iter = NULL;
  esp_err_t err =
      nvs_entry_find(NVS_DEFAULT_PART_NAME, NULL, NVS_TYPE_ANY, &iter);
  if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
    printf("Failed to find attributes in NVS: %d\n", err);
    return 1;
  }

  printf("Namespace\tKey\tType\n");

  while (iter) {
    nvs_entry_info_t info;
    err = nvs_entry_info(iter, &info);
    if (err != ESP_OK) {
      printf("Failed to get attribute info: %d\n", err);
      break;
    }

    const char *type;
    switch (info.type) {
    case NVS_TYPE_I8:
      type = "i8";
      break;
    case NVS_TYPE_U8:
      type = "u8";
      break;
    case NVS_TYPE_I16:
      type = "i16";
      break;
    case NVS_TYPE_U16:
      type = "u16";
      break;
    case NVS_TYPE_I32:
      type = "i32";
      break;
    case NVS_TYPE_U32:
      type = "u32";
      break;
    case NVS_TYPE_I64:
      type = "i64";
      break;
    case NVS_TYPE_U64:
      type = "u64";
      break;
    case NVS_TYPE_STR:
      type = "str";
      break;
    case NVS_TYPE_BLOB:
      type = "blob";
      break;
    default:
      type = "unknown";
      break;
    }

    printf("%s\t%s\t%s\n", info.namespace_name, info.key, type);

    err = nvs_entry_next(&iter);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
      printf("Failed to find attributes in NVS: %d\n", err);
      break;
    }
  }

  return 0;
}

static struct {
  struct arg_str *namespace;
  struct arg_str *key;
  struct arg_end *end;
} nvs_get_args;

static int nvs_get(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&nvs_get_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, nvs_get_args.end, argv[0]);
    return 1;
  }

  if (nvs_get_args.namespace->count == 0 || nvs_get_args.key->count == 0) {
    arg_print_syntax(stderr, (void **)&nvs_get_args, argv[0]);
    return 1;
  }

  int ret = 0;
  nvs_handle_t handle;
  char *str_value = NULL;
  esp_err_t err =
      nvs_open(nvs_get_args.namespace->sval[0], NVS_READONLY, &handle);
  if (err != ESP_OK) {
    printf("Failed to open NVS handle: %d\n", err);
    return 1;
  }

  nvs_type_t type;
  err = nvs_find_key(handle, nvs_get_args.key->sval[0], &type);
  if (err != ESP_OK) {
    printf("Failed to find key in NVS: %d\n", err);
    ret = 1;
    goto cleanup;
  }

  switch (type) {
  case NVS_TYPE_I8: {
    int8_t value;
    err = nvs_get_i8(handle, nvs_get_args.key->sval[0], &value);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }
    printf("%" PRId8 " (0x%" PRIx8 ")\n", value, value);
    break;
  }
  case NVS_TYPE_U8: {
    uint8_t value;
    err = nvs_get_u8(handle, nvs_get_args.key->sval[0], &value);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }
    printf("%" PRIu8 " (0x%" PRIx8 ")\n", value, value);
    break;
  }
  case NVS_TYPE_I16: {
    int16_t value;
    err = nvs_get_i16(handle, nvs_get_args.key->sval[0], &value);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }
    printf("%" PRId16 " (0x%" PRIx16 ")\n", value, value);
    break;
  }
  case NVS_TYPE_U16: {
    uint16_t value;
    err = nvs_get_u16(handle, nvs_get_args.key->sval[0], &value);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }
    printf("%" PRIu16 " (0x%" PRIx16 ")\n", value, value);
    break;
  }
  case NVS_TYPE_I32: {
    int32_t value;
    err = nvs_get_i32(handle, nvs_get_args.key->sval[0], &value);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }
    printf("%" PRId32 " (0x%" PRIx32 ")\n", value, value);
    break;
  }
  case NVS_TYPE_U32: {
    uint32_t value;
    err = nvs_get_u32(handle, nvs_get_args.key->sval[0], &value);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }
    printf("%" PRIu32 " (0x%" PRIx32 ")\n", value, value);
    break;
  }
  case NVS_TYPE_I64: {
    int64_t value;
    err = nvs_get_i64(handle, nvs_get_args.key->sval[0], &value);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }
    printf("%" PRId64 " (0x%" PRIx64 ")\n", value, value);
    break;
  }
  case NVS_TYPE_U64: {
    uint64_t value;
    err = nvs_get_u64(handle, nvs_get_args.key->sval[0], &value);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }
    printf("%" PRIu64 " (0x%" PRIx64 ")\n", value, value);
    break;
  }
  case NVS_TYPE_STR: {
    size_t length;
    err = nvs_get_str(handle, nvs_get_args.key->sval[0], NULL, &length);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }

    str_value = malloc(length);
    if (str_value == NULL) {
      printf("Failed to allocate memory for string value\n");
      ret = 1;
      goto cleanup;
    }

    err = nvs_get_str(handle, nvs_get_args.key->sval[0], str_value, &length);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }

    printf("%s\n", str_value);
    break;
  }
  case NVS_TYPE_BLOB: {
    size_t length;
    err = nvs_get_blob(handle, nvs_get_args.key->sval[0], NULL, &length);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }

    str_value = malloc(length);
    if (str_value == NULL) {
      printf("Failed to allocate memory for blob value\n");
      ret = 1;
      goto cleanup;
    }

    err = nvs_get_blob(handle, nvs_get_args.key->sval[0], str_value, &length);
    if (err != ESP_OK) {
      printf("Failed to get value from NVS: %d\n", err);
      ret = 1;
      goto cleanup;
    }

    ESP_LOG_BUFFER_HEXDUMP(RADIO_TAG, str_value, length, ESP_LOG_INFO);
    break;
  }
  default:
    printf("Unsupported attribute type: %d\n", type);
    break;
  }

cleanup:
  if (str_value != NULL) {
    free(str_value);
  }
  nvs_close(handle);
  return ret;
}

static struct {
  struct arg_str *namespace;
  struct arg_str *key;
  struct arg_end *end;
} nvs_rm_args;

static int nvs_rm(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&nvs_rm_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, nvs_rm_args.end, argv[0]);
    return 1;
  }

  if (nvs_rm_args.namespace->count == 0 || nvs_rm_args.key->count == 0) {
    arg_print_syntax(stderr, (void **)&nvs_rm_args, argv[0]);
    return 1;
  }

  nvs_handle_t handle;
  esp_err_t err =
      nvs_open(nvs_rm_args.namespace->sval[0], NVS_READWRITE, &handle);
  if (err != ESP_OK) {
    printf("Failed to open NVS handle: %d\n", err);
    return 1;
  }

  err = nvs_erase_key(handle, nvs_rm_args.key->sval[0]);
  if (err != ESP_OK) {
    printf("Failed to erase key from NVS: %d\n", err);
    nvs_close(handle);
    return 1;
  }

  nvs_close(handle);
  return 0;
}

static struct {
  struct arg_int *stage;
  struct arg_end *end;
} pi_set_stage_args;

static int pi_set_stage(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&pi_set_stage_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, pi_set_stage_args.end, argv[0]);
    return 1;
  }

  if (pi_set_stage_args.stage->count == 0) {
    arg_print_syntax(stderr, (void **)&pi_set_stage_args, argv[0]);
    return 1;
  }

  esp_err_t err = station_pi_set_stage(pi_set_stage_args.stage->ival[0]);
  if (err != ESP_OK) {
    printf("Failed to save stage: %d\n", err);
    return 1;
  }

  return 0;
}

static int pi_reset_play_time(int argc, char **argv) {
  esp_err_t err = station_pi_reset_play_time();
  if (err != ESP_OK) {
    printf("Failed to reset play time: %d\n", err);
    return 1;
  }

  return 0;
}

static struct {
  struct arg_int *ratchet;
  struct arg_end *end;
} funaround_set_ratchet_args;

static int funaround_set_ratchet(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&funaround_set_ratchet_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, funaround_set_ratchet_args.end, argv[0]);
    return 1;
  }

  if (funaround_set_ratchet_args.ratchet->count == 0) {
    arg_print_syntax(stderr, (void **)&funaround_set_ratchet_args, argv[0]);
    return 1;
  }

  esp_err_t err = station_funaround_set_ratchet(
      funaround_set_ratchet_args.ratchet->ival[0]);
  if (err != ESP_OK) {
    printf("Failed to set ratchet: %d\n", err);
    return 1;
  }

  return 0;
}

static struct {
  struct arg_str *ssid;
  struct arg_str *password;
  struct arg_end *end;
} wifi_set_alt_network_args;

static int wifi_set_alt_network_cmd(int argc, char **argv) {
  int nerrors = arg_parse(argc, argv, (void **)&wifi_set_alt_network_args);

  if (nerrors != 0) {
    arg_print_errors(stderr, wifi_set_alt_network_args.end, argv[0]);
    return 1;
  }

  if (wifi_set_alt_network_args.ssid->count == 0) {
    arg_print_syntax(stderr, (void **)&wifi_set_alt_network_args, argv[0]);
    return 1;
  }

  esp_err_t err =
      wifi_set_alt_network(wifi_set_alt_network_args.ssid->sval[0],
                           wifi_set_alt_network_args.password->sval[0]);
  if (err != ESP_OK) {
    printf("Failed to set alt network: %d\n", err);
    return 1;
  }

  err = wifi_force_reconnect();
  if (err != ESP_OK) {
    printf("Failed to force reconnect: %d\n", err);
    return 1;
  }

  return 0;
}

esp_err_t console_init() {
  // Check for debug mode (forced console)
#ifdef CONFIG_RADIO_GIANT_SWITCH
  force_console = true;
#else
  gpio_config_t button_config = {
      .pin_bit_mask = BIT64(BUTTON_CIRCLE_PIN) | BIT64(BUTTON_TRIANGLE_PIN),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_RETURN_ON_ERROR(gpio_config(&button_config), RADIO_TAG,
                      "Failed to configure buttons");
  if (gpio_get_level(BUTTON_CIRCLE_PIN) == 0 &&
      gpio_get_level(BUTTON_TRIANGLE_PIN) == 0) {
    led_set_pixel(1, 255, 100, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    led_set_pixel(1, 0, 0, 0);
    force_console = true;
  }
#endif

  usb_serial_jtag_driver_config_t usb_serial_jtag_config =
      USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  usb_serial_jtag_config.tx_buffer_size = 1024;
  esp_err_t err = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to install USB serial JTAG driver: %d", err);

  usb_serial_jtag_vfs_use_driver();

  // Block on stdin and stdout
  fcntl(fileno(stdout), F_SETFL, 0);
  fcntl(fileno(stdin), F_SETFL, 0);

  esp_console_config_t cfg = ESP_CONSOLE_CONFIG_DEFAULT();
  cfg.hint_color = atoi(LOG_COLOR_CYAN);
  err = esp_console_init(&cfg);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to initialize console: %d", err);

  linenoiseSetCompletionCallback(&esp_console_get_completion);
  linenoiseSetHintsCallback((linenoiseHintsCallback *)&esp_console_get_hint);

  err = esp_console_register_help_command();
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register help command: %d",
                      err);

  esp_console_cmd_t cmd_restart = {
      .command = "restart",
      .help = "Restart the device",
      .func = &restart_func,
  };
  err = esp_console_cmd_register(&cmd_restart);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register restart command: %d",
                      err);

  esp_console_cmd_t cmd_panic = {
      .command = "panic",
      .help = "Cause a panic",
      .func = &panic_func,
  };
  err = esp_console_cmd_register(&cmd_panic);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register panic command: %d",
                      err);

  esp_console_cmd_t cmd_trace = {
      .command = "trace",
      .help = "Print backtrace of all tasks",
      .func = &trace_func,
  };
  err = esp_console_cmd_register(&cmd_trace);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register trace command: %d",
                      err);

  heap_args.caps =
      arg_str0(NULL, NULL, "internal|psram",
               "Limit to these memory capabilities (defaults to all)");
  heap_args.end = arg_end(1);
  esp_console_cmd_t cmd_heap = {
      .command = "heap",
      .help = "Print heap usage",
      .hint = NULL,
      .func = &heap_func,
      .argtable = &heap_args,
  };
  err = esp_console_cmd_register(&cmd_heap);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register heap command: %d",
                      err);

  esp_console_cmd_t cmd_tasks = {
      .command = "tasks",
      .help = "Print task list",
      .func = &tasks_func,
  };
  err = esp_console_cmd_register(&cmd_tasks);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register tasks command: %d",
                      err);

  esp_console_cmd_t cmd_lwip = {
      .command = "lwip",
      .help = "Print lwIP statistics",
      .func = &lwip_func,
  };
  err = esp_console_cmd_register(&cmd_lwip);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register lwip command: %d",
                      err);

  esp_console_cmd_t cmd_intr = {
      .command = "intr",
      .help = "Print interrupt information",
      .func = &intr_func,
  };
  err = esp_console_cmd_register(&cmd_intr);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register intr command: %d",
                      err);

  esp_console_cmd_t cmd_gpio = {
      .command = "gpio",
      .help = "Print GPIO configuration",
      .func = &gpio_func,
  };
  err = esp_console_cmd_register(&cmd_gpio);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register gpio command: %d",
                      err);

  esp_console_cmd_t cmd_timers = {
      .command = "timers",
      .help = "Print timer information",
      .func = &timers_func,
  };
  err = esp_console_cmd_register(&cmd_timers);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register timer command: %d",
                      err);

  provision_args.hostname =
      arg_str1(NULL, NULL, "<hostname>", "ThingsBoard hostname");
  provision_args.token =
      arg_str1(NULL, NULL, "<token>", "ThingsBoard device token");
  provision_args.end = arg_end(1);
  esp_console_cmd_t cmd_provision = {
      .command = "provision",
      .help = "Provision the device with a ThingsBoard device token",
      .hint = NULL,
      .func = &provision_func,
      .argtable = &provision_args,
  };
  err = esp_console_cmd_register(&cmd_provision);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to register provision command: %d", err);

  esp_console_cmd_t cmd_recalibrate = {
      .command = "recalibrate",
      .help = "Erase calibration data and restart",
      .func = &recalibrate_func,
  };
  err = esp_console_cmd_register(&cmd_recalibrate);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to register recalibrate command: %d", err);

  list_args.dir = arg_str1(NULL, NULL, "<dir>", "Directory to list");
  list_args.long_format = arg_lit0("l", "long", "Use a long listing format");
  list_args.end = arg_end(1);
  esp_console_cmd_t cmd_list = {
      .command = "ls",
      .help = "List files in a directory",
      .hint = NULL,
      .func = &list_func,
      .argtable = &list_args,
  };
  err = esp_console_cmd_register(&cmd_list);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register ls command: %d", err);

  cat_args.path = arg_str1(NULL, NULL, "<path>", "File to display");
  cat_args.end = arg_end(1);
  esp_console_cmd_t cmd_cat = {
      .command = "cat",
      .help = "Display the contents of a file",
      .hint = NULL,
      .func = &cat_func,
      .argtable = &cat_args,
  };
  err = esp_console_cmd_register(&cmd_cat);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register cat command: %d",
                      err);

  rm_args.path = arg_str1(NULL, NULL, "<path>", "File to remove");
  rm_args.end = arg_end(1);
  esp_console_cmd_t cmd_rm = {
      .command = "rm",
      .help = "Remove a file",
      .hint = NULL,
      .func = &rm_func,
      .argtable = &rm_args,
  };
  err = esp_console_cmd_register(&cmd_rm);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register rm command: %d", err);

  esp_console_cmd_t cmd_df = {
      .command = "df",
      .help = "Print storage usage",
      .func = &df_func,
  };
  err = esp_console_cmd_register(&cmd_df);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register df command: %d", err);

  esp_console_cmd_t cmd_gc = {
      .command = "gc",
      .help = "Run garbage collection on storage",
      .func = &gc_func,
  };
  err = esp_console_cmd_register(&cmd_gc);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register gc command: %d", err);

  esp_console_cmd_t cmd_nvs_stats = {
      .command = "nvs-stats",
      .help = "Print NVS statistics",
      .func = &nvs_stats_func,
  };
  err = esp_console_cmd_register(&cmd_nvs_stats);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to register nvs-stats command: %d", err);

  esp_console_cmd_t cmd_nvs_ls = {
      .command = "nvs-ls",
      .help = "List NVS entries",
      .func = &nvs_ls_func,
  };
  err = esp_console_cmd_register(&cmd_nvs_ls);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register nvs-ls command: %d",
                      err);

  nvs_get_args.namespace = arg_str1(NULL, NULL, "<namespace>", "NVS namespace");
  nvs_get_args.key = arg_str1(NULL, NULL, "<key>", "NVS key");
  nvs_get_args.end = arg_end(2);
  esp_console_cmd_t cmd_nvs_get = {
      .command = "nvs-get",
      .help = "Get a value from NVS",
      .hint = NULL,
      .func = &nvs_get,
      .argtable = &nvs_get_args,
  };
  err = esp_console_cmd_register(&cmd_nvs_get);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register nvs-get command: %d",
                      err);

  nvs_rm_args.namespace = arg_str1(NULL, NULL, "<namespace>", "NVS namespace");
  nvs_rm_args.key = arg_str1(NULL, NULL, "<key>", "NVS key");
  nvs_rm_args.end = arg_end(2);
  esp_console_cmd_t cmd_nvs_rm = {
      .command = "nvs-rm",
      .help = "Remove a value from NVS",
      .hint = NULL,
      .func = &nvs_rm,
      .argtable = &nvs_rm_args,
  };
  err = esp_console_cmd_register(&cmd_nvs_rm);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to register nvs-rm command: %d",
                      err);

  pi_set_stage_args.stage =
      arg_int1(NULL, NULL, "<stage>", "Stage to set (0-indexed)");
  pi_set_stage_args.end = arg_end(1);
  esp_console_cmd_t cmd_pi_set_stage = {
      .command = "pi-set-stage",
      .help = "Set the current stage for the Pi",
      .hint = NULL,
      .func = &pi_set_stage,
      .argtable = &pi_set_stage_args,
  };
  err = esp_console_cmd_register(&cmd_pi_set_stage);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to register pi-set-stage command: %d", err);

  esp_console_cmd_t cmd_pi_reset_play_time = {
      .command = "pi-reset-play-time",
      .help = "Reset the total play time for the Pi",
      .func = &pi_reset_play_time,
  };
  err = esp_console_cmd_register(&cmd_pi_reset_play_time);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to register pi-reset-play-time command: %d", err);

  funaround_set_ratchet_args.ratchet = arg_int1(
      NULL, NULL, "<ratchet>", "Ratchet to set (0 for start of funaround)");
  funaround_set_ratchet_args.end = arg_end(1);
  esp_console_cmd_t cmd_funaround_set_ratchet = {
      .command = "funaround-set-ratchet",
      .help = "Set the current ratchet for the Funaround",
      .hint = NULL,
      .func = &funaround_set_ratchet,
      .argtable = &funaround_set_ratchet_args,
  };
  err = esp_console_cmd_register(&cmd_funaround_set_ratchet);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to register funaround-set-ratchet command: %d",
                      err);

  wifi_set_alt_network_args.ssid =
      arg_str1(NULL, NULL, "<ssid>", "SSID of the network");
  wifi_set_alt_network_args.password =
      arg_str0(NULL, NULL, "<password>", "Password of the network");
  wifi_set_alt_network_args.end = arg_end(2);
  esp_console_cmd_t cmd_wifi_set_alt_network = {
      .command = "wifi-set-alt-network",
      .help = "Set an alternate network for the device to connect to",
      .hint = NULL,
      .func = &wifi_set_alt_network_cmd,
      .argtable = &wifi_set_alt_network_args,
  };
  err = esp_console_cmd_register(&cmd_wifi_set_alt_network);
  ESP_RETURN_ON_ERROR(err, RADIO_TAG,
                      "Failed to register wifi-set-alt-network command: %d",
                      err);

  ESP_RETURN_ON_FALSE(pdPASS == xTaskCreate(console_task, "console", 6144, NULL,
                                            21, &console_task_handle),
                      ESP_FAIL, RADIO_TAG, "Failed to create console task");

  ESP_RETURN_ON_ERROR(
      things_subscribe_attribute("en_console", things_en_console_cb), RADIO_TAG,
      "Failed to subscribe to en_console attribute: %d", err);

  esp_timer_handle_t timer = NULL;
  ESP_RETURN_ON_ERROR(esp_timer_create(
                          &(esp_timer_create_args_t){
                              .callback = jtag_poll_timer_cb,
                              .name = "jtag_poll_timer",
                              .dispatch_method = ESP_TIMER_TASK,
                          },
                          &timer),
                      RADIO_TAG, "Failed to create JTAG poll timer: %d", err);
  jtag_poll_timer_cb();
  ESP_RETURN_ON_ERROR(esp_timer_start_periodic(timer, 500 * 1000), RADIO_TAG,
                      "Failed to start JTAG poll timer: %d", err);

  return ESP_OK;
}

#include "console.h"
#include "main.h"
#include "things.h"

#include <fcntl.h>

#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "driver/usb_serial_jtag.h"

static void console_task(void *arg)
{
  // TODO: support starting/stopping the console when radio is in/out of debug
  // mode by dup'ing stdin and closing/reopening the original fd as needed

  const char *prompt = LOG_COLOR_I "radio> " LOG_RESET_COLOR;
  while (true)
  {
    char *line = linenoise(prompt);
    if (line == NULL)
    {
      continue;
    }
    linenoiseHistoryAdd(line);

    int ret;
    esp_err_t err = esp_console_run(line, &ret);
    if (err == ESP_ERR_NOT_FOUND)
    {
      printf("Unrecognized command\n");
    }
    else if (err == ESP_ERR_INVALID_ARG)
    {
      // command was empty
    }
    else if (err == ESP_OK && ret != ESP_OK)
    {
      printf("Command returned non-zero error code: 0x%x (%s)\n", ret, esp_err_to_name(ret));
    }
    else if (err != ESP_OK)
    {
      printf("Internal error: %s\n", esp_err_to_name(err));
    }
    linenoiseFree(line);
  }
}

static int restart_func(int argc, char **argv)
{
  esp_restart();
  return 0;
}

static int heap_func(int argc, char **argv)
{
  heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);
  return 0;
}

static struct
{
  struct arg_str *token;
  struct arg_end *end;
} provision_args;

static int provision_func(int argc, char **argv)
{
  int nerrors = arg_parse(argc, argv, (void **)&provision_args);

  if (nerrors != 0)
  {
    arg_print_errors(stderr, provision_args.end, argv[0]);
    return 1;
  }

  if (provision_args.token->count == 0)
  {
    arg_print_syntax(stderr, (void **)&provision_args, argv[0]);
    return 1;
  }

  esp_err_t err = things_provision(provision_args.token->sval[0]);
  if (err != ESP_OK)
  {
    printf("Failed to provision device: %s\n", esp_err_to_name(err));
    return 1;
  }

  return 0;
}

static int deprovision_func(int argc, char **argv)
{
  esp_err_t err = things_deprovision();
  if (err != ESP_OK)
  {
    printf("Failed to deprovision device: %s\n", esp_err_to_name(err));
    return 1;
  }

  return 0;
}

esp_err_t console_init()
{
  // Block on stdin and stdout
  fcntl(fileno(stdout), F_SETFL, 0);
  fcntl(fileno(stdin), F_SETFL, 0);

  usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  esp_err_t err = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to install USB serial JTAG driver: %s", esp_err_to_name(err));
    return err;
  }

  esp_vfs_usb_serial_jtag_use_driver();

  esp_console_config_t cfg = ESP_CONSOLE_CONFIG_DEFAULT();
  cfg.hint_color = atoi(LOG_COLOR_CYAN);
  err = esp_console_init(&cfg);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to initialize console: %s", esp_err_to_name(err));
    return err;
  }

  linenoiseSetCompletionCallback(&esp_console_get_completion);
  linenoiseSetHintsCallback((linenoiseHintsCallback *)&esp_console_get_hint);

  err = esp_console_register_help_command();
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to register help command: %s", esp_err_to_name(err));
    return err;
  }

  esp_console_cmd_t cmd_restart = {
      .command = "restart",
      .help = "Restart the device",
      .func = &restart_func,
  };
  err = esp_console_cmd_register(&cmd_restart);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to register restart command: %s", esp_err_to_name(err));
    return err;
  }

  esp_console_cmd_t cmd_heap = {
      .command = "heap",
      .help = "Print heap usage",
      .func = &heap_func,
  };
  err = esp_console_cmd_register(&cmd_heap);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to register heap command: %s", esp_err_to_name(err));
    return err;
  }

  provision_args.token = arg_str1(NULL, NULL, "<token>", "ThingsBoard device token");
  provision_args.end = arg_end(1);
  esp_console_cmd_t cmd_provision = {
      .command = "provision",
      .help = "Provision the device with a ThingsBoard device token",
      .hint = NULL,
      .func = &provision_func,
      .argtable = &provision_args,
  };
  err = esp_console_cmd_register(&cmd_provision);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to register provision command: %s", esp_err_to_name(err));
    return err;
  }

  esp_console_cmd_t cmd_deprovision = {
      .command = "deprovision",
      .help = "Erase the ThingsBoard device token. (Note that this will have no effect until reset.)",
      .func = &deprovision_func,
  };
  err = esp_console_cmd_register(&cmd_deprovision);
  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to register deprovision command: %s", esp_err_to_name(err));
    return err;
  }

  xTaskCreatePinnedToCore(console_task, "console", 4096, NULL, 5, NULL, APP_CPU_NUM);

  return ESP_OK;
}

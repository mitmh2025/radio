#include "storage.h"
#include "main.h"

#include "esp_log.h"
#include "esp_check.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_littlefs.h"
#include "rom/ets_sys.h"

#include "errno.h"

#include "board.h"

#define BLOCK_FLASH_CMD_READ_STATUS_1 0x05
#define BLOCK_FLASH_CMD_READ_STATUS_2 0x35
#define BLOCK_FLASH_CMD_READ_STATUS_3 0x15
#define BLOCK_FLASH_CMD_WRITE_STATUS_1 0x01
#define BLOCK_FLASH_CMD_WRITE_STATUS_2 0x31
#define BLOCK_FLASH_CMD_WRITE_STATUS_3 0x11
#define BLOCK_FLASH_CMD_WRITE_ENABLE 0x06
#define BLOCK_FLASH_CMD_WRITE_ENABLE_VOLATILE 0x50
#define BLOCK_FLASH_CMD_READ_FAST_QUAD_IO 0xeb
#define BLOCK_FLASH_CMD_WRITE_QUAD 0x32
#define BLOCK_FLASH_CMD_ERASE_SECTOR 0x20
#define BLOCK_FLASH_CMD_JEDEC_ID 0x9f
#define BLOCK_FLASH_CMD_ENABLE_RESET 0x66
#define BLOCK_FLASH_CMD_RESET 0x99

typedef union
{
  uint8_t raw;
  struct
  {
    uint8_t busy : 1;
    uint8_t write_enable_latch : 1;
    uint8_t block_protect_0 : 1;
    uint8_t block_protect_1 : 1;
    uint8_t block_protect_2 : 1;
    uint8_t top_bottom_protect : 1;
    uint8_t sector_protect : 1;
    uint8_t status_register_protect : 1;
  } __attribute__((packed)) parsed;
} block_flash_status_reg1;

typedef union
{
  uint8_t raw;
  struct
  {
    uint8_t status_register_lock : 1;
    uint8_t quad_enable : 1;
    uint8_t reserved : 1;
    uint8_t security_register_lock_1 : 1;
    uint8_t security_register_lock_2 : 1;
    uint8_t security_register_lock_3 : 1;
    uint8_t complement_protect : 1;
    uint8_t suspend_status : 1;
  } __attribute__((packed)) parsed;
} block_flash_status_reg2;

typedef union
{
  uint8_t raw;
  struct
  {
    uint8_t reserved1 : 2;
    uint8_t write_protect_selection : 1;
    uint8_t reserved2 : 2;
    uint8_t drive_strength : 2;
    uint8_t hold_reset : 1;
  } __attribute__((packed)) parsed;
} block_flash_status_reg3;

static enum {
  BLOCK_UNINITIALIZED = -1,
  BLOCK_FLASH,
  BLOCK_SD_CARD,
} block_type = BLOCK_UNINITIALIZED;

typedef struct
{
  SemaphoreHandle_t mutex;
  spi_device_handle_t handle;
  // Track whether the flash is known ready for a read or write command. (If
  // false, we need to wait until the busy status bit is cleared)
  bool ready;
} block_flash_t;

#define BLOCK_FLASH_WRITE_MAX_SIZE 256
#define BLOCK_FLASH_ERASE_SIZE 4096
#define BLOCK_FLASH_BLOCK_COUNT (16 /* erase sectors per block */ * 256 /* blocks per chip */)
#define BLOCK_FLASH_SIZE_BYTES (BLOCK_FLASH_ERASE_SIZE * BLOCK_FLASH_BLOCK_COUNT)
#define BLOCK_FLASH_COUNT 2

// One of these two will be populated based on the block type
static block_flash_t block_flash[BLOCK_FLASH_COUNT] = {};
static sdmmc_card_t block_sdmmc_card;

SemaphoreHandle_t storage_mounted_mutex = NULL;
static bool storage_mounted = false;

// Must hold mutex before calling this
static esp_err_t flash_wait_ready(block_flash_t *flash, TickType_t timeout)
{
  if (!flash)
  {
    return ESP_ERR_INVALID_ARG;
  }

  if (flash->ready)
  {
    return ESP_OK;
  }

  spi_transaction_t txn = {
      .cmd = BLOCK_FLASH_CMD_READ_STATUS_1,
      .flags = SPI_TRANS_USE_RXDATA,
      .rxlength = 8,
  };

  TickType_t now = xTaskGetTickCount();
  TickType_t deadline = now + timeout;
  if (deadline < now)
  {
    // Overflow
    deadline = portMAX_DELAY;
  }

  do
  {
    esp_err_t err = spi_device_transmit(flash->handle, &txn);
    ESP_RETURN_ON_ERROR(err, RADIO_TAG, "Failed to fetch SPI flash status");
    block_flash_status_reg1 reg = {.raw = txn.rx_data[0]};
    if (reg.parsed.busy == 0)
    {
      flash->ready = true;
      return ESP_OK;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  } while (xTaskGetTickCount() < deadline);

  return ESP_ERR_TIMEOUT;
}

// Must hold mutex before calling this
static esp_err_t flash_enable_write(block_flash_t *flash)
{
  if (!flash)
  {
    return ESP_ERR_INVALID_ARG;
  }

  spi_transaction_ext_t txn = {
      .base = {
          .cmd = BLOCK_FLASH_CMD_WRITE_ENABLE,
      },
  };
  return spi_device_transmit(flash->handle, &txn.base);
}

static esp_err_t flash_init(block_flash_t *flash, spi_device_handle_t device)
{
  if (!flash || !device)
  {
    return ESP_ERR_INVALID_ARG;
  }

  spi_transaction_t txn = {
      .cmd = BLOCK_FLASH_CMD_JEDEC_ID,
      .flags = SPI_TRANS_USE_RXDATA,
      .rxlength = 24,
  };
  ESP_RETURN_ON_ERROR(spi_device_transmit(device, &txn), RADIO_TAG, "Failed to probe SPI flash");

  uint16_t dev_id = (txn.tx_data[1] << 8) | txn.tx_data[2];
  if (txn.tx_data[0] != 0xef || (dev_id != 0x7018 && dev_id != 0x4018))
  {
    return ESP_ERR_NOT_FOUND;
  }

  // Wait to allocate mutex until we've finished initialization
  flash->handle = device;
  flash->ready = false;

  // Ensure ready before issuing reset
  ESP_RETURN_ON_ERROR(flash_wait_ready(flash, portMAX_DELAY), RADIO_TAG, "SPI flash did not become ready");

  // Issue reset
  txn.cmd = BLOCK_FLASH_CMD_ENABLE_RESET;
  txn.flags = 0;
  txn.rxlength = 0;
  txn.length = 0;
  ESP_RETURN_ON_ERROR(spi_device_transmit(device, &txn), RADIO_TAG, "Failed to enable reset on SPI flash");
  txn.cmd = BLOCK_FLASH_CMD_RESET;
  ESP_RETURN_ON_ERROR(spi_device_transmit(device, &txn), RADIO_TAG, "Failed to reset SPI flash");

  // Wait for reset to complete
  ets_delay_us(50);

  flash->ready = false;
  ESP_RETURN_ON_ERROR(flash_wait_ready(flash, pdMS_TO_TICKS(1)), RADIO_TAG, "SPI flash did not become ready after reset");

  // Check quad enable bit
  txn.cmd = BLOCK_FLASH_CMD_READ_STATUS_2;
  txn.flags = SPI_TRANS_USE_RXDATA;
  txn.length = 0;
  txn.rxlength = 8;
  ESP_RETURN_ON_ERROR(spi_device_transmit(device, &txn), RADIO_TAG, "Failed to read SPI flash status register 2");

  block_flash_status_reg2 reg2 = {.raw = txn.rx_data[0]};
  if (!reg2.parsed.quad_enable)
  {
    ESP_LOGI(RADIO_TAG, "SPI flash chip present, quad mode not enabled");

    flash->ready = false;

    txn.cmd = BLOCK_FLASH_CMD_WRITE_ENABLE_VOLATILE;
    txn.flags = 0;
    txn.length = 0;
    txn.rxlength = 0;
    ESP_RETURN_ON_ERROR(spi_device_transmit(device, &txn), RADIO_TAG, "Failed to enable register write on SPI flash");

    reg2.parsed.quad_enable = 1;
    txn.cmd = BLOCK_FLASH_CMD_WRITE_STATUS_2;
    txn.flags = SPI_TRANS_USE_TXDATA;
    txn.tx_data[0] = reg2.raw;
    txn.length = 8;
    txn.rxlength = 0;
    ESP_RETURN_ON_ERROR(spi_device_transmit(device, &txn), RADIO_TAG, "Failed to enable quad mode on SPI flash");

    ESP_RETURN_ON_ERROR(flash_wait_ready(flash, pdMS_TO_TICKS(1)), RADIO_TAG, "SPI flash did not become ready after enabling quad mode");
  }

  flash->mutex = xSemaphoreCreateMutex();
  if (!flash->mutex)
  {
    return ESP_ERR_NO_MEM;
  }

  return ESP_OK;
}

// Reads can be of any length
static esp_err_t flash_read(block_flash_t *flash, size_t addr, void *buffer, size_t size)
{
  if (!flash || !buffer || size == 0)
  {
    return ESP_ERR_INVALID_ARG;
  }

  if (addr + size < addr || addr + size > BLOCK_FLASH_SIZE_BYTES)
  {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = ESP_OK;

  xSemaphoreTake(flash->mutex, portMAX_DELAY);
  ESP_GOTO_ON_ERROR(flash_wait_ready(flash, portMAX_DELAY), cleanup, RADIO_TAG, "SPI flash did not become ready");

  spi_transaction_ext_t txn = {
      .base = {
          .cmd = BLOCK_FLASH_CMD_READ_FAST_QUAD_IO,
          .addr = addr << 8 | 0xff,
          .flags = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY | SPI_TRANS_MODE_QIO | SPI_TRANS_MODE_DIOQIO_ADDR,
          .rxlength = size * 8,
          .rx_buffer = buffer,
      },
      .address_bits = 32,
      // Note I'm pretty sure that this is actually clock cycles, not bits
      .dummy_bits = 4,
  };
  ESP_GOTO_ON_ERROR(spi_device_transmit(flash->handle, &txn.base), cleanup, RADIO_TAG,
                    "Failed to read from SPI flash: addr 0x%08zx, size %zu",
                    addr, size);

cleanup:
  xSemaphoreGive(flash->mutex);
  return ret;
}

// Writes must fit within a single 256-byte page
static esp_err_t flash_write(block_flash_t *flash, size_t addr, const void *buffer, size_t size)
{
  if (!flash || !buffer || size == 0)
  {
    return ESP_ERR_INVALID_ARG;
  }

  if (
      addr + size < addr ||
      addr + size > BLOCK_FLASH_SIZE_BYTES ||
      (addr & ~0xff) != ((addr + size - 1) & ~0xff))
  {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = ESP_OK;
  xSemaphoreTake(flash->mutex, portMAX_DELAY);
  ESP_GOTO_ON_ERROR(flash_wait_ready(flash, portMAX_DELAY), cleanup, RADIO_TAG, "SPI flash did not become ready");
  ESP_GOTO_ON_ERROR(flash_enable_write(flash), cleanup, RADIO_TAG, "Failed to enable write on SPI flash");

  // Set ready=false so the next read/write will wait for the busy bit to clear
  flash->ready = false;

  spi_transaction_ext_t txn = {
      .base = {
          .cmd = BLOCK_FLASH_CMD_WRITE_QUAD,
          .addr = addr,
          .tx_buffer = buffer,
          .length = size * 8,
          .flags = SPI_TRANS_MODE_QIO | SPI_TRANS_VARIABLE_ADDR,
      },
      .address_bits = 24,
  };
  ESP_GOTO_ON_ERROR(spi_device_transmit(flash->handle, &txn.base), cleanup, RADIO_TAG,
                    "Failed to write to SPI flash: addr 0x%08zx, size %zu",
                    addr, size);

  // Typical page program time is 0.4ms with a max of 3ms
  ets_delay_us(400);
  ESP_GOTO_ON_ERROR(flash_wait_ready(flash, pdMS_TO_TICKS(4)), cleanup, RADIO_TAG, "SPI flash did not become ready after write");

cleanup:
  xSemaphoreGive(flash->mutex);
  return ret;
}

static esp_err_t flash_erase(block_flash_t *flash, size_t addr)
{
  if (!flash)
  {
    return ESP_ERR_INVALID_ARG;
  }

  if (addr > BLOCK_FLASH_SIZE_BYTES || addr % BLOCK_FLASH_ERASE_SIZE != 0)
  {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = ESP_OK;
  xSemaphoreTake(flash->mutex, portMAX_DELAY);
  ESP_GOTO_ON_ERROR(flash_wait_ready(flash, portMAX_DELAY), cleanup, RADIO_TAG, "SPI flash did not become ready");
  ESP_GOTO_ON_ERROR(flash_enable_write(flash), cleanup, RADIO_TAG, "Failed to enable write on SPI flash");

  // Set ready=false so the next read/write will wait for the busy bit to clear
  flash->ready = false;

  spi_transaction_ext_t txn = {
      .base = {
          .cmd = BLOCK_FLASH_CMD_ERASE_SECTOR,
          .addr = addr,
          .flags = SPI_TRANS_VARIABLE_ADDR,
      },
      .address_bits = 24,
  };
  ESP_GOTO_ON_ERROR(spi_device_transmit(flash->handle, &txn.base), cleanup, RADIO_TAG,
                    "Failed to erase SPI flash: addr 0x%08zx",
                    addr);

  // Typical sector erase time is 45ms with max of 400ms
  vTaskDelay(pdMS_TO_TICKS(45));
  ESP_GOTO_ON_ERROR(flash_wait_ready(flash, pdMS_TO_TICKS(400)), cleanup, RADIO_TAG, "SPI flash did not become ready after erase");

cleanup:
  xSemaphoreGive(flash->mutex);
  return ret;
}

esp_err_t storage_init(void)
{
  storage_mounted_mutex = xSemaphoreCreateMutex();
  ESP_RETURN_ON_FALSE(storage_mounted_mutex != NULL, ESP_ERR_NO_MEM, RADIO_TAG, "Failed to create storage mutex");

  spi_bus_config_t spi_bus_config = {
      .mosi_io_num = RADIO_SPI_PIN_MOSI,
      .miso_io_num = RADIO_SPI_PIN_MISO,
      .data2_io_num = RADIO_SPI_PIN_D2,
      .data3_io_num = RADIO_SPI_PIN_D3,
      .data4_io_num = GPIO_NUM_NC,
      .data5_io_num = GPIO_NUM_NC,
      .data6_io_num = GPIO_NUM_NC,
      .data7_io_num = GPIO_NUM_NC,
      .sclk_io_num = RADIO_SPI_PIN_CLK,
      .isr_cpu_id = ESP_INTR_CPU_AFFINITY_0,
      .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD,
  };
  ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO), RADIO_TAG, "Failed to initialize SPI bus for SPI flash");

  // Attempt to talk to the W25Q128JV flash chips
  spi_device_interface_config_t dev_cfg = {
      // We can override these later with SPI_TRANS_VARIABLE_* flags
      .command_bits = 8,
      .address_bits = 0,
      .dummy_bits = 0,
      .spics_io_num = RADIO_SPI_PIN_CS1,
      .clock_speed_hz = 20000000,
      .queue_size = 1,
      .flags = SPI_DEVICE_HALFDUPLEX,
  };
  spi_device_handle_t device;
  ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &dev_cfg, &device), RADIO_TAG, "Failed to initialize SPI flash");

  esp_err_t err = flash_init(&block_flash[0], device);
  if (err == ESP_OK)
  {
    // We have flash, initialize the other one
    dev_cfg.spics_io_num = RADIO_SPI_PIN_CS2;
    ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST, &dev_cfg, &device), RADIO_TAG, "Failed to initialize second SPI bus device");
    ESP_RETURN_ON_ERROR(flash_init(&block_flash[1], device), RADIO_TAG, "Failed to initialize second SPI flash");
    block_type = BLOCK_FLASH;
    return ESP_OK;
  }
  else if (err != ESP_ERR_NOT_FOUND)
  {
    return err;
  }

  // Otherwise, we don't have flash chips, so tear down the SPI infrastructure
  // and try again with microSD
  ESP_RETURN_ON_ERROR(spi_bus_remove_device(device), RADIO_TAG, "Failed to remove non-present SPI flash");
  ESP_RETURN_ON_ERROR(spi_bus_free(SPI2_HOST), RADIO_TAG, "Failed to deinitialize SPI bus");

  spi_bus_config.data2_io_num = GPIO_NUM_NC;
  spi_bus_config.data3_io_num = GPIO_NUM_NC;
  spi_bus_config.flags = SPICOMMON_BUSFLAG_MASTER;
  ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO), RADIO_TAG, "Failed to initialize SPI bus for microSD");
  ESP_RETURN_ON_ERROR(sdspi_host_init(), RADIO_TAG, "Failed to initialize SD SPI driver");
  sdspi_device_config_t sdspi_device_config = {
      .gpio_cd = RADIO_SPI_PIN_SD_CD,
      .gpio_cs = RADIO_SPI_PIN_SD_CS,
      .gpio_int = GPIO_NUM_NC,
      .gpio_wp = GPIO_NUM_NC,
      .host_id = SPI2_HOST,
  };
  sdspi_dev_handle_t block_sdspi_handle;
  ESP_RETURN_ON_ERROR(sdspi_host_init_device(&sdspi_device_config, &block_sdspi_handle), RADIO_TAG, "Failed to initialize SD SPI device");

  sdmmc_host_t block_sdmmc_host = SDSPI_HOST_DEFAULT();
  block_sdmmc_host.slot = block_sdspi_handle;
  ESP_RETURN_ON_ERROR(sdmmc_card_init(&block_sdmmc_host, &block_sdmmc_card), RADIO_TAG, "Failed to initialize SD card");

  ESP_LOGI(RADIO_TAG, "Detected SD card with %d sectors of size %d", block_sdmmc_card.csd.capacity, block_sdmmc_card.csd.sector_size);
  block_type = BLOCK_SD_CARD;

  return ESP_OK;
}

int block_read(void *context, uint32_t block, uint32_t offset, void *buffer, size_t size)
{
  // Need to split the read across chips if it crosses a chip boundary
  size_t start_addr = (size_t)block * BLOCK_FLASH_ERASE_SIZE + offset;
  size_t end_addr = start_addr + size;

  if (end_addr < start_addr || end_addr > BLOCK_FLASH_SIZE_BYTES * BLOCK_FLASH_COUNT)
  {
    return -EINVAL;
  }

  int first_chip = start_addr / BLOCK_FLASH_SIZE_BYTES;
  int last_chip = (end_addr - 1) / BLOCK_FLASH_SIZE_BYTES;

  for (int idx = first_chip; idx <= last_chip; idx++)
  {
    block_flash_t *flash = &block_flash[idx];
    size_t flash_start = idx * BLOCK_FLASH_SIZE_BYTES;
    size_t flash_end = flash_start + BLOCK_FLASH_SIZE_BYTES;

    size_t read_start = start_addr > flash_start ? start_addr : flash_start;
    size_t read_end = end_addr < flash_end ? end_addr : flash_end;
    size_t read_size = read_end - read_start;

    esp_err_t ret = flash_read(flash, read_start % BLOCK_FLASH_SIZE_BYTES, buffer, read_size);
    if (ret != ESP_OK)
    {
      ESP_LOGE(RADIO_TAG, "Failed to read from SPI flash (idx=%d addr=%08zx size=%08zx): %s",
               idx, read_start, read_size, esp_err_to_name(ret));
      return -EIO;
    }

    buffer += read_size;
  }

  return 0;
}

static int block_write(void *context, uint32_t block, uint32_t offset, const void *buffer, size_t size)
{
  // Need to split writes into 256-byte pages and also potentially across chips
  size_t start_addr = (size_t)block * BLOCK_FLASH_ERASE_SIZE + offset;
  size_t end_addr = start_addr + size;

  if (end_addr < start_addr || end_addr > BLOCK_FLASH_SIZE_BYTES * BLOCK_FLASH_COUNT)
  {
    return -EINVAL;
  }

  int first_chip = start_addr / BLOCK_FLASH_SIZE_BYTES;
  int last_chip = (end_addr - 1) / BLOCK_FLASH_SIZE_BYTES;

  for (int idx = first_chip; idx <= last_chip; idx++)
  {
    block_flash_t *flash = &block_flash[idx];
    size_t flash_start = idx * BLOCK_FLASH_SIZE_BYTES;
    size_t flash_end = flash_start + BLOCK_FLASH_SIZE_BYTES;

    size_t flash_write_start = start_addr > flash_start ? start_addr : flash_start;
    size_t flash_write_end = end_addr < flash_end ? end_addr : flash_end;

    size_t first_sector = flash_write_start / BLOCK_FLASH_WRITE_MAX_SIZE;
    size_t last_sector = (flash_write_end - 1) / BLOCK_FLASH_WRITE_MAX_SIZE;

    for (int sector = first_sector; sector <= last_sector; sector++)
    {
      size_t sector_start = sector * BLOCK_FLASH_WRITE_MAX_SIZE;
      size_t sector_end = sector_start + BLOCK_FLASH_WRITE_MAX_SIZE;

      size_t write_start = flash_write_start > sector_start ? flash_write_start : sector_start;
      size_t write_end = flash_write_end < sector_end ? flash_write_end : sector_end;
      size_t write_size = write_end - write_start;

      esp_err_t ret = flash_write(flash, write_start % BLOCK_FLASH_SIZE_BYTES, buffer, write_size);
      if (ret != ESP_OK)
      {
        ESP_LOGE(RADIO_TAG, "Failed to write to SPI flash (idx=%d addr=%08zx size=%08zx): %s",
                 idx, write_start, write_size, esp_err_to_name(ret));
        return -EIO;
      }

      buffer += write_size;
    }
  }

  return 0;
}

static int block_erase(void *context, uint32_t block)
{
  if (block >= BLOCK_FLASH_BLOCK_COUNT * BLOCK_FLASH_COUNT)
  {
    return -EINVAL;
  }

  size_t addr = block * BLOCK_FLASH_ERASE_SIZE;
  esp_err_t err;
  if (block < BLOCK_FLASH_BLOCK_COUNT)
  {
    err = flash_erase(&block_flash[0], addr);
  }
  else
  {
    err = flash_erase(&block_flash[1], addr - BLOCK_FLASH_SIZE_BYTES);
  }

  if (err != ESP_OK)
  {
    ESP_LOGE(RADIO_TAG, "Failed to erase SPI flash block %" PRIx32 ": %d (%s)", block, err, esp_err_to_name(err));
    return -EIO;
  }

  return 0;
}

static int block_sync(void *context)
{
  return 0;
}

esp_err_t storage_mount(bool format)
{
  if (storage_mounted)
  {
    return ESP_ERR_INVALID_STATE;
  }

  esp_vfs_littlefs_conf_t conf = {
      .base_path = "/data",
      .format_if_mount_failed = format,
  };
  esp_vfs_littlefs_custom_conf_t custom = {
      .read = block_read,
      .write = block_write,
      .erase = block_erase,
      .sync = block_sync,
      .read_size = 1,
      .write_size = 1,
      .block_size = BLOCK_FLASH_ERASE_SIZE,
      .block_count = BLOCK_FLASH_BLOCK_COUNT * BLOCK_FLASH_COUNT,
  };
  switch (block_type)
  {
  case BLOCK_FLASH:
    // Our patch to esp_littlefs copies the custom config, so we can use a stack
    // variable
    conf.custom = &custom;
    break;
  case BLOCK_SD_CARD:
    conf.sdcard = &block_sdmmc_card;
    break;
  default:
    return ESP_ERR_INVALID_STATE;
  }

  xSemaphoreTake(storage_mounted_mutex, portMAX_DELAY);
  if (storage_mounted)
  {
    xSemaphoreGive(storage_mounted_mutex);
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t err = esp_vfs_littlefs_register(&conf);
  if (err == ESP_OK)
  {
    storage_mounted = true;
  }
  xSemaphoreGive(storage_mounted_mutex);
  return err;
}

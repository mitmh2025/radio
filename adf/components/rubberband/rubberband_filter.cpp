#include "rubberband_filter.h"

#include "esp_log.h"
#include "esp_check.h"
#include "audio_error.h"
#include "audio_mem.h"
#include "RubberBandStretcher.h"

using namespace RubberBand;

static const char *TAG = "rubberband_filter";

typedef struct rubberband_filter
{
  int bits;
  int sample_rate;
  int channels;
  bool changed;
  RubberBandStretcher *stretcher;
  float *buffer;
  size_t buffer_size;
} rubberband_filter_t;

static esp_err_t rubberband_filter_destroy(audio_element_handle_t self)
{
  rubberband_filter_t *filter = (rubberband_filter_t *)audio_element_getdata(self);
  if (!filter)
  {
    return ESP_OK;
  }

  if (filter->stretcher)
  {
    delete filter->stretcher;
    filter->stretcher = NULL;
  }

  audio_free(filter);
  return ESP_OK;
}

static esp_err_t rubberband_filter_open(audio_element_handle_t self)
{
  rubberband_filter_t *filter = (rubberband_filter_t *)audio_element_getdata(self);
  filter->stretcher = new RubberBandStretcher(
      filter->sample_rate,
      filter->channels,
      RubberBandStretcher::OptionProcessRealTime |
          RubberBandStretcher::OptionThreadingNever |
          RubberBandStretcher::OptionEngineFaster |
          RubberBandStretcher::OptionChannelsTogether |
          RubberBandStretcher::OptionPitchHighSpeed |
          RubberBandStretcher::OptionWindowShort);
  if (filter->stretcher == NULL)
  {
    ESP_LOGE(TAG, "Failed to create the rubberband stretcher");
    return ESP_FAIL;
  }

  return ESP_OK;
}

static esp_err_t rubberband_filter_close(audio_element_handle_t self)
{
  rubberband_filter_t *filter = (rubberband_filter_t *)audio_element_getdata(self);
  if (filter->stretcher)
  {
    delete filter->stretcher;
    filter->stretcher = NULL;
  }
  return ESP_OK;
}

static bool _rubberband_resize_buffer(rubberband_filter_t *filter, size_t size)
{
  if (filter->buffer_size < size)
  {
    if (filter->buffer)
    {
      free(filter->buffer);
    }
    filter->buffer = (float *)audio_calloc(size, sizeof(float));
    if (filter->buffer == NULL)
    {
      return false;
    }
    filter->buffer_size = size;
  }
  return true;
}

static audio_element_err_t rubberband_filter_process(audio_element_handle_t self, char *in_buffer, int in_len)
{
  rubberband_filter_t *filter = (rubberband_filter_t *)audio_element_getdata(self);
  if (filter->changed)
  {
    esp_err_t ret = rubberband_filter_close(self);
    if (ret != ESP_OK)
    {
      return AEL_PROCESS_FAIL;
    }
    ret = rubberband_filter_open(self);
    if (ret != ESP_OK)
    {
      return AEL_PROCESS_FAIL;
    }
    filter->changed = false;
  }

  audio_element_err_t read_len = audio_element_input(self, in_buffer, in_len);
  if (read_len < 0)
  {
    return read_len;
  }

  // Need to convert the input buffer to float
  int samples = read_len / (filter->bits >> 3);
  AUDIO_MEM_CHECK(TAG, _rubberband_resize_buffer(filter, samples), return AEL_IO_FAIL);

  // Convert the input buffer from int to float
  for (int i = 0; i < samples; i++)
  {
    if (filter->bits == 16)
    {
      filter->buffer[i] = ((int16_t *)in_buffer)[i] / 32768.0f;
    }
    else if (filter->bits == 32)
    {
      filter->buffer[i] = ((int32_t *)in_buffer)[i] / 2147483648.0f;
    }
  }

  // Process the audio - note that rubberband wants audio samples (i.e. divided
  // by channels) not total samples
  filter->stretcher->process(&filter->buffer, samples / filter->channels, false);

  int available = filter->stretcher->available();
  if (available <= 0)
  {
    return AEL_IO_OK;
  }

  AUDIO_MEM_CHECK(TAG, _rubberband_resize_buffer(filter, available * filter->channels), return AEL_IO_FAIL);
  filter->stretcher->retrieve(&filter->buffer, available);

  // Need to convert back to int, but we should have enough space to do it in place
  for (int i = 0; i < available * filter->channels; i++)
  {
    if (filter->bits == 16)
    {
      ((int16_t *)in_buffer)[i] = filter->buffer[i] * 32768.0f;
    }
    else if (filter->bits == 32)
    {
      ((int32_t *)in_buffer)[i] = filter->buffer[i] * 2147483648.0f;
    }
  }

  return audio_element_output(self, in_buffer, available * filter->channels * (filter->bits >> 3));
}

static esp_err_t _rubberband_validate_config(rubberband_filter_cfg_t *config)
{
  if (config->sample_rate < 8000 || config->sample_rate > 192000)
  {
    ESP_LOGE(TAG, "Invalid sample rate");
    return ESP_ERR_INVALID_ARG;
  }
  if (config->channels != 1 && config->channels != 2)
  {
    ESP_LOGE(TAG, "Invalid channels");
    return ESP_ERR_INVALID_ARG;
  }
  if (config->bits != 16 && config->bits != 32)
  {
    ESP_LOGE(TAG, "Invalid bits");
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

audio_element_handle_t rubberband_filter_init(rubberband_filter_cfg_t *config)
{
  if (config == NULL)
  {
    ESP_LOGE(TAG, "rubberband config is NULL");
    return NULL;
  }
  if (_rubberband_validate_config(config) != ESP_OK)
  {
    ESP_LOGE(TAG, "Invalid rubberband config");
    return NULL;
  }
  rubberband_filter_t *filter = (rubberband_filter_t *)audio_calloc(1, sizeof(rubberband_filter_t));
  AUDIO_MEM_CHECK(TAG, filter, return NULL);
  filter->bits = config->bits;
  filter->sample_rate = config->sample_rate;
  filter->channels = config->channels;
  audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
  cfg.open = rubberband_filter_open;
  cfg.close = rubberband_filter_close;
  cfg.destroy = rubberband_filter_destroy;
  cfg.process = rubberband_filter_process;
  cfg.task_stack = config->task_stack;
  cfg.task_prio = config->task_prio;
  cfg.task_core = config->task_core;
  cfg.out_rb_size = config->out_rb_size;
  cfg.stack_in_ext = config->stack_in_ext;
  cfg.tag = "rubberband";
  audio_element_handle_t el = audio_element_init(&cfg);
  AUDIO_MEM_CHECK(TAG, el, {audio_free(filter); return NULL; });
  audio_element_setdata(el, filter);
  audio_element_info_t info = {0};
  audio_element_setinfo(el, &info);
  RubberBand::RubberBandStretcher::setDefaultDebugLevel(2);
  ESP_LOGD(TAG, "rubberband_filter_init");
  return el;
}

esp_err_t rubberband_filter_change_src_info(audio_element_handle_t self, int sample_rates, int channels, int bits)
{
  rubberband_filter_t *filter = (rubberband_filter_t *)audio_element_getdata(self);
  if (!filter)
  {
    return ESP_FAIL;
  }
  filter->sample_rate = sample_rates;
  filter->channels = channels;
  filter->bits = bits;
  filter->changed = true;
  return ESP_OK;
}
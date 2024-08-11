#include "soundtouch_filter.h"

#include "esp_log.h"
#include "esp_check.h"
#include "audio_error.h"
#include "audio_mem.h"
#include "SoundTouch.h"

using namespace soundtouch;

static const char *TAG = "soundtouch_filter";

typedef struct soundtouch_filter
{
  int sample_rate;
  int channels;
  bool changed;
  SoundTouch *stretcher;
} soundtouch_filter_t;

static esp_err_t _soundtouch_validate_config(int sample_rate, int bits, int channels)
{
  if (sample_rate < 8000 || sample_rate > 48000)
  {
    ESP_LOGE(TAG, "Invalid sample rate");
    return ESP_ERR_INVALID_ARG;
  }
  if (bits != 16)
  {
    ESP_LOGE(TAG, "Invalid bits");
    return ESP_ERR_INVALID_ARG;
  }
  return ESP_OK;
}

static esp_err_t soundtouch_filter_destroy(audio_element_handle_t self)
{
  soundtouch_filter_t *filter = (soundtouch_filter_t *)audio_element_getdata(self);
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

static esp_err_t soundtouch_filter_open(audio_element_handle_t self)
{
  soundtouch_filter_t *filter = (soundtouch_filter_t *)audio_element_getdata(self);
  filter->stretcher = new SoundTouch();
  if (filter->stretcher == NULL)
  {
    ESP_LOGE(TAG, "Failed to create the soundtouch stretcher");
    return ESP_FAIL;
  }
  filter->stretcher->setSampleRate(filter->sample_rate);
  filter->stretcher->setChannels(filter->channels);
  filter->stretcher->setSetting(SETTING_USE_QUICKSEEK, 1);
  filter->stretcher->setSetting(SETTING_USE_AA_FILTER, 0);
  filter->stretcher->setSetting(SETTING_SEQUENCE_MS, 40);
  filter->stretcher->setSetting(SETTING_SEEKWINDOW_MS, 15);
  filter->stretcher->setSetting(SETTING_OVERLAP_MS, 4);

  return ESP_OK;
}

static esp_err_t soundtouch_filter_close(audio_element_handle_t self)
{
  soundtouch_filter_t *filter = (soundtouch_filter_t *)audio_element_getdata(self);
  if (filter->stretcher)
  {
    delete filter->stretcher;
    filter->stretcher = NULL;
  }
  return ESP_OK;
}

static audio_element_err_t soundtouch_filter_process(audio_element_handle_t self, char *in_buffer, int in_len)
{
  soundtouch_filter_t *filter = (soundtouch_filter_t *)audio_element_getdata(self);
  if (filter->changed)
  {
    esp_err_t ret = soundtouch_filter_close(self);
    if (ret != ESP_OK)
    {
      return AEL_PROCESS_FAIL;
    }
    ret = soundtouch_filter_open(self);
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

  filter->stretcher->putSamples((const SAMPLETYPE *)in_buffer, read_len / 2 / filter->channels);

  int num_samples = 0;
  do
  {
    num_samples = filter->stretcher->receiveSamples((SAMPLETYPE *)in_buffer, in_len / 2 / filter->channels);
    if (num_samples < 0)
    {
      break;
    }
    audio_element_err_t err = audio_element_output(self, in_buffer, num_samples * 2 * filter->channels);
    if (err < 0)
    {
      return err;
    }
  } while (num_samples > 0);

  return read_len;
}

audio_element_handle_t soundtouch_filter_init(soundtouch_filter_cfg_t *config)
{
  if (config == NULL)
  {
    ESP_LOGE(TAG, "soundtouch config is NULL");
    return NULL;
  }
  if (_soundtouch_validate_config(config->sample_rate, config->bits, config->channels) != ESP_OK)
  {
    ESP_LOGE(TAG, "Invalid soundtouch config");
    return NULL;
  }
  soundtouch_filter_t *filter = (soundtouch_filter_t *)audio_calloc(1, sizeof(soundtouch_filter_t));
  AUDIO_MEM_CHECK(TAG, filter, return NULL);
  filter->sample_rate = config->sample_rate;
  filter->channels = config->channels;
  audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
  cfg.open = soundtouch_filter_open;
  cfg.close = soundtouch_filter_close;
  cfg.destroy = soundtouch_filter_destroy;
  cfg.process = soundtouch_filter_process;
  cfg.buffer_len = config->buffer_len;
  cfg.task_stack = config->task_stack;
  cfg.task_prio = config->task_prio;
  cfg.task_core = config->task_core;
  cfg.out_rb_size = config->out_rb_size;
  cfg.stack_in_ext = config->stack_in_ext;
  cfg.tag = "soundtouch";
  audio_element_handle_t el = audio_element_init(&cfg);
  AUDIO_MEM_CHECK(TAG, el, {audio_free(filter); return NULL; });
  audio_element_setdata(el, filter);
  audio_element_info_t info = {0};
  audio_element_setinfo(el, &info);
  ESP_LOGD(TAG, "soundtouch_filter_init");
  return el;
}

esp_err_t soundtouch_filter_change_src_info(audio_element_handle_t self, int sample_rates, int channels, int bits)
{
  soundtouch_filter_t *filter = (soundtouch_filter_t *)audio_element_getdata(self);
  if (!filter)
  {
    return ESP_FAIL;
  }
  if (_soundtouch_validate_config(sample_rates, bits, channels) != ESP_OK)
  {
    return ESP_ERR_INVALID_ARG;
  }
  filter->sample_rate = sample_rates;
  filter->channels = channels;
  filter->changed = true;
  return ESP_OK;
}

esp_err_t soundtouch_filter_set_tempo(audio_element_handle_t self, double tempo)
{
  soundtouch_filter_t *filter = (soundtouch_filter_t *)audio_element_getdata(self);
  if (!filter)
  {
    return ESP_FAIL;
  }
  if (filter->stretcher)
  {
    filter->stretcher->setTempo(tempo);
  }
  return ESP_OK;
}

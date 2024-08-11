#include "stretch_filter.h"

#include <atomic>
#include <memory>

#include "esp_log.h"
#include "esp_check.h"
#include "audio_error.h"
#include "audio_mem.h"

#include "signalsmith-stretch.h"

using namespace signalsmith::stretch;

static const char *TAG = "stretch_filter";

struct StretchFilter
{
  StretchFilter() : sample_rate(0), channels(0), bits(0), changed(false), time_ratio(1.0f), out_buffer(nullptr), out_buffer_len(0) {};
  int sample_rate;
  int channels;
  int bits;
  bool changed;
  std::atomic<float> time_ratio;
  std::unique_ptr<SignalsmithStretch<float>> algo;
  std::vector<std::vector<float>> in_buffer;
  std::vector<std::vector<float>> out_float_buffer;
  char *out_buffer;
  size_t out_buffer_len;
};

static esp_err_t _stretch_validate_config(int sample_rate, int bits, int channels)
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

static esp_err_t stretch_filter_destroy(audio_element_handle_t self)
{
  StretchFilter *filter = (StretchFilter *)audio_element_getdata(self);
  if (!filter)
  {
    return ESP_OK;
  }

  delete filter;
  return ESP_OK;
}

static esp_err_t stretch_filter_open(audio_element_handle_t self)
{
  StretchFilter *filter = (StretchFilter *)audio_element_getdata(self);
  filter->algo.reset(new SignalsmithStretch());
  filter->algo->presetCheaper(filter->channels, filter->sample_rate);
  filter->in_buffer.resize(filter->channels);
  filter->out_float_buffer.resize(filter->channels);

  return ESP_OK;
}

static esp_err_t stretch_filter_close(audio_element_handle_t self)
{
  StretchFilter *filter = (StretchFilter *)audio_element_getdata(self);
  filter->algo.reset();
  return ESP_OK;
}

static audio_element_err_t stretch_filter_process(audio_element_handle_t self, char *in_buffer, int in_len)
{
  StretchFilter *filter = (StretchFilter *)audio_element_getdata(self);
  if (filter->changed)
  {
    esp_err_t ret = stretch_filter_close(self);
    if (ret != ESP_OK)
    {
      return AEL_PROCESS_FAIL;
    }
    ret = stretch_filter_open(self);
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

  size_t in_frames = read_len / (filter->channels * filter->bits / 8);
  // Need to convert the input buffer to float and separate the interleaved channels
  for (int i = 0; i < filter->channels; i++)
  {
    filter->in_buffer[i].resize(in_frames);
    for (size_t j = 0; j < in_frames; j++)
    {
      size_t idx = j * filter->channels + i;
      if (filter->bits == 16)
      {
        int16_t *data = (int16_t *)in_buffer;
        filter->in_buffer[i][j] = data[idx] / 32768.0f;
      }
    }
  }

  // Resize output buffer based on the time ratio
  float time_ratio = filter->time_ratio.load();
  size_t out_frames = (size_t)(time_ratio * in_frames);
  for (auto &buf : filter->out_float_buffer)
  {
    buf.reserve(out_frames);
  }

  // Process the stretch algorithm
  filter->algo->process(filter->in_buffer, in_frames, filter->out_float_buffer, out_frames);

  // Convert the output buffer back to interleaved format
  size_t out_len = out_frames * filter->channels * filter->bits / 8;
  if (filter->out_buffer_len < out_len)
  {
    if (filter->out_buffer)
    {
      free(filter->out_buffer);
    }
    filter->out_buffer = (char *)audio_calloc(1, out_len);
    if (!filter->out_buffer)
    {
      ESP_LOGE(TAG, "Failed to allocate output buffer");
      return AEL_PROCESS_FAIL;
    }
    filter->out_buffer_len = out_len;
  }

  for (int i = 0; i < filter->channels; i++)
  {
    for (size_t j = 0; j < out_frames; j++)
    {
      size_t idx = j * filter->channels + i;
      if (filter->bits == 16)
      {
        int16_t *data = (int16_t *)filter->out_buffer;
        data[idx] = filter->out_float_buffer[i][j] * 32768.0f;
      }
    }
  }

  return audio_element_output(self, filter->out_buffer, out_len);
}

audio_element_handle_t stretch_filter_init(stretch_filter_cfg_t *config)
{
  if (config == NULL)
  {
    ESP_LOGE(TAG, "stretch config is NULL");
    return NULL;
  }
  if (_stretch_validate_config(config->sample_rate, config->bits, config->channels) != ESP_OK)
  {
    ESP_LOGE(TAG, "Invalid stretch config");
    return NULL;
  }
  StretchFilter *filter = new StretchFilter();
  AUDIO_MEM_CHECK(TAG, filter, return NULL);
  filter->sample_rate = config->sample_rate;
  filter->channels = config->channels;
  filter->bits = config->bits;
  audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
  cfg.open = stretch_filter_open;
  cfg.close = stretch_filter_close;
  cfg.destroy = stretch_filter_destroy;
  cfg.process = stretch_filter_process;
  cfg.buffer_len = config->buffer_len;
  cfg.task_stack = config->task_stack;
  cfg.task_prio = config->task_prio;
  cfg.task_core = config->task_core;
  cfg.out_rb_size = config->out_rb_size;
  cfg.stack_in_ext = config->stack_in_ext;
  cfg.tag = "stretch";
  audio_element_handle_t el = audio_element_init(&cfg);
  AUDIO_MEM_CHECK(TAG, el, {delete filter; return NULL; });
  audio_element_setdata(el, filter);
  audio_element_info_t info = {0};
  audio_element_setinfo(el, &info);
  ESP_LOGD(TAG, "stretch_filter_init");
  return el;
}

esp_err_t stretch_filter_change_src_info(audio_element_handle_t self, int sample_rates, int channels, int bits)
{
  StretchFilter *filter = (StretchFilter *)audio_element_getdata(self);
  if (!filter)
  {
    return ESP_FAIL;
  }
  if (_stretch_validate_config(sample_rates, bits, channels) != ESP_OK)
  {
    return ESP_ERR_INVALID_ARG;
  }
  filter->sample_rate = sample_rates;
  filter->channels = channels;
  filter->bits = bits;
  filter->changed = true;
  return ESP_OK;
}

esp_err_t stretch_filter_set_time_ratio(audio_element_handle_t self, float time_ratio)
{
  StretchFilter *filter = (StretchFilter *)audio_element_getdata(self);
  if (!filter)
  {
    return ESP_FAIL;
  }
  filter->time_ratio.store(time_ratio);
  return ESP_OK;
}

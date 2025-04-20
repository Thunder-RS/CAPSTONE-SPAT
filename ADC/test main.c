#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(saadc_multi_fullrange, LOG_LEVEL_INF);

#define ADC_NODE DT_NODELABEL(adc)

#define ADC_RESOLUTION 12
#define NUM_CHANNELS   3
#define BUFFER_SIZE    NUM_CHANNELS

#define VDD_VOLTAGE    3.3f
#define ADC_MAX        4095
#define ADC_GAIN       (1.0f / 6.0f)
#define ADC_REFERENCE  (VDD_VOLTAGE / 4.0f)
#define ADC_FULL_SCALE (ADC_REFERENCE / ADC_GAIN) // 4.95V

static const uint8_t channels[NUM_CHANNELS] = {0, 1, 2}; // AIN0, AIN1, AIN2
static int16_t sample_buffer[BUFFER_SIZE];
static struct adc_channel_cfg adc_cfg[NUM_CHANNELS];

static struct adc_sequence sequence = {
    .channels    = 0,
    .buffer      = sample_buffer,
    .buffer_size = sizeof(sample_buffer),
    .resolution  = ADC_RESOLUTION,
};

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

float convert_to_voltage(int16_t raw)
{
    return ((float)raw / ADC_MAX) * ADC_FULL_SCALE;
}

int main(void)
{
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return 0;
    }

    for (int i = 0; i < NUM_CHANNELS; i++) {
        adc_cfg[i] = (struct adc_channel_cfg){
            .gain             = ADC_GAIN_1_6,
            .reference        = ADC_REF_VDD_1_4,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id       = channels[i],
        #ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
            .input_positive   = SAADC_CH_PSELP_PSELP_AnalogInput0 + channels[i],
        #endif
        };

        int err = adc_channel_setup(adc_dev, &adc_cfg[i]);
        if (err) {
            LOG_ERR("Channel %d setup failed: %d", channels[i], err);
            return 0;
        }
        LOG_INF("Channel %d setup OK", channels[i]);
        sequence.channels |= BIT(channels[i]);
    }

    while (1) {
        int err = adc_read(adc_dev, &sequence);
        if (err) {
            LOG_ERR("ADC read failed: %d", err);
            k_sleep(K_MSEC(1000));
            continue;
        }

        for (int i = 0; i < NUM_CHANNELS; i++) {
            float voltage = convert_to_voltage(sample_buffer[i]);
            LOG_INF("AIN%d: raw = %d → voltage = %.3f V", channels[i], sample_buffer[i], voltage);
            if (voltage > 1.5f) {
                LOG_WRN("ALERT: High voltage detected on AIN%d: %.3f V", channels[i], voltage);
            }
        }

        k_sleep(K_MSEC(1000));
    }

    return 0;
}

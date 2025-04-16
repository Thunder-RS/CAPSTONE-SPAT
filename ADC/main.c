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

#define VDD_VOLTAGE    3.3f // VDD_GPIO assumed to be 3.3V
#define ADC_MAX        4095

// Analog input channels (AIN0, AIN1, AIN2 → check pin mappings)
static const uint8_t channels[NUM_CHANNELS] = {0, 1, 2};  // AIN0, AIN1, AIN2
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
    // Raw is two’s complement, positive values only for single-ended
    return ((float)raw / ADC_MAX) * VDD_VOLTAGE;
}

int main(void)
{
    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return 0;
    }

    for (int i = 0; i < NUM_CHANNELS; i++) {
        adc_cfg[i] = (struct adc_channel_cfg){
            .gain             = ADC_GAIN_1_6,
            .reference        = ADC_REF_VDD_1_4,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id       = channels[i],
        };
    
        int err = adc_channel_setup(adc_dev, &adc_cfg[i]);
        if (err) {
            printk("Channel %d setup failed: %d\n", channels[i], err);
            return 0;
        }
    
        sequence.channels |= BIT(channels[i]);
    }
    
    
    while (1) {
        int err = adc_read(adc_dev, &sequence);
        if (err) {
            printk("ADC read failed: %d\n", err);
        } else {
            for (int i = 0; i < NUM_CHANNELS; i++) {
                float voltage = convert_to_voltage(sample_buffer[i]);
                printk("AIN%d: raw = %d → voltage = %.3f V\n", channels[i], sample_buffer[i], (double)voltage);
            }
        }

        k_sleep(K_MSEC(1000));
    }

    return 0;
}


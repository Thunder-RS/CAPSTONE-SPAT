#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <string.h>

#define BUF_SIZE 64
#define TEMP_CMD "Temperature: "
#define HUMIDITY_CMD "Humidity: "
#define LIGHT_CMD "Light: "

static const struct device *uart_dev;
static char rx_buf[BUF_SIZE];
static int rx_buf_pos = 0;
static int temp_threshold = 25;    // Default threshold in Fahrenheit
static int humidity_threshold = 50; // Default threshold in percentage
static int light_threshold = 500;   // Default threshold in lux

static void send_response(const char *response) {
    for (int i = 0; response[i] != '\0'; i++) {
        uart_poll_out(uart_dev, response[i]);
        k_sleep(K_MSEC(1)); // Small delay between characters
    }
}

static void process_input(void) {
    // Null terminate the received data
    rx_buf[rx_buf_pos] = '\0';
    
    printk("DEBUG: Processing input: '%s'\n", rx_buf);
    
    // Check for temperature command
    if (strncmp(rx_buf, TEMP_CMD, strlen(TEMP_CMD)) == 0) {
        char *temp_str = rx_buf + strlen(TEMP_CMD);
        int new_temp = atoi(temp_str);
        
        printk("DEBUG: Extracted temperature value: %d\n", new_temp);
        temp_threshold = new_temp;
        printk("Temperature threshold updated to: %d F\n", temp_threshold);
        
        // Send response
        char response[64];
        snprintf(response, sizeof(response), "Temperature threshold set to: %d F\r\n", new_temp);
        send_response(response);
    }
    // Check for humidity command
    else if (strncmp(rx_buf, HUMIDITY_CMD, strlen(HUMIDITY_CMD)) == 0) {
        char *humidity_str = rx_buf + strlen(HUMIDITY_CMD);
        int new_humidity = atoi(humidity_str);
        
        printk("DEBUG: Extracted humidity value: %d\n", new_humidity);
        humidity_threshold = new_humidity;
        printk("Humidity threshold updated to: %d%%\n", humidity_threshold);
        
        // Send response
        char response[64];
        snprintf(response, sizeof(response), "Humidity threshold set to: %d%%\r\n", new_humidity);
        send_response(response);
    }
    // Check for light command
    else if (strncmp(rx_buf, LIGHT_CMD, strlen(LIGHT_CMD)) == 0) {
        char *light_str = rx_buf + strlen(LIGHT_CMD);
        int new_light = atoi(light_str);
        
        printk("DEBUG: Extracted light value: %d\n", new_light);
        light_threshold = new_light;
        printk("Light threshold updated to: %d lux\n", light_threshold);
        
        // Send response
        char response[64];
        snprintf(response, sizeof(response), "Light threshold set to: %d lux\r\n", new_light);
        send_response(response);
    }
    else {
        printk("DEBUG: Command not recognized\n");
        // Send current thresholds
        char response[128];
        snprintf(response, sizeof(response), 
                "Current thresholds:\r\n"
                "Temperature: %d F\r\n"
                "Humidity: %d%%\r\n"
                "Light: %d lux\r\n", 
                temp_threshold, humidity_threshold, light_threshold);
        send_response(response);
    }
    
    // Reset buffer position
    rx_buf_pos = 0;
}

void uart_cb(const struct device *dev, void *user_data) {
    uint8_t c;

    if (!uart_irq_update(dev)) {
        printk("DEBUG: No UART IRQ update\n");
        return;
    }

    if (uart_irq_rx_ready(dev)) {
        printk("DEBUG: UART RX ready\n");
        while (uart_fifo_read(dev, &c, 1) > 0) {
            printk("DEBUG: Received char (0x%02x)\n", c);
            if (rx_buf_pos < (BUF_SIZE - 1)) {
                if (c == '\n' || c == '\r') {
                    if (rx_buf_pos > 0) {
                        printk("DEBUG: End of line detected, processing input\n");
                        process_input();
                    }
                } else {
                    rx_buf[rx_buf_pos++] = c;
                    // Echo the character back
                    uart_poll_out(dev, c);
                    k_sleep(K_MSEC(1)); // Small delay between characters
                }
            } else {
                // Buffer full, process what we have
                printk("DEBUG: Buffer full, processing input\n");
                process_input();
            }
        }
    }
}

void main(void) {
    // Get UART device - use console UART for COM3
    uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (!device_is_ready(uart_dev)) {
        printk("ERROR: UART device not found!\n");
        return;
    }
    printk("DEBUG: UART device found and ready\n");

    // Set up callback
    int ret = uart_irq_callback_user_data_set(uart_dev, uart_cb, NULL);
    if (ret < 0) {
        printk("ERROR: Failed to set UART callback: %d\n", ret);
        return;
    }
    printk("DEBUG: UART callback set successfully\n");

    uart_irq_rx_enable(uart_dev);
    printk("DEBUG: UART RX interrupt enabled\n");

    printk("UART Serial Listener Started\n");
    printk("Default Thresholds:\n");
    printk("Temperature: %d F\n", temp_threshold);
    printk("Humidity: %d%%\n", humidity_threshold);
    printk("Light: %d lux\n", light_threshold);
    printk("Send commands in format:\n");
    printk("Temperature: XX F\n");
    printk("Humidity: XX%%\n");
    printk("Light: XX lux\n");
    
    while (1) {
        k_sleep(K_SECONDS(1));
    }
}

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <modem/lte_lc.h>
#include <zephyr/net/socket.h>
#include <net/nrf_cloud.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Sensor devices */
const struct device *bme280 = DEVICE_DT_GET_ANY(bosch_bme280);
const struct device *apds = DEVICE_DT_GET_ANY(avago_apds9960);

/* Function to initialize LTE and connect to nRF Cloud */
static int cloud_connect(void) {
    int err;

    LOG_INF("Initializing LTE...");
    // currently the problem 
    err = lte_lc_init_and_connect();
    if (err) {
        LOG_ERR("Failed to initialize LTE: %d", err);
        return err;
    }
    LOG_INF("Connected to LTE!");

    
    LOG_INF("Connecting to nRF Cloud...");
    err = nrf_cloud_connect();
    if (err) {
        LOG_ERR("Failed to connect to nRF Cloud: %d", err);
        return err;
    }
    LOG_INF("Connected to nRF Cloud!");

    return 0;
}

/* Function to read sensor data */
static int read_sensors(struct sensor_value *temp, struct sensor_value *hum, struct sensor_value *light) {
    if (!device_is_ready(bme280) || !device_is_ready(apds)) {
        LOG_ERR("Sensors not ready!");
        return -ENODEV;
    }

    /* Fetch and read temperature & humidity from BME280 */
    sensor_sample_fetch(bme280);
    sensor_channel_get(bme280, SENSOR_CHAN_AMBIENT_TEMP, temp);
    sensor_channel_get(bme280, SENSOR_CHAN_HUMIDITY, hum);

    /* Fetch and read light level from APDS-9960 */
    sensor_sample_fetch(apds);
    sensor_channel_get(apds, SENSOR_CHAN_LIGHT, light);

    return 0;
}

/* Function to send data to nRF Cloud */
static int send_data_to_cloud(struct sensor_value *temp, struct sensor_value *hum, struct sensor_value *light) {
    char json_payload[128];

    /* Create JSON payload */
    snprintf(json_payload, sizeof(json_payload),
        "{\"temperature\":%d.%06d, \"humidity\":%d.%06d, \"light\":%d.%06d}",
        temp->val1, temp->val2, hum->val1, hum->val2, light->val1, light->val2);

    /* Send JSON to nRF Cloud */
    struct nrf_cloud_tx_data msg = {
        .data.ptr = json_payload,
        .data.len = strlen(json_payload),
        .qos = MQTT_QOS_1_AT_LEAST_ONCE,
        .topic_type = NRF_CLOUD_TOPIC_MESSAGE
    };

    int err = nrf_cloud_send(&msg);
    if (err) {
        LOG_ERR("Failed to send data to nRF Cloud: %d", err);
        return err;
    }

    LOG_INF("Sent data to nRF Cloud: %s", json_payload);
    return 0;
}

int main(void) {
    struct sensor_value temp, hum, light;
    

    LOG_INF("Starting nRF9161 Sensor Cloud Application");

    if (!device_is_ready(bme280)) {
        printk("BME280 device is not ready\n");
    }

    if (!device_is_ready(apds)) {
        printk("APDS9960 device is not ready\n");

    }

    if (cloud_connect() != 0) {
        LOG_ERR("Cloud connection failed!");
    }

    while (1) {
        /* Read sensor values */
        if (read_sensors(&temp, &hum, &light) == 0) {
            LOG_INF("Temp: %d.%06d°C, Humidity: %d.%06d%%, Light: %d.%06d lux",
                    temp.val1, temp.val2, hum.val1, hum.val2, light.val1, light.val2);

            /* Send data to nRF Cloud */
            send_data_to_cloud(&temp, &hum, &light);
        }

        /* Send data every 60 seconds */
        k_sleep(K_SECONDS(60));
    }
    return 0; 
}

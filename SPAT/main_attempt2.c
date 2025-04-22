
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <modem/lte_lc.h>
#include <zephyr/net/socket.h>
#include <net/nrf_cloud.h>
#include <net/nrf_cloud_agnss.h>
#include <net/nrf_cloud_alert.h>
#include <modem/nrf_modem_lib.h>
#include <modem/modem_info.h>
#include <nrf_modem_gnss.h>
#include <dk_buttons_and_leds.h>
#include <cJSON.h>

LOG_MODULE_REGISTER(sensor_gnss_cloud, CONFIG_LOG_DEFAULT_LEVEL);
K_SEM_DEFINE(lte_connected, 0, 1);
K_SEM_DEFINE(gnss_done, 0, 1);

/* Sensor devices */
const struct device *bme280 = DEVICE_DT_GET_ANY(bosch_bme280);
const struct device *apds = DEVICE_DT_GET_ANY(avago_apds9960);

/* ADC configuration */
#define ADC_NODE DT_NODELABEL(adc)
#define ADC_RESOLUTION 12
#define NUM_CHANNELS   2
#define BUFFER_SIZE    NUM_CHANNELS
#define VDD_VOLTAGE    3.3f
#define ADC_MAX        4095
#define ADC_GAIN       (1.0f / 6.0f)
#define ADC_REFERENCE  (VDD_VOLTAGE / 4.0f)
#define ADC_FULL_SCALE (ADC_REFERENCE / ADC_GAIN) // 4.95V

static const uint8_t adc_channels[NUM_CHANNELS] = {1, 2}; // AIN1, AIN2
static __aligned(4) int16_t adc_sample_buffer[BUFFER_SIZE];
static struct adc_channel_cfg adc_cfg[NUM_CHANNELS];
static struct adc_sequence adc_sequence = {
    .channels    = 0,
    .buffer      = adc_sample_buffer,
    .buffer_size = sizeof(adc_sample_buffer),
    .resolution  = ADC_RESOLUTION,
};
static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

/* GNSS data */
static struct nrf_modem_gnss_pvt_data_frame pvt_data;
static struct nrf_modem_gnss_agnss_data_frame agnss_request;
static int64_t gnss_start_time;
static bool gnss_active = false;
static bool has_fix = false;
static int num_satellites = 0;
#define SLEEP_DURATION_MS (60 * 1000) // 1 minute
#define GNSS_TIMEOUT_MS (60 * 1000)   // 1 minute

/* Alert thresholds */
#define TEMP_THRESHOLD   (double)  25.0  // °C
#define HUMID_THRESHOLD (double)  20.0f  // %
#define LIGHT_THRESHOLD (double)  1000.0f // lux
#define ADC_THRESHOLD   (double)  1.5f   // Volts


/* nRF Cloud device ID */
static char device_id[NRF_CLOUD_CLIENT_ID_MAX_LEN + 1];

static float convert_to_voltage(int16_t raw)
{
    return ((float)raw / ADC_MAX) * ADC_FULL_SCALE; // 4.95V full scale
}

static int setup_adc(void)
{
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    for (int i = 0; i < NUM_CHANNELS; i++) {
        adc_cfg[i] = (struct adc_channel_cfg){
            .gain             = ADC_GAIN_1_6,
            .reference        = ADC_REF_VDD_1_4,
            .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10),
            .channel_id       = adc_channels[i],
        #ifdef CONFIG_ADC_CONFIGURABLE_INPUTS
            .input_positive   = SAADC_CH_PSELP_PSELP_AnalogInput0 + adc_channels[i],
        #endif
        };

        int err = adc_channel_setup(adc_dev, &adc_cfg[i]);
        if (err) {
            LOG_ERR("ADC channel %d setup failed: %d", adc_channels[i], err);
            return err;
        }
        LOG_INF("ADC channel %d setup OK", adc_channels[i]);
        adc_sequence.channels |= BIT(adc_channels[i]);
    }
    return 0;
}

static int read_adc(float *voltages)
{
    int err = adc_read(adc_dev, &adc_sequence);
    if (err) {
        LOG_ERR("ADC read failed: %d", err);
        return err;
    }

    for (int i = 0; i < NUM_CHANNELS; i++) {
        voltages[i] = convert_to_voltage(adc_sample_buffer[i]);
        LOG_INF("AIN%d: raw = %d → voltage = %.3f V", adc_channels[i], adc_sample_buffer[i], (double)voltages[i]);
    }
    return 0;
}

static int send_adc_to_cloud(float *voltages)
{
    int ret;
    cJSON *root;
    char *json_str;

    for (int i = 0; i < NUM_CHANNELS; i++) {
        root = cJSON_CreateObject();
        if (!root) {
            LOG_ERR("Failed to create JSON object for ADC%d", i);
            return -ENOMEM;
        }
        char app_id[8];
        snprintf(app_id, sizeof(app_id), "ADC%d", i);
        cJSON_AddStringToObject(root, "appId", app_id);
        cJSON_AddNumberToObject(root, "data", voltages[i]);
        cJSON_AddStringToObject(root, "messageType", "DATA");

        json_str = cJSON_PrintUnformatted(root);
        if (!json_str) {
            LOG_ERR("Failed to create JSON string for ADC%d", i);
            cJSON_Delete(root);
            return -ENOMEM;
        }
        struct nrf_cloud_tx_data adc_msg = {
            .data.ptr = json_str,
            .data.len = strlen(json_str),
            .qos = MQTT_QOS_1_AT_LEAST_ONCE,
            .topic_type = NRF_CLOUD_TOPIC_MESSAGE
        };
        LOG_INF("Sending ADC%d: %s", i, json_str);
        ret = nrf_cloud_send(&adc_msg);
        cJSON_free(json_str);
        cJSON_Delete(root);
        if (ret) {
            LOG_ERR("Failed to send ADC%d data: %d", i, ret);
            return ret;
        }
    }
    return 0;
    k_sleep(K_MSEC(2000));
}

// https://docs.nordicsemi.com/bundle/nrf-apis-latest/page/group_nrf_cloud_alert.html
// https://github.com/nrfconnect/sdk-nrf/blob/main/doc/nrf/libraries/networking/nrf_cloud_alert.rst#requirements
// https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/libraries/networking/nrf_cloud_alert.html#samples_using_the_library


static void check_and_send_alerts(struct sensor_value *temp, struct sensor_value *hum, struct sensor_value *light, float *adc_voltages)
{
    float temp_val = (float)sensor_value_to_double(temp);
    float hum_val = (float)sensor_value_to_double(hum);
    float light_val = (float)sensor_value_to_double(light);
    char desc[64];
    int ret;

    if ((double)temp_val > TEMP_THRESHOLD) {
        snprintf(desc, sizeof(desc), "Temperature exceeds %.1f°C", TEMP_THRESHOLD);
        ret = nrf_cloud_alert_send(ALERT_TYPE_TEMPERATURE, temp_val, desc);
        if (ret) {
            LOG_ERR("Failed to send temperature alert: %d", ret);
        } else {
            LOG_INF("Sent temperature alert: %s (value: %.2f)", desc, (double)temp_val);
        }
    }
    k_sleep(K_MSEC(1000));

    if ((double)hum_val > HUMID_THRESHOLD) {
        snprintf(desc, sizeof(desc), "Humidity exceeds %.1f%%", HUMID_THRESHOLD);
        ret = nrf_cloud_alert_send(ALERT_TYPE_HUMIDITY, hum_val, desc);
        if (ret) {
            LOG_ERR("Failed to send humidity alert: %d", ret);
        } else {
            LOG_INF("Sent humidity alert: %s (value: %.2f)", desc, (double)hum_val);
        }
    }
    k_sleep(K_MSEC(1000));

    if ((double)light_val > LIGHT_THRESHOLD) {
        snprintf(desc, sizeof(desc), "Light exceeds %.1f lux", LIGHT_THRESHOLD);
        ret = nrf_cloud_alert_send(ALERT_TYPE_CUSTOM, light_val, desc);
        if (ret) {
            LOG_ERR("Failed to send light alert: %d", ret);
        } else {
            LOG_INF("Sent light alert: %s (value: %.2f)", desc, (double)light_val);
        }
    }
    k_sleep(K_MSEC(1000));

    for (int i = 0; i < NUM_CHANNELS; i++) {
        if ((double)adc_voltages[i] > ADC_THRESHOLD) {
            snprintf(desc, sizeof(desc), "ADC%d voltage exceeds %.1fV", i, ADC_THRESHOLD);
            enum nrf_cloud_alert_type alert_type = (i == 0) ? ALERT_TYPE_CUSTOM : ALERT_TYPE_CUSTOM;
            ret = nrf_cloud_alert_send(alert_type, adc_voltages[i], desc);
            if (ret) {
                LOG_ERR("Failed to send ADC%d alert: %d", i, ret);
            } else {
                LOG_INF("Sent ADC%d alert: %s (value: %.2f)", i, desc, (double)adc_voltages[i]);
            }
        }
    }
    k_sleep(K_MSEC(2000));
}

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
    LOG_INF("Latitude:  %.6f", pvt_data->latitude);
    LOG_INF("Longitude: %.6f", pvt_data->longitude);
    LOG_INF("Altitude:  %.1f m", (double)pvt_data->altitude);
    LOG_INF("Time (UTC): %02d:%02d:%02d.%03d",
            pvt_data->datetime.hour, pvt_data->datetime.minute,
            pvt_data->datetime.seconds, pvt_data->datetime.ms);
}

static void enter_sleep_mode(void)
{
    LOG_INF("Entering sleep mode");
    dk_set_led_off(DK_LED1);
    LOG_INF("Sleeping for %d seconds", SLEEP_DURATION_MS / 1000);
    k_msleep(SLEEP_DURATION_MS);
}

static int read_sensors(struct sensor_value *temp, struct sensor_value *hum, struct sensor_value *light)
{
    int err, retries = 3;
    while (retries--) {
        err = sensor_sample_fetch(bme280);
        if (!err) break;
        LOG_ERR("BME280 fetch failed: %d, retries left: %d", err, retries);
        k_sleep(K_MSEC(100));
    }
    if (err) return err;

    err = sensor_channel_get(bme280, SENSOR_CHAN_AMBIENT_TEMP, temp);
    if (err) {
        LOG_ERR("BME280 temp get failed: %d", err);
        return err;
    }
    err = sensor_channel_get(bme280, SENSOR_CHAN_HUMIDITY, hum);
    if (err) {
        LOG_ERR("BME280 humidity get failed: %d", err);
        return err;
    }

    err = sensor_sample_fetch(apds);
    if (err) {
        LOG_ERR("APDS9960 fetch failed: %d", err);
        return err;
    }
    err = sensor_channel_get(apds, SENSOR_CHAN_LIGHT, light);
    if (err) {
        LOG_ERR("APDS9960 light get failed: %d", err);
        return err;
    }
    return 0;
}

static int send_data_to_cloud(struct sensor_value *temp, struct sensor_value *hum, struct sensor_value *light)
{
    int ret;
    cJSON *root;
    char *json_str;

    // Temperature
    root = cJSON_CreateObject();
    if (!root) {
        LOG_ERR("Failed to create JSON object");
        return -ENOMEM;
    }
    cJSON_AddStringToObject(root, "appId", "TEMP");
    cJSON_AddNumberToObject(root, "data", sensor_value_to_double(temp));
    cJSON_AddStringToObject(root, "messageType", "DATA");
    json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        LOG_ERR("Failed to create JSON string");
        cJSON_Delete(root);
        return -ENOMEM;
    }
    struct nrf_cloud_tx_data temp_msg = {
        .data.ptr = json_str,
        .data.len = strlen(json_str),
        .qos = MQTT_QOS_1_AT_LEAST_ONCE,
        .topic_type = NRF_CLOUD_TOPIC_MESSAGE
    };
    LOG_INF("Sending temp: %s", json_str);
    ret = nrf_cloud_send(&temp_msg);
    cJSON_free(json_str);
    cJSON_Delete(root);
    if (ret) {
        LOG_ERR("Failed to send temperature data: %d", ret);
        return ret;
    }

    // Humidity
    root = cJSON_CreateObject();
    if (!root) {
        LOG_ERR("Failed to create JSON object");
        return -ENOMEM;
    }
    cJSON_AddStringToObject(root, "appId", "HUMID");
    cJSON_AddNumberToObject(root, "data", sensor_value_to_double(hum));
    cJSON_AddStringToObject(root, "messageType", "DATA");
    json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        LOG_ERR("Failed to create JSON string");
        cJSON_Delete(root);
        return -ENOMEM;
    }
    struct nrf_cloud_tx_data hum_msg = {
        .data.ptr = json_str,
        .data.len = strlen(json_str),
        .qos = MQTT_QOS_1_AT_LEAST_ONCE,
        .topic_type = NRF_CLOUD_TOPIC_MESSAGE
    };
    LOG_INF("Sending humid: %s", json_str);
    ret = nrf_cloud_send(&hum_msg);
    cJSON_free(json_str);
    cJSON_Delete(root);
    if (ret) {
        LOG_ERR("Failed to send humidity data: %d", ret);
        return ret;
    }

    // Light
    root = cJSON_CreateObject();
    if (!root) {
        LOG_ERR("Failed to create JSON object");
        return -ENOMEM;
    }
    cJSON_AddStringToObject(root, "appId", "LIGHT");
    cJSON_AddNumberToObject(root, "data", sensor_value_to_double(light));
    cJSON_AddStringToObject(root, "messageType", "DATA");
    json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        LOG_ERR("Failed to create JSON string");
        cJSON_Delete(root);
        return -ENOMEM;
    }
    struct nrf_cloud_tx_data light_msg = {
        .data.ptr = json_str,
        .data.len = strlen(json_str),
        .qos = MQTT_QOS_1_AT_LEAST_ONCE,
        .topic_type = NRF_CLOUD_TOPIC_MESSAGE
    };
    LOG_INF("Sending light: %s", json_str);
    ret = nrf_cloud_send(&light_msg);
    cJSON_free(json_str);
    cJSON_Delete(root);
    if (ret) {
        LOG_ERR("Failed to send light data: %d", ret);
        return ret;
    }
    k_sleep(K_MSEC(1000));
    return 0;
}

static int send_gnss_to_cloud(void)
{
    int ret;
    cJSON *root;
    char *json_str;

    root = cJSON_CreateObject();
    if (!root) {
        LOG_ERR("Failed to create JSON object");
        return -ENOMEM;
    }
    cJSON_AddStringToObject(root, "appId", "GNSS");
    cJSON *data = cJSON_CreateObject();
    cJSON_AddBoolToObject(data, "fix_valid", has_fix);
    cJSON_AddNumberToObject(data, "num_satellites", num_satellites);
    if (has_fix) {
        cJSON_AddNumberToObject(data, "latitude", pvt_data.latitude);
        cJSON_AddNumberToObject(data, "longitude", pvt_data.longitude);
        cJSON_AddNumberToObject(data, "altitude", pvt_data.altitude);
        cJSON *time = cJSON_CreateObject();
        cJSON_AddNumberToObject(time, "hour", pvt_data.datetime.hour);
        cJSON_AddNumberToObject(time, "minute", pvt_data.datetime.minute);
        cJSON_AddNumberToObject(time, "seconds", pvt_data.datetime.seconds);
        cJSON_AddNumberToObject(time, "ms", pvt_data.datetime.ms);
        cJSON_AddItemToObject(root, "time", time);
    }
    cJSON_AddItemToObject(root, "data", data);
    cJSON_AddStringToObject(root, "messageType", "DATA");

    json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        LOG_ERR("Failed to create JSON string");
        cJSON_Delete(root);
        return -ENOMEM;
    }
    struct nrf_cloud_tx_data gnss_msg = {
        .data.ptr = json_str,
        .data.len = strlen(json_str),
        .qos = MQTT_QOS_1_AT_LEAST_ONCE,
        .topic_type = NRF_CLOUD_TOPIC_MESSAGE
    };
    LOG_INF("Sending GNSS: %s", json_str);
    ret = nrf_cloud_send(&gnss_msg);
    cJSON_free(json_str);
    cJSON_Delete(root);
    if (ret) {
        LOG_ERR("Failed to send GNSS data: %d", ret);
    }

    return ret;
}

static void gnss_event_handler(int event)
{
    int err;

    if (gnss_active && (k_uptime_get() - gnss_start_time) >= GNSS_TIMEOUT_MS) {
        LOG_INF("GNSS fix timeout after %d seconds", GNSS_TIMEOUT_MS / 1000);
        has_fix = false;
        k_sem_give(&gnss_done);
        return;
    }

    switch (event) {
    case NRF_MODEM_GNSS_EVT_PVT:
        LOG_INF("Searching...");
        err = nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT);
        if (err) {
            LOG_ERR("nrf_modem_gnss_read failed, err %d", err);
            return;
        }
        num_satellites = 0;
        for (int i = 0; i < 12; i++) {
            if (pvt_data.sv[i].signal != 0) {
                LOG_INF("sv: %d, cn0: %d, signal: %d, elev: %d, azim: %d",
                        pvt_data.sv[i].sv, pvt_data.sv[i].cn0, pvt_data.sv[i].signal,
                        pvt_data.sv[i].elevation, pvt_data.sv[i].azimuth);
                num_satellites++;
            }
        }
        LOG_INF("Number of current satellites: %d", num_satellites);
        LOG_INF("PVT flags: 0x%08x", pvt_data.flags);
        if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
            dk_set_led_on(DK_LED1);
            print_fix_data(&pvt_data);
            has_fix = true;
            k_sem_give(&gnss_done);
        } else {
            LOG_INF("No valid fix");
        }
        break;

    case NRF_MODEM_GNSS_EVT_AGNSS_REQ:
        LOG_INF("A-GNSS data needed");
        err = nrf_modem_gnss_read(&agnss_request, sizeof(agnss_request), NRF_MODEM_GNSS_DATA_AGNSS_REQ);
        if (err) {
            LOG_ERR("Failed to read A-GNSS request, err %d", err);
            return;
        }
        err = nrf_cloud_agnss_request(&agnss_request);
        if (err) {
            LOG_ERR("Failed to request A-GNSS data, err %d", err);
            return;
        }
        LOG_INF("A-GNSS request sent to nRF Cloud");
        break;

    case NRF_MODEM_GNSS_EVT_FIX:
        LOG_INF("GNSS fix event received");
        break;

    default:
        LOG_INF("Unhandled GNSS event: %d", event);
        break;
    }
    k_sleep(K_MSEC(1000));
}

static void lte_handler(const struct lte_lc_evt *const evt)
{
    switch (evt->type) {
    case LTE_LC_EVT_NW_REG_STATUS:
        LOG_INF("Network registration status: %d", evt->nw_reg_status);
        if (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ||
            evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING) {
            LOG_INF("Connected to %s network",
                    evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "home" : "roaming");
            k_sem_give(&lte_connected);
        } else if (evt->nw_reg_status == LTE_LC_NW_REG_NOT_REGISTERED ||
                   evt->nw_reg_status == LTE_LC_NW_REG_SEARCHING) {
            LOG_INF("Network not registered or searching...");
        } else {
            LOG_ERR("Unexpected registration status: %d", evt->nw_reg_status);
        }
        break;
    case LTE_LC_EVT_CELL_UPDATE:
        LOG_INF("Cell update: Cell ID %d", evt->cell.id);
        break;
    case LTE_LC_EVT_LTE_MODE_UPDATE:
        LOG_INF("LTE mode update: %d", evt->lte_mode);
        break;
    case LTE_LC_EVT_MODEM_EVENT:
        LOG_INF("Modem event: %d", evt->modem_evt);
        break;
    default:
        LOG_INF("Unhandled LTE event: %d", evt->type);
        break;
    }
}

static void cloud_event_handler(const struct nrf_cloud_evt *evt)
{
    if (evt == NULL) {
        LOG_ERR("Received NULL event");
        return;
    }
    switch (evt->type) {
    case NRF_CLOUD_EVT_TRANSPORT_CONNECTED:
        LOG_INF("Transport connected");
        break;
    case NRF_CLOUD_EVT_TRANSPORT_DISCONNECTED:
        LOG_INF("Transport disconnected");
        break;
    case NRF_CLOUD_EVT_READY:
        LOG_INF("Cloud connection ready");
        break;
    case NRF_CLOUD_EVT_ERROR:
        LOG_ERR("Cloud error occurred (type: %d)", evt->type);
        break;
    case NRF_CLOUD_EVT_RX_DATA_SHADOW:
        LOG_INF("Received shadow data");
        if (evt->data.ptr && evt->data.len > 0) {
            LOG_INF("Shadow content: %s", (const char *)evt->data.ptr);
        } else {
            LOG_INF("Shadow content: <empty>");
        }
        break;
    default:
        LOG_INF("Unhandled cloud event: %d", evt->type);
        break;
    }
}

static int init(void)
{
    int err;

    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Failed to initialize modem library: 0x%X", err);
        return -EFAULT;
    }

    err = modem_info_init();
    if (err) {
        LOG_ERR("Modem info initialization failed: %d", err);
        return err;
    }

    err = dk_leds_init();
    if (err) {
        LOG_ERR("Failed to initialize LEDs, err %d", err);
        return err;
    }


    LOG_INF("Modem, peripherals, and alerts initialized");
    return 0;
}

static int connect_to_lte(void)
{
    int err;
    LOG_INF("Waiting for network...");

    k_sem_reset(&lte_connected);
    err = lte_lc_connect_async(lte_handler);
    if (err) {
        LOG_ERR("Failed to init modem, error: %d", err);
        return err;
    }

    err = lte_lc_psm_req(true);
    if (err) {
        LOG_ERR("Failed to request PSM, err %d", err);
    }

    err = lte_lc_edrx_req(true);
    if (err) {
        LOG_ERR("Failed to request eDRX, err %d", err);
    }

    k_sem_take(&lte_connected, K_FOREVER);
    LOG_INF("Connected to LTE");
    return 0;
}

static int setup_connection(void)
{
    int err;

    err = connect_to_lte();
    if (err) {
        LOG_ERR("Failed to connect to cellular network: %d", err);
        return err;
    }

    memset(device_id, 0, sizeof(device_id));
    err = nrf_cloud_client_id_get(device_id, sizeof(device_id));
    if (err) {
        LOG_ERR("Failed to get device ID, error: %d", err);
        return err;
    }
    LOG_INF("Device ID: %s (length: %d)", device_id, strlen(device_id));

    LOG_INF("Attempting initial nRF Cloud connection...");
    struct nrf_cloud_init_param params = {
        .client_id = device_id,
        .event_handler = cloud_event_handler,
    };

    err = nrf_cloud_init(&params);
    if (err) {
        LOG_ERR("Failed to initialize nRF Cloud library: %d", err);
        return err;
    }

    err = nrf_cloud_connect();
    if (err) {
        LOG_ERR("Initial nRF Cloud connection failed: %d", err);
        return err;
    }
    LOG_INF("Connected to nRF Cloud");
    return 0;
}

static int setup_gnss(void)
{
    int err;

    err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
    if (err) {
        LOG_ERR("Failed to activate GNSS, err %d", err);
        return err;
    }

    err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
    if (err) {
        LOG_ERR("Failed to set GNSS event handler, err %d", err);
        return err;
    }

    err = nrf_modem_gnss_fix_interval_set(1);
    if (err) {
        LOG_ERR("Failed to set GNSS fix interval, err %d", err);
        return err;
    }

    err = nrf_modem_gnss_fix_retry_set(10);
    if (err) {
        LOG_ERR("Failed to set GNSS fix retry, err %d", err);
        return err;
    }

    err = nrf_modem_gnss_start();
    if (err) {
        LOG_ERR("Failed to start GNSS, err %d", err);
        return err;
    }

    gnss_start_time = k_uptime_get();
    gnss_active = true;
    LOG_INF("GNSS started");
    return 0;
}

static int attempt_gnss_fix(void)
{
    int err;

    k_sem_reset(&gnss_done);
    err = setup_gnss();
    if (err) {
        LOG_ERR("Failed to set up GNSS, err %d", err);
        return err;
    }

    k_sem_take(&gnss_done, K_MSEC(GNSS_TIMEOUT_MS));
    LOG_INF("GNSS attempt completed (fix: %s)", has_fix ? "yes" : "no");

    if (gnss_active) {
        err = nrf_modem_gnss_stop();
        if (err) {
            LOG_ERR("Failed to stop GNSS, err %d", err);
        } else {
            gnss_active = false;
        }
    }

    return 0;
}

static int setup(void)
{
    int err, retries = 3;

    err = init();
    if (err) {
        LOG_ERR("Initialization failed.");
        return err;
    }

    err = setup_connection();
    if (err) {
        LOG_ERR("Connection set-up failed.");
        return err;
    }

    err = setup_adc();
    if (err) {
        LOG_ERR("ADC setup failed.");
        return err;
    }

    while (retries--) {
        if (device_is_ready(bme280) && device_is_ready(apds)) {
            break;
        }
        LOG_ERR("Sensors not ready, retries left: %d", retries);
        k_sleep(K_MSEC(1000));
    }
    if (!device_is_ready(bme280)) {
        LOG_ERR("BME280 device is not ready, stopping");
        return -ENODEV;
    }
    if (!device_is_ready(apds)) {
        LOG_ERR("APDS9960 device is not ready, stopping");
        return -ENODEV;
    }
    return 0;
}

static bool cred_check(struct nrf_cloud_credentials_status *const cs)
{
    int ret;

    ret = nrf_cloud_credentials_check(cs);
    if (ret) {
        LOG_ERR("nRF Cloud credentials check failed, error: %d", ret);
        return false;
    }

    LOG_INF("Checking credentials in sec tag %u", cs->sec_tag);
    if (!cs->ca) {
        LOG_WRN("CA certificate missing");
    }
    if (!cs->prv_key) {
        LOG_WRN("Private key missing");
    }
    if (!cs->client_cert) {
        LOG_WRN("Client certificate missing");
    }

    return (cs->ca && cs->prv_key && cs->client_cert);
}

int main(void)
{
    int err;
    struct nrf_cloud_credentials_status cs = {0};
    struct sensor_value temp, hum, light;
    struct modem_param_info modem_info;
    float adc_voltages[NUM_CHANNELS];

    LOG_INF("Starting nRF9161 Sensor, GNSS, and ADC Cloud Application");

    err = setup();
    if (err) {
        LOG_ERR("Setup failed, stopping.");
        return err;
    }

    if (!cred_check(&cs)) {
        LOG_ERR("Credentials check failed, stopping.");
        return -EACCES;
    }

    while (1) {
        // Attempt GNSS fix
        err = attempt_gnss_fix();
        if (err) {
            LOG_ERR("GNSS attempt failed, continuing: %d", err);
        }

        // Get modem info
        err = modem_info_params_get(&modem_info);
        if (!err) {
            LOG_INF("Signal strength: %d dBm", modem_info.network.rsrp.value);
        } else {
            LOG_WRN("Failed to get modem info: %d (continuing)", err);
            k_sleep(K_MSEC(1000));
        }
        
        // Send GNSS data
        err = send_gnss_to_cloud();
        if (err) {
            LOG_ERR("GNSS data send failed, attempting reconnect");
            nrf_cloud_disconnect();
            err = setup_connection();
            if (err) {
                LOG_ERR("Reconnection failed, continuing: %d", err);
            }
        }
            
        // Read and send sensor data
        if (read_sensors(&temp, &hum, &light) == 0) {
            LOG_INF("Temp: %d.%06d°C, Humidity: %d.%06d%%, Light: %d.%06d lux",
                    temp.val1, temp.val2, hum.val1, hum.val2, light.val1, light.val2);

            err = send_data_to_cloud(&temp, &hum, &light);
            if (err) {
                LOG_ERR("Sensor data send failed, attempting reconnect");
                nrf_cloud_disconnect();
                err = setup_connection();
                if (err) {
                    LOG_ERR("Reconnection failed, continuing: %d", err);
                }
            }
        } else {
            LOG_ERR("Sensor read failed");
        }

        // Read and send ADC data
        if (read_adc(adc_voltages) == 0) {
            err = send_adc_to_cloud(adc_voltages);
            if (err) {
                LOG_ERR("ADC data send failed, attempting reconnect");
                nrf_cloud_disconnect();
                err = setup_connection();
                if (err) {
                    LOG_ERR("Reconnection failed, continuing: %d", err);
                }
            }
        } else {
            LOG_ERR("ADC read failed");
        }

        // Check and send alerts
        check_and_send_alerts(&temp, &hum, &light, adc_voltages);


        // Enter sleep mode
        enter_sleep_mode();
    }
    return 0;
}

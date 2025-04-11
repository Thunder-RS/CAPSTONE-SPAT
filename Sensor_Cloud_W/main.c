#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <modem/lte_lc.h>
#include <zephyr/net/socket.h>
#include <net/nrf_cloud.h>
#include <modem/nrf_modem_lib.h>
#include <modem/modem_info.h>
#include <cJSON.h>

static char device_id[NRF_CLOUD_CLIENT_ID_MAX_LEN + 1];

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);
K_SEM_DEFINE(lte_connected, 0, 1);

/* Sensor devices */
const struct device *bme280 = DEVICE_DT_GET_ANY(bosch_bme280);
const struct device *apds = DEVICE_DT_GET_ANY(avago_apds9960);

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

    k_sem_take(&lte_connected, K_FOREVER);
    LOG_INF("Connected to LTE");
    return 0;
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
    LOG_INF("Modem library initialized");
    return 0;
}

void cloud_event_handler(const struct nrf_cloud_evt *evt)
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

    err = nrf_cloud_init(&params);  // Fixed typo here
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

static int setup(void)
{
    int err;
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

    if (!device_is_ready(bme280)) {
        LOG_ERR("BME280 device is not ready");
        return -ENODEV;
    }
    if (!device_is_ready(apds)) {
        LOG_ERR("APDS9960 device is not ready");
        return -ENODEV;
    }
    return 0;
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
    cJSON_AddNumberToObject(root, "data", sensor_value_to_double(temp));  // Must be a number
    cJSON_AddStringToObject(root, "messageType", "DATA");
    
    json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        LOG_ERR("Failed to create JSON string");
        cJSON_Delete(root);
        return -ENOMEM;
    }
    struct nrf_cloud_tx_data temp_msg = {
        .data.ptr = json_str, .data.len = strlen(json_str),
        .qos = MQTT_QOS_1_AT_LEAST_ONCE, .topic_type = NRF_CLOUD_TOPIC_MESSAGE
    };
    LOG_INF("Sending temp: %s (value: %f)", json_str, sensor_value_to_double(temp));
    ret = nrf_cloud_send(&temp_msg);
    cJSON_free(json_str);
    cJSON_Delete(root);
    if (ret) return ret;

    // Humidity
    root = cJSON_CreateObject();
    if (!root) {
        LOG_ERR("Failed to create JSON object");
        return -ENOMEM;
    }
    cJSON_AddStringToObject(root, "appId", "HUMID");
    cJSON_AddNumberToObject(root, "data", sensor_value_to_double(hum));  // Must be a number
    cJSON_AddStringToObject(root, "messageType", "DATA");
    json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        LOG_ERR("Failed to create JSON string");
        cJSON_Delete(root);
        return -ENOMEM;
    }
    struct nrf_cloud_tx_data hum_msg = {
        .data.ptr = json_str, .data.len = strlen(json_str),
        .qos = MQTT_QOS_1_AT_LEAST_ONCE, .topic_type = NRF_CLOUD_TOPIC_MESSAGE
    };
    LOG_INF("Sending humid: %s", json_str);  // Log directly
    ret = nrf_cloud_send(&hum_msg);
    cJSON_free(json_str);
    cJSON_Delete(root);
    if (ret) return ret;

    // Light
    root = cJSON_CreateObject();
    if (!root) {
        LOG_ERR("Failed to create JSON object");
        return -ENOMEM;
    }
    cJSON_AddStringToObject(root, "appId", "LIGHT");
    cJSON_AddNumberToObject(root, "data", sensor_value_to_double(light));  // Must be a number
    cJSON_AddStringToObject(root, "messageType", "DATA");
    json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        LOG_ERR("Failed to create JSON string");
        cJSON_Delete(root);
        return -ENOMEM;
    }
    struct nrf_cloud_tx_data light_msg = {
        .data.ptr = json_str, .data.len = strlen(json_str),
        .qos = MQTT_QOS_1_AT_LEAST_ONCE, .topic_type = NRF_CLOUD_TOPIC_MESSAGE
    };
    LOG_INF("Sending light: %s", json_str);  // Log directly
    ret = nrf_cloud_send(&light_msg);
    cJSON_free(json_str);
    cJSON_Delete(root);

    return ret;
}

static bool cred_check(struct nrf_cloud_credentials_status *const cs)
{
        int ret;

        ret = nrf_cloud_credentials_check(cs);
if (ret) {
        LOG_ERR("nRF Cloud credentials check failed, error: %d", ret);
        return false;
}
/* Since this is a REST sample, we only need two credentials:
	 *  - a CA for the TLS connections
	 *  - a private key to sign the JWT
	 */

         LOG_INF("Checking credentials in sec tag %u", cs->sec_tag);
         if (!cs->ca) {
             LOG_WRN("CA certificate missing");
         }
         if (!cs->ca_aws) {
             LOG_WRN("AWS CA certificate missing");
         }
         if (!cs->prv_key) {
             LOG_WRN("Private key missing");
         }
      
     
         // For MQTT, you typically need CA and private key at minimum
         return (cs->ca && cs->prv_key);
   
}

int main(void)
{
    int err;
    struct nrf_cloud_credentials_status cs = {0};
    struct sensor_value temp, hum, light;
    struct modem_param_info modem_info;

    LOG_INF("Starting nRF9161 Sensor Cloud Application");

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
        err = modem_info_params_get(&modem_info);
        if (!err) {
            LOG_INF("Signal strength: %d dBm", modem_info.network.rsrp.value);
        } else {
            LOG_WRN("Failed to get modem info: %d (continuing)", err);
        }

        if (read_sensors(&temp, &hum, &light) == 0) {
            LOG_INF("Temp: %d.%06d°C, Humidity: %d.%06d%%, Light: %d.%06d lux",
                    temp.val1, temp.val2, hum.val1, hum.val2, light.val1, light.val2);

            err = send_data_to_cloud(&temp, &hum, &light);
            if (err) {
                LOG_ERR("Data send failed, will retry");
            }
        } else {
            LOG_ERR("Sensor read failed");
        }

        k_sleep(K_SECONDS(10));
    }
    return 0;
}

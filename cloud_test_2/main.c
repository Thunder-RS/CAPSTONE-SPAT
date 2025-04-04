#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <modem/lte_lc.h>
#include <zephyr/net/socket.h>
#include <net/nrf_cloud.h>
#include <modem/nrf_modem_lib.h>
#include <modem/modem_info.h>

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
    struct modem_param_info modem_info;
	LOG_INF("Waiting for network...");

	k_sem_reset(&lte_connected);

	err = lte_lc_connect_async(lte_handler);
	if (err) {
		LOG_ERR("Failed to init modem, error: %d", err);
		return err;
	}
    err = modem_info_params_get(&modem_info);
    if (!err) {
        LOG_INF("Initial signal strength: %d dBm", modem_info.network.rsrp.value);
    } else {
        LOG_ERR("Failed to get initial modem info: %d", err);
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
        switch (err) {
            case -EINVAL:
                LOG_ERR("Invalid argument in modem info init");
                break;
            case -EAGAIN:
                LOG_ERR("Modem not ready yet");
                break;
            default:
                LOG_ERR("Unknown modem info init error");
                break;
        }
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
            // For versions without param.err, just log the event
            LOG_ERR("Cloud error occurred (type: %d)", evt->type);
            break;
        case NRF_CLOUD_EVT_RX_DATA_SHADOW:
            LOG_INF("Received shadow data");
            // Add shadow validation if needed
            break;
        default:
            LOG_INF("Unhandled cloud event: %d", evt->type);
            break;
    }
}

static int setup_connection(void)
{
	int err;

        /* Connect to LTE */

        err = connect_to_lte();
	if (err) {
		LOG_ERR("Failed to connect to cellular network: %d", err);
		return err;
	}

	/* Get the device ID */
        memset(device_id, 0, sizeof(device_id));
	err = nrf_cloud_client_id_get(device_id, sizeof(device_id));
	if (err) {
		LOG_ERR("Failed to get device ID, error: %d", err);
		return err;
	}
        size_t id_len = strlen(device_id);
         if (id_len == 0 || id_len >= NRF_CLOUD_CLIENT_ID_MAX_LEN) {
        LOG_ERR("Invalid client ID length: %d (max: %d)", id_len, NRF_CLOUD_CLIENT_ID_MAX_LEN);
        return -EINVAL;
        }
        LOG_INF("Device ID: %s (length: %d)", device_id, strlen(device_id));

	
        if (strlen(device_id) > NRF_CLOUD_CLIENT_ID_MAX_LEN) {
            LOG_ERR("Device ID too long: %d > %d", strlen(device_id), NRF_CLOUD_CLIENT_ID_MAX_LEN);
            return -EINVAL;
        }

        LOG_INF("Attempting initial nRF Cloud connection...");



        struct nrf_cloud_init_param params = {
                .client_id = device_id,  // Use the device ID retrieved earlier
                .event_handler = cloud_event_handler,   // Add a handler if you need to process cloud events
           
            };


        

            err = nrf_cloud_init(&params);
            if (err) {
                LOG_ERR("Failed to initialize nRF Cloud library: %d", err);
                switch (err) {
                        case -EINVAL:
                            LOG_ERR("Invalid argument - check client ID or Kconfig");
                            break;
                        case -EACCES:
                            LOG_ERR("Access denied - check credentials");
                            break;
                        case -EFAULT:
                            LOG_ERR("Internal error - check modem initialization");
                            break;
                        default:
                            LOG_ERR("Unknown error");
                            break;
                    }
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


	/* Initialize libraries and hardware */
	err = init();
	if (err) {
		LOG_ERR("Initialization failed.");
		return err;
	}

	/* Initiate Connection */
	err = setup_connection();
	if (err) {
		LOG_ERR("Connection set-up failed.");
		return err;
	}
     
       
    


	

	return 0;
}

static int read_sensors(struct sensor_value *temp, struct sensor_value *hum, struct sensor_value *light) {
        if (!device_is_ready(bme280) || !device_is_ready(apds)) {
            LOG_ERR("Sensors not ready!");
            return -ENODEV;
        }
        int err;
        err = sensor_sample_fetch(bme280);
        if (err) {
        LOG_ERR("BME280 fetch failed: %d", err);
        return err;
        }
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
        static char json_payload[128];  // Static to reduce stack usage
        int err, retries = 3;
    
        while (retries--) {
            // Check if connected
            err = nrf_cloud_disconnect();
            if (err && err != -ENOTCONN) {  // ENOTCONN means already disconnected
                LOG_ERR("Disconnect check failed: %d", err);
            }
            
            LOG_INF("Attempting to connect...");
            err = nrf_cloud_connect();
            if (err) {
                LOG_ERR("Connect failed: %d, retries left: %d", err, retries);
                k_sleep(K_SECONDS(5));
                continue;
            }
            
            k_sleep(K_SECONDS(2)); // Wait for connection stability
            
            snprintf(json_payload, sizeof(json_payload),
                     "{\"temperature\":%d.%06d,\"humidity\":%d.%06d,\"light\":%d.%06d}",
                     temp->val1, temp->val2, hum->val1, hum->val2, light->val1, light->val2);
    
            struct nrf_cloud_tx_data msg = {
                .data.ptr = json_payload,
                .data.len = strlen(json_payload),
                .qos = MQTT_QOS_1_AT_LEAST_ONCE,
                .topic_type = NRF_CLOUD_TOPIC_MESSAGE
            };
    
            LOG_INF("Sending: %s", json_payload);
            err = nrf_cloud_send(&msg);
            if (err == 0) {
                LOG_INF("Data sent successfully");
                return 0;
            }
            LOG_ERR("Send failed: %d", err);
            k_sleep(K_SECONDS(5));
        }
        LOG_ERR("Failed to send data after retries");
        return err;
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

int main(void) {
        int err;
      
    

        err = setup();
	if (err) {
		LOG_ERR("Setup failed, stopping.");
		return err;
	}
   


        struct nrf_cloud_credentials_status cs;
        struct sensor_value temp, hum, light;
        struct modem_param_info modem_info;
        
        LOG_INF("Starting nRF9161 Sensor Cloud Application");
        
        if (!device_is_ready(bme280)) {
                printk("BME280 device is not ready\n");
            }
        
            if (!device_is_ready(apds)) {
                printk("APDS9960 device is not ready\n");
        
            }
            if (!cred_check(&cs)) {
                LOG_ERR("Credentials check failed");
            }
   
            while (1) {
                err = modem_info_params_get(&modem_info);
                if (!err) {
                    LOG_INF("Signal strength: %d dBm", modem_info.network.rsrp.value);
                    if (modem_info.network.rsrp.value < -100) {
                        LOG_WRN("Weak signal strength detected");
                    }
                } else {
                    LOG_ERR("Failed to get modem info: %d", err);
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

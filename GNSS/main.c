#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_gnss.h>
#include <dk_buttons_and_leds.h>
#include <modem/lte_lc.h>
#include <net/nrf_cloud.h>
#include <net/nrf_cloud_location.h>

// Register the logging module
LOG_MODULE_REGISTER(location_tracker, CONFIG_LOG_DEFAULT_LEVEL);

// Configuration for update interval (seconds)
#define UPDATE_INTERVAL_SECONDS 60

// Global flag for LTE connection status
static volatile bool lte_connected = false;

// Callback for LTE connection events
static void lte_handler(const struct lte_lc_evt *const evt)
{
    switch (evt->type) {
        case LTE_LC_EVT_NW_REG_SUCCESS:
            LOG_INF("LTE network registration successful");
            lte_connected = true;
            dk_set_led_on(DK_LED1); // Indicate LTE connection
            break;
        case LTE_LC_EVT_NW_REG_FAILED:
            LOG_ERR("LTE network registration failed");
            lte_connected = false;
            dk_set_led_off(DK_LED1);
            break;
        default:
            break;
    }
}

// Callback for GPS fix data
static void gnss_event_handler(uint32_t event)
{
    int err;
    struct nrf_modem_gnss_pvt_data_frame pvt_data;

    switch (event) {
        case NRF_MODEM_GNSS_EVT_PVT:
            // Read PVT (Position, Velocity, Time) data
            err = nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data), NRF_MODEM_GNSS_DATA_PVT);
            if (err) {
                LOG_ERR("Failed to read PVT data: %d", err);
                return;
            }

            // Check if a valid fix is obtained
            if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
                LOG_INF("GPS Fix: Lat=%.6f, Lon=%.6f, Alt=%.1f, Accuracy=%.1f",
                        pvt_data.latitude, pvt_data.longitude,
                        pvt_data.altitude, pvt_data.accuracy);

                // Prepare location data for nRF Cloud
                struct nrf_cloud_location_data location = {
                    .latitude = pvt_data.latitude,
                    .longitude = pvt_data.longitude,
                    .accuracy = pvt_data.accuracy
                };

                // Send to nRF Cloud
                err = nrf_cloud_location_send(&location);
                if (err) {
                    LOG_ERR("Failed to send location to nRF Cloud: %d", err);
                } else {
                    LOG_INF("Location sent to nRF Cloud");
                    dk_set_led_on(DK_LED2); // Indicate successful send
                    k_sleep(K_MSEC(500));
                    dk_set_led_off(DK_LED2);
                }

                // Stop GNSS to save power
                nrf_modem_gnss_stop();
            }
            break;
        case NRF_MODEM_GNSS_EVT_FIX:
            LOG_INF("GNSS fix obtained");
            break;
        case NRF_MODEM_GNSS_EVT_NMEA:
            // Ignore NMEA data for simplicity
            break;
        case NRF_MODEM_GNSS_EVT_TIMEOUT:
            LOG_WRN("GNSS timeout, no fix obtained");
            nrf_modem_gnss_stop();
            break;
        default:
            break;
    }
}

// Initialize LTE modem
static int modem_init(void)
{
    int err;

    // Initialize LTE modem
    err = lte_lc_init();
    if (err) {
        LOG_ERR("Failed to initialize LTE modem: %d", err);
        return err;
    }

    // Register LTE event handler
    lte_lc_register_handler(lte_handler);

    // Connect to LTE network
    err = lte_lc_connect_async(NULL);
    if (err) {
        LOG_ERR("Failed to initiate LTE connection: %d", err);
        return err;
    }

    return 0;
}

// Initialize GNSS (GPS)
static int gnss_init(void)
{
    int err;

    // Initialize GNSS
    err = nrf_modem_gnss_init();
    if (err) {
        LOG_ERR("Failed to initialize GNSS: %d", err);
        return err;
    }

    // Set GNSS event handler
    err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
    if (err) {
        LOG_ERR("Failed to set GNSS event handler: %d", err);
        return err;
    }

    // Configure GNSS for single-fix mode to save power
    err = nrf_modem_gnss_fix_interval_set(1);
    if (err) {
        LOG_ERR("Failed to set GNSS fix interval: %d", err);
        return err;
    }

    // Enable A-GPS for faster fixes
    err = nrf_modem_gnss_use_case_set(NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START);
    if (err) {
        LOG_ERR("Failed to set GNSS use case: %d", err);
        return err;
    }

    return 0;
}

// Initialize nRF Cloud
static int cloud_init(void)
{
    int err;

    // Initialize nRF Cloud
    err = nrf_cloud_init(NULL);
    if (err) {
        LOG_ERR("Failed to initialize nRF Cloud: %d", err);
        return err;
    }

    // Connect to nRF Cloud
    err = nrf_cloud_connect();
    if (err) {
        LOG_ERR("Failed to connect to nRF Cloud: %d", err);
        return err;
    }

    return 0;
}

void main(void)
{
    int err;

    // Initialize logging
    LOG_INF("Starting nRF9161 Location Tracker");

    // Initialize buttons and LEDs for status indication
    err = dk_leds_init();
    if (err) {
        LOG_ERR("Failed to initialize LEDs: %d", err);
        return;
    }

    // Initialize LTE modem
    err = modem_init();
    if (err) {
        LOG_ERR("Modem initialization failed, exiting");
        return;
    }

    // Wait for LTE connection
    while (!lte_connected) {
        k_sleep(K_SECONDS(1));
    }

    // Initialize nRF Cloud
    err = cloud_init();
    if (err) {
        LOG_ERR("nRF Cloud initialization failed, exiting");
        return;
    }

    // Initialize GNSS
    err = gnss_init();
    if (err) {
        LOG_ERR("GNSS initialization failed, exiting");
        return;
    }

    // Main loop
    while (1) {
        if (lte_connected) {
            // Start GNSS to acquire a fix
            LOG_INF("Starting GNSS");
            err = nrf_modem_gnss_start();
            if (err) {
                LOG_ERR("Failed to start GNSS: %d", err);
            }

            // Wait for the next update cycle
            k_sleep(K_SECONDS(UPDATE_INTERVAL_SECONDS));
        } else {
            LOG_WRN("LTE not connected, retrying...");
            k_sleep(K_SECONDS(5));
        }
    }
}

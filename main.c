#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "bsp_btn_ble.h"

// LSM6DSO Driver Headers
#include "lsm6dso_reg.h"
#include "nrfx_twi.h"
#include "lsm6dso_platform.h"
// Definitions
#define TWI_INSTANCE_ID     0
#define DEVICE_NAME         "PHW_StepCounter"
#define APP_BLE_CONN_CFG_TAG 1
#define APP_BLE_OBSERVER_PRIO 3
#define APP_ADV_INTERVAL      MSEC_TO_UNITS(25, UNIT_0_625_MS)  // 25ms interval
#define APP_ADV_DURATION      18000

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS)   // 7.5ms minimum
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(30, UNIT_1_25_MS)    // 30ms maximum  
#define SLAVE_LATENCY     0
#define CONN_SUP_TIMEOUT  MSEC_TO_UNITS(4000, UNIT_10_MS)

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

// LSM6DSO Global Context
nrfx_twi_t m_twi = NRFX_TWI_INSTANCE(TWI_INSTANCE_ID);
static stmdev_ctx_t dev_ctx;
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
static bool m_nus_ready = false;

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);
NRF_BLE_GATT_DEF(m_gatt);
NRF_BLE_QWR_DEF(m_qwr);
BLE_ADVERTISING_DEF(m_advertising);

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}};


// --- Initialization ---

void twi_init(void) 
{
    nrfx_twi_config_t twi_config = NRFX_TWI_DEFAULT_CONFIG;
    twi_config.scl = 27;
    twi_config.sda = 26; 
    twi_config.frequency = NRF_TWI_FREQ_400K;
    nrfx_twi_init(&m_twi, &twi_config, NULL, NULL);
    nrfx_twi_enable(&m_twi);
}

void lsm6dso_setup(void) 
{
    uint8_t whoami;
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.mdelay    = platform_delay_ms;
    dev_ctx.handle    = (void *)&m_twi;

    // Wait a moment for sensor to stabilize
    nrf_delay_ms(100);

    lsm6dso_device_id_get(&dev_ctx, &whoami);
    if (whoami != LSM6DSO_ID) {
        NRF_LOG_ERROR("LSM6DSO not found! WHO_AM_I: 0x%02X (Expected 0x6C)", whoami);
        // Don't loop forever here if debugging, so you can see logs
    } else {
        NRF_LOG_INFO("LSM6DSO detected successfully");
    }

    // Reset device to default
    lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
    nrf_delay_ms(10);

    // 1. Disable I3C to ensure I2C stability
    lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
    
    // 2. Enable Block Data Update (BDU)
    lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    // 3. Set XL ODR and Scale (Required: ODR >= 26Hz, Scale = 2g or 4g)
    lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_26Hz);
    lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_2g);

    // 4. Enable the Pedometer algorithm
    lsm6dso_emb_sens_t emb_sens;
    memset(&emb_sens, 0, sizeof(emb_sens));
    emb_sens.step = PROPERTY_ENABLE;
    lsm6dso_embedded_sens_set(&dev_ctx, &emb_sens);
    
    // 5. Reset step count
    lsm6dso_steps_reset(&dev_ctx);

    NRF_LOG_INFO("Pedometer initialized and reset");
}

// --- BLE Handlers ---

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    NRF_LOG_ERROR("QWR Error: 0x%X", nrf_error);
    APP_ERROR_HANDLER(nrf_error);
}

static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        NRF_LOG_INFO("Received data (ignored)");
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
        m_nus_ready = true;
        NRF_LOG_INFO("*** NUS NOTIFICATIONS ENABLED ***");
    }
    else if (p_evt->type == BLE_NUS_EVT_COMM_STOPPED)
    {
        m_nus_ready = false;
        NRF_LOG_INFO("NUS notifications disabled");
    }
    else if (p_evt->type == BLE_NUS_EVT_TX_RDY)
    {
        NRF_LOG_INFO("TX Ready");
    }
}

static void gap_params_init(void)
{
    uint32_t err_code;
    ble_gap_conn_params_t gap_conn_params = {0};
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, 
                                          (const uint8_t *)DEVICE_NAME, 
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
    
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed. MTU set to %u bytes.", 
                     p_evt->params.att_mtu_effective);
    }
}

static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    NRF_LOG_ERROR("Conn params error: 0x%X", nrf_error);
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
    uint32_t err_code;
    ble_conn_params_init_t cp_init = {0};
    
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) 
{
    uint32_t err_code;
    
    switch (p_ble_evt->header.evt_id) 
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("=== CONNECTED ===");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            m_nus_ready = false;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("=== DISCONNECTED ===");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            m_nus_ready = false;
            break;
            
        case BLE_GATTS_EVT_WRITE:
            NRF_LOG_INFO("GATT Write Event");
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            NRF_LOG_INFO("System attributes missing");
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTC_EVT_TIMEOUT:
            NRF_LOG_INFO("GATT Client Timeout");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            NRF_LOG_INFO("GATT Server Timeout");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
            
        default: 
            break;
    }
}

static void services_init(void) 
{
    uint32_t err_code;
    ble_nus_init_t nus_init = {0};
    nrf_ble_qwr_init_t qwr_init = {0};
    
    // QWR init with error handler
    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
    
    // NUS init
    nus_init.data_handler = nus_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("Services initialized");
}

static void ble_stack_init(void) 
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

static void advertising_init(void) 
{
    uint32_t err_code;
    ble_advertising_init_t init = {0};
    
    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_adv_uuids;
    
    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;
    
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

int main(void) 
{
    uint32_t err_code;
    
    // Logging first for debugging
    log_init();

    
    // Hardware Init
    NRF_LOG_INFO("Initializing TWI...");
    twi_init();
    
    NRF_LOG_INFO("Initializing LSM6DSO...");
    lsm6dso_setup();
    
    // SDK Init
    NRF_LOG_INFO("Initializing Power Management...");
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("Initializing App Timer...");
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("Initializing BLE Stack...");
    ble_stack_init();
    
    NRF_LOG_INFO("Initializing GAP...");
    gap_params_init();
    
    NRF_LOG_INFO("Initializing GATT...");
    gatt_init();
    
    NRF_LOG_INFO("Initializing Services...");
    services_init();
    
    NRF_LOG_INFO("Initializing Connection Parameters...");
    conn_params_init();
    
    NRF_LOG_INFO("Initializing Advertising...");
    advertising_init();
    
    NRF_LOG_INFO("Starting Advertising...");
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("  Device Name: %s", DEVICE_NAME);
    
    NRF_LOG_FLUSH();

    char step_string[32];
    uint16_t steps = 0;
    uint16_t prev_steps = 0xFFFF;
    uint32_t loop_count = 0;

    while (true) 
    {
        loop_count++;
        
        // Process logs first
        while (NRF_LOG_PROCESS());
        
        // Read step count
        lsm6dso_number_of_steps_get(&dev_ctx, &steps);

        // Debug heartbeat every 10 seconds
        if (loop_count % 20 == 0)
        {
            NRF_LOG_INFO("Loop #%d | Steps: %d | Connected: %s | NUS Ready: %s", 
                         loop_count,
                         steps,
                         (m_conn_handle != BLE_CONN_HANDLE_INVALID) ? "YES" : "NO",
                         m_nus_ready ? "YES" : "NO");
        }

        if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_nus_ready) 
        {
            if (steps != prev_steps) 
            {
                uint16_t len = snprintf(step_string, sizeof(step_string), "Steps: %d\r\n", steps);
                uint32_t err_code = ble_nus_data_send(&m_nus, (uint8_t*)step_string, &len, m_conn_handle);
                
                if (err_code == NRF_SUCCESS)
                {
                    NRF_LOG_INFO(">>> SENT: Steps=%d <<<", steps);
                    prev_steps = steps;
                }
                else if (err_code == NRF_ERROR_INVALID_STATE)
                {
                    NRF_LOG_WARNING("Send failed: NUS not ready");
                    m_nus_ready = false; // Reset flag
                }
                else if (err_code == NRF_ERROR_RESOURCES)
                {
                    NRF_LOG_WARNING("Send failed: No resources (buffer full)");
                }
                else if (err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
                {
                    NRF_LOG_WARNING("Send failed: System attributes missing");
                }
                else
                {
                    NRF_LOG_ERROR("Send failed: Error 0x%X", err_code);
                }
            }
        }
        
        nrf_delay_ms(500);
        nrf_pwr_mgmt_run();
    }
}
#include "lib_misc_helpers.hpp"
#include "lib_thread.hpp"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "zbh_helpers.hpp"
#include "ph_i2c.hpp"
#include "../sensors/scd40.hpp"

namespace zb
{
    constexpr uint8_t MEASURE_EP = 1;

    inline static auto g_Manufacturer = ZbStr("Orlangur");
    inline static auto g_Model = ZbStr("Co2-NG");
    inline static uint8_t g_AppVersion = 1;
    //inline static const char *TAG = "ESP_ZB_MEASURE_SENSOR";

    using clock_t = std::chrono::system_clock;

    clock_t::time_point g_co2_start_measure;

    /**********************************************************************/
    /* Cluster type definitions                                           */
    /**********************************************************************/
    using Co2Cluster_t = ZclServerCluster<MEASURE_EP, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT>;

    /**********************************************************************/
    /* Attributes types for occupancy cluster                             */
    /**********************************************************************/
    using ZclAttributeCo2MeasuredValue_t = Co2Cluster_t::Attribute<ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID , float>;

    constexpr float operator ""_ppm(unsigned long long v) { return float(v) / 1000'000.f; }
    float to_ppm(uint16_t v) { return float(v) / 1000'000.f; }

    i2c::I2CBusMaster g_i2c_bus(i2c::SDAType(gpio_num_t(10)), i2c::SCLType(gpio_num_t(11)));
    std::optional<SCD40> g_scd40;// = *SCD40::Open(i2c_bus);

    /**********************************************************************/
    /* Common zigbee network handling                                     */
    /**********************************************************************/
    static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
    {
        if (esp_zb_bdb_start_top_level_commissioning(mode_mask) != ESP_OK)
            FMT_PRINTLN("Failed to start Zigbee bdb commissioning");
    }

    extern "C" void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
    {
        uint32_t *p_sg_p     = signal_struct->p_app_signal;
        esp_err_t err_status = signal_struct->esp_err_status;
        esp_zb_app_signal_type_t sig_type = *(esp_zb_app_signal_type_t*)p_sg_p;
        static int failed_counter = 0;
        using clock_t = std::chrono::system_clock;
        static auto last_failed_counter_update = clock_t::now();
        auto reset_failure = []{
            failed_counter = 0;
            last_failed_counter_update = clock_t::now();
        };
        auto inc_failure = [](const char *pInfo){
            if (++failed_counter > 4)
            {
                FMT_PRINT("Many Failures on {}\n", pInfo);
                failed_counter = 0;
                auto n = clock_t::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(n - last_failed_counter_update).count() > 60)
                {
                    failed_counter = 1;
                    last_failed_counter_update = n;
                }else
                    esp_restart();
            }else
            {
                FMT_PRINT("Registered Failure on {}\n", pInfo);
                auto n = clock_t::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(n - last_failed_counter_update).count() > 60)
                    failed_counter = 1;
                last_failed_counter_update = n;
            }
        };
        switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            FMT_PRINTLN("Initialize Zigbee stack");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                //async setup
                FMT_PRINTLN("Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
                if (esp_zb_bdb_is_factory_new()) {
                    FMT_PRINTLN("Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    FMT_PRINTLN("Device rebooted");
                }
            } else {
                /* commissioning failed */
                inc_failure("commissioning");
                FMT_PRINTLN("Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        case ESP_ZB_ZDO_SIGNAL_LEAVE:
            FMT_PRINTLN("Got leave signal");
            esp_zb_factory_reset();
            esp_restart();
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                reset_failure();
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                FMT_PRINTLN("Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                         extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                         esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            } else {
                inc_failure("steering");
                FMT_PRINTLN("Network steering was not successful (status: %s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
            FMT_PRINTLN("Can sleep");
            break;
        default:
            FMT_PRINTLN("ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                     esp_err_to_name(err_status));
            break;
        }
    }

    static void create_measure_ep(esp_zb_ep_list_t *ep_list, uint8_t ep_id)
    {
        /**********************************************************************/
        /* Boilerplate config for standard clusters: basic, identify          */
        /**********************************************************************/
        esp_zb_basic_cluster_cfg_t basic_cfg =                                                                                \
            {                                                                                       
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,                          
                .power_source = 0x3,//battery                        
            };                                                                                      
        esp_zb_identify_cluster_cfg_t identify_cfg =                                                                             
            {                                                                                       
                .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,                   
            };                                                                                      
        esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
        esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, g_Manufacturer));
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, g_Model));
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &g_AppVersion));

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

        /**********************************************************************/
        /* CO2 cluster config (server)                                        */
        /**********************************************************************/
        {
            esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_cfg =                                                                            
            {                                                                                       
                .measured_value = 500_ppm,
                .min_measured_value = 400_ppm,
                .max_measured_value = 2000_ppm,
            };                                                                                      
            esp_zb_attribute_list_t *pCo2Attributes = esp_zb_carbon_dioxide_measurement_cluster_create(&co2_cfg);
            ESP_ERROR_CHECK(esp_zb_carbon_dioxide_measurement_cluster_add_attr(pCo2Attributes, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, &co2_cfg.measured_value));
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(cluster_list, pCo2Attributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        }

        /**********************************************************************/
        /* Endpoint configuration                                             */
        /**********************************************************************/
        esp_zb_endpoint_config_t endpoint_config = {
            .endpoint = ep_id,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
    }

    /**********************************************************************/
    /* Zigbee Task Entry Point                                            */
    /**********************************************************************/
    void zigbee_main(void *)
    {
        esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_CRITICAL, 0);
        {
            esp_zb_scheduler_queue_size_set(160);
            esp_zb_cfg_t zb_nwk_cfg = {                                                               
                .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       
                .install_code_policy = false,           
                .nwk_cfg = {
                    .zed_cfg = {.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN, .keep_alive = 10*1000}
                },                                                          
            };
            esp_zb_init(&zb_nwk_cfg);
        }

        esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

        create_measure_ep(ep_list, MEASURE_EP);
        esp_zb_device_register(ep_list);

        esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

        if (auto r = g_i2c_bus.Open(); !r)
        {
            esp_restart();
            return;
        }

        if (auto r = SCD40::Open(g_i2c_bus); !r)
        {
            esp_restart();
            return;
        }else
            g_scd40.emplace(std::move(*r));

        //TODO: run this only if start from deep sleep
        g_co2_start_measure = clock_t::now();
        if (auto r = g_scd40->start(); !r)
        {
            esp_restart();
            return;
        }

        ESP_ERROR_CHECK(esp_zb_start(false));
        FMT_PRINTLN("ZB started, looping...");
        esp_zb_stack_main_loop();
    }

    void main()
    {
        esp_zb_platform_config_t config = {
            .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE, .radio_uart_config = {}},
            .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, .host_uart_config = {}},
        };
        ESP_ERROR_CHECK(nvs_flash_init());
        FMT_PRINT("nvs_flash_init done\n");
        ESP_ERROR_CHECK(esp_zb_platform_config(&config));
        FMT_PRINT("esp_zb_platform_config done\n");
        xTaskCreate(zigbee_main, "Zigbee_main", 2*4096, NULL, thread::kPrioDefault, NULL);
    }
}

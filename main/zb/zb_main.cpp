#include "lib_misc_helpers.hpp"
#include "lib_thread.hpp"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "zbh_helpers.hpp"
#include "esp_sleep.h"
#include "ph_i2c.hpp"
#include "../sensors/scd40.hpp"
#include "driver/rtc_io.h"
#include "zboss_api.h"
#include "ph_adc.hpp"

#include "ph_board_led.hpp"

namespace colors
{
    /**********************************************************************/
    /* Colors and patterns                                                */
    /**********************************************************************/
    static constexpr led::Color kColorError{255, 0, 0};
    static constexpr led::Color kColorFactoryReset{0, 255, 255};

    static constexpr uint32_t kBlinkPatternFactoryReset = 0x0F00F00F;
    static constexpr uint32_t kBlinkPatternZStackError = 0x0F00F00F;
    static constexpr uint32_t kBlinkPatternSteeringError = 0x0000F00F;
}

template<>
struct tools::formatter_t<esp_sleep_source_t>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, esp_sleep_source_t const& v)
    {
#define ESP_SLEEP_SRC_OUTPUT(x) dst(x); return sizeof(x) - 1
        switch(v)
        {
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_UNDEFINED: ESP_SLEEP_SRC_OUTPUT("Undefined");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_ALL: ESP_SLEEP_SRC_OUTPUT("All");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_EXT0: ESP_SLEEP_SRC_OUTPUT("Ext0");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_EXT1: ESP_SLEEP_SRC_OUTPUT("Ext1");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_TIMER: ESP_SLEEP_SRC_OUTPUT("Timer");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_TOUCHPAD: ESP_SLEEP_SRC_OUTPUT("Touchpad");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_ULP: ESP_SLEEP_SRC_OUTPUT("ULP");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_GPIO: ESP_SLEEP_SRC_OUTPUT("GPIO");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_UART: ESP_SLEEP_SRC_OUTPUT("UART");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_WIFI: ESP_SLEEP_SRC_OUTPUT("WiFi");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_COCPU: ESP_SLEEP_SRC_OUTPUT("CoCPU");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG: ESP_SLEEP_SRC_OUTPUT("CoCPU_Trap");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_BT: ESP_SLEEP_SRC_OUTPUT("BT");
            case esp_sleep_source_t::ESP_SLEEP_WAKEUP_VAD: ESP_SLEEP_SRC_OUTPUT("VAD");
            default:  ESP_SLEEP_SRC_OUTPUT("<unknown>");
        }
    }
};

namespace zb
{
    constexpr uint8_t MEASURE_EP = 1;

    inline static auto g_Manufacturer = ZbStr("Orlangur");
    inline static auto g_Model = ZbStr("Co2-NG");
    inline static uint8_t g_AppVersion = 1;
    //inline static const char *TAG = "ESP_ZB_MEASURE_SENSOR";

#if CONFIG_IDF_TARGET_ESP32C6
        /* For ESP32C6 boards, RTCIO only supports GPIO0~GPIO7 */
        /* GPIO7 pull down to wake up */
        static constexpr gpio_num_t gpio_wakeup_pin = gpio_num_t(7);
#elif CONFIG_IDF_TARGET_ESP32H2
        /* You can wake up by pulling down GPIO9. On ESP32H2 development boards, the BOOT button is connected to GPIO9.
           You can use the BOOT button to wake up the boards directly.*/
        static constexpr gpio_num_t gpio_wakeup_pin = gpio_num_t(9);
#endif

    using clock_t = std::chrono::system_clock;

    clock_t::time_point g_co2_start_measure;

    /**********************************************************************/
    /* Cluster type definitions                                           */
    /**********************************************************************/
    using Co2Cluster_t = ZclServerCluster<MEASURE_EP, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT>;
    using TempCluster_t = ZclServerCluster<MEASURE_EP, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT>;
    using RHCluster_t = ZclServerCluster<MEASURE_EP, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT>;
    using PowerConfigCluster_t = ZclServerCluster<MEASURE_EP, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG>;

    /**********************************************************************/
    /* Attributes types for occupancy cluster                             */
    /**********************************************************************/
    using ZclAttributeCo2MeasuredValue_t = Co2Cluster_t::Attribute<ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID , float>;
    using ZclAttributeTempMeasuredValue_t = TempCluster_t::Attribute<ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID , int16_t>;
    using ZclAttributeRHMeasuredValue_t = RHCluster_t::Attribute<ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID , uint16_t>;
    //using ZclAttributeBatteryVoltage_t = PowerConfigCluster_t::Attribute<ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, uint8_t>;

    constexpr ZclAttributeCo2MeasuredValue_t g_Co2Attr;
    constexpr ZclAttributeTempMeasuredValue_t g_TempAttr;
    constexpr ZclAttributeRHMeasuredValue_t g_RHAttr;
    //constexpr ZclAttributeBatteryVoltage_t g_BatVoltage;

    constexpr float operator ""_ppm(unsigned long long v) { return float(v) / 1000'000.f; }
    float to_ppm(uint16_t v) { return float(v) / 1000'000.f; }

    constexpr int16_t operator ""_deg(long double v) { return v * 100; }
    int16_t to_deg(float v) { return v * 100; }

    constexpr uint16_t operator ""_rh(long double v) { return v * 100; }
    uint16_t to_rh(float v) { return v * 100; }

    i2c::I2CBusMaster g_i2c_bus(i2c::SDAType(gpio_num_t(10)), i2c::SCLType(gpio_num_t(11)));
    std::optional<SCD40> g_scd40;// = *SCD40::Open(i2c_bus);
    RTC_DATA_ATTR uint16_t g_DeepSleepCO2;
    RTC_DATA_ATTR int16_t g_DeepSleepTemp;
    RTC_DATA_ATTR uint16_t g_DeepSleepRH;

    static void config_deep_sleep()
    {
        const int wakeup_time_sec = 60;
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));

        const uint64_t gpio_wakeup_pin_mask = 1ULL << gpio_wakeup_pin;

        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(gpio_wakeup_pin_mask, ESP_EXT1_WAKEUP_ANY_LOW));
        ESP_ERROR_CHECK(gpio_pullup_en(gpio_wakeup_pin));
        ESP_ERROR_CHECK(gpio_pulldown_dis(gpio_wakeup_pin));
    }

    static void enter_deep_sleep(uint8_t)
    {
        FMT_PRINTLN("Entering deep sleep...");
        esp_deep_sleep_start();
    }

    static void attempt_to_read_co2_measurements(uint8_t)
    {
        FMT_PRINTLN("attempt_to_read_co2_measurements");
        if (auto r = g_scd40->is_data_ready(); !r)
        {
            FMT_PRINTLN("attempt_to_read_co2_measurements: data ready error: {}", r.error());
            //failure
        }else if (*r)
        {
            FMT_PRINTLN("attempt_to_read_co2_measurements: data ready. Reading...");
            if (auto x = g_scd40->read_measurements(); !x)
            {
                //failure
                FMT_PRINTLN("attempt_to_read_co2_measurements: Reading error: {}", x.error());
            }else
            {
                g_Co2Attr.Set(to_ppm(g_DeepSleepCO2 = x->co2));
                g_TempAttr.Set(g_DeepSleepTemp = to_deg(x->temp));
                g_RHAttr.Set(g_DeepSleepRH = to_rh(x->rh));
                FMT_PRINTLN("attempt_to_read_co2_measurements: Reading done with: {}ppm {:.2}C {:.2}%", g_DeepSleepCO2, x->temp, x->rh);
                if (auto stop_r = g_scd40->stop(); !stop_r)
                {
                    FMT_PRINTLN("attempt_to_read_co2_measurements: stop error: {}", stop_r.error());
                }
                //g_BatVoltage.Set(16);//1.6V
                esp_zb_scheduler_alarm(enter_deep_sleep, 0, 5000);
            }
        }else
        {
            FMT_PRINTLN("attempt_to_read_co2_measurements: data not ready. Waiting 500ms...");
            //re-schedule timer
            esp_zb_scheduler_alarm(attempt_to_read_co2_measurements, 0, 500);
        }
    }

    /**********************************************************************/
    /* Common zigbee network handling                                     */
    /**********************************************************************/
    static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
    {
        if (esp_zb_bdb_start_top_level_commissioning(mode_mask) != ESP_OK)
        {
            FMT_PRINTLN("Failed to start Zigbee bdb commissioning");
            esp_restart();
        }
    }

    extern "C" void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
    {
        uint32_t *p_sg_p     = signal_struct->p_app_signal;
        esp_err_t err_status = signal_struct->esp_err_status;
        esp_zb_app_signal_type_t sig_type = *(esp_zb_app_signal_type_t*)p_sg_p;
        static int failed_counter = 0;
        using clock_t = std::chrono::system_clock;
        static auto last_failed_counter_update = clock_t::now();
        static clock_t::time_point steering_start;
        auto reset_failure = []{
            failed_counter = 0;
            last_failed_counter_update = clock_t::now();
        };
        auto inc_failure = [](const char *pInfo){
            if (++failed_counter > 10)
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

        auto check_co2_measurements = []{
            auto now = clock_t::now();

            using namespace std::chrono_literals;
            if ((now - g_co2_start_measure) >= 5s)
            {
                FMT_PRINTLN("Time passed since co2 measure start: {}ms; Attempting to read", std::chrono::duration_cast<std::chrono::milliseconds>(now - g_co2_start_measure).count());
                attempt_to_read_co2_measurements(0);
            }else
            {
                FMT_PRINTLN("Time passed since co2 measure start less than 5s: sleeping {}ms more", std::chrono::duration_cast<std::chrono::milliseconds>(5s - (now - g_co2_start_measure)).count());
                esp_zb_scheduler_alarm(attempt_to_read_co2_measurements, 0, std::chrono::duration_cast<std::chrono::milliseconds>(5s - (now - g_co2_start_measure)).count());
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
                FMT_PRINTLN("Device started up in {} factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
                if (esp_zb_bdb_is_factory_new()) {
                    FMT_PRINTLN("Start network steering");
                    steering_start = clock_t::now();
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    FMT_PRINTLN("Device rebooted");
                    auto wakeup_cause = esp_sleep_get_wakeup_cause();
                    FMT_PRINTLN("deep sleep wake up cause: {}", wakeup_cause);
                    check_co2_measurements();
                };
            } else {
                /* commissioning failed */
                inc_failure("commissioning");
                FMT_PRINTLN("Failed to initialize Zigbee stack (status: {})", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
                led::blink_pattern(colors::kBlinkPatternZStackError, colors::kColorError, duration_ms_t(700));
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
                FMT_PRINTLN("Joined network successfully (Extended PAN ID: {}, PAN ID: 0x{:x}, Channel:{}, Short Address: 0x{:x})",
                         std::span<uint8_t>(extended_pan_id, 8)
                         , esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
                check_co2_measurements();
            } else {
                //inc_failure("steering");
                auto n = clock_t::now();
                using namespace std::chrono_literals;
                if ((n - steering_start) >= 10s)
                {
                    //enter a long deep sleep to wake up only on 'boot' click
                    esp_sleep_disable_wakeup_source(esp_sleep_source_t::ESP_SLEEP_WAKEUP_TIMER);
                    esp_deep_sleep_start();//wake only by a button press
                    return;
                }
                FMT_PRINTLN("Network steering was not successful (status: {})", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
                led::blink_pattern(colors::kBlinkPatternSteeringError, colors::kColorError, duration_ms_t(700));
            }
            break;
        default:
            FMT_PRINTLN("ZDO signal: {} (0x{:x}), status: {}", esp_zb_zdo_signal_to_string(sig_type), sig_type,
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

        auto wakeup_cause = esp_sleep_get_wakeup_cause();
        FMT_PRINTLN("deep sleep wake up cause: {}", wakeup_cause);

        {
            uint8_t batV = 0xff;//measure here
            uint8_t batPercentage = 0xff; 
            adc::OneShot os(adc_channel_t::ADC_CHANNEL_0);
            if (os.valid())
            {
                auto mv = os.read();
                batV = mv / 100;
                constexpr auto maxVLevel = 1600;
                constexpr auto minVLevel = 1200;
                auto maxDeltaV = maxVLevel - minVLevel;
                auto currentDelta = (maxVLevel - mv);
                if (currentDelta < 0) currentDelta = 0;
                if (currentDelta > maxDeltaV) currentDelta = maxDeltaV;
                currentDelta = maxDeltaV - currentDelta;
                batPercentage = 200 * currentDelta / maxDeltaV;
                FMT_PRINTLN("batV: {} 100mV; percentage: {}%", batV, batPercentage / 2);
            }
            esp_zb_power_config_cluster_cfg_t power_cfg = {};
            esp_zb_attribute_list_t *pElectricMeasAttributes = esp_zb_power_config_cluster_create(&power_cfg);
            esp_zb_power_config_cluster_add_attr(pElectricMeasAttributes, ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &batV);
            esp_zb_power_config_cluster_add_attr(pElectricMeasAttributes, ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, &batPercentage);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(cluster_list, pElectricMeasAttributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        }
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
            
            if (wakeup_cause != ESP_SLEEP_WAKEUP_UNDEFINED)
                co2_cfg.measured_value = to_ppm(g_DeepSleepCO2);//previous measured value

            esp_zb_attribute_list_t *pCo2Attributes = esp_zb_carbon_dioxide_measurement_cluster_create(&co2_cfg);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(cluster_list, pCo2Attributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        }
        {
            esp_zb_temperature_meas_cluster_cfg_t temp_cfg =                                                                            
            {                                                                                       
                .measured_value = 25.0_deg,
                .min_value = -25.0_deg,
                .max_value = 60.0_deg,
            };                                                                                      
            
            if (wakeup_cause != ESP_SLEEP_WAKEUP_UNDEFINED)
                temp_cfg.measured_value = g_DeepSleepTemp;//previous measured value

            esp_zb_attribute_list_t *pTempAttributes = esp_zb_temperature_meas_cluster_create(&temp_cfg);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, pTempAttributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        }
        {
            esp_zb_humidity_meas_cluster_cfg_t rh_cfg =                                                                            
            {                                                                                       
                .measured_value = 25.0_rh,
                .min_value = 0.0_rh,
                .max_value = 100.0_rh,
            };                                                                                      
            
            if (wakeup_cause != ESP_SLEEP_WAKEUP_UNDEFINED)
                rh_cfg.measured_value = g_DeepSleepRH;//previous measured value

            esp_zb_attribute_list_t *pRHAttributes = esp_zb_humidity_meas_cluster_create(&rh_cfg);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, pRHAttributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
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

        if (gpio_get_level(gpio_wakeup_pin) == 0)//pressed
        {
            int secs_to_factory_reset = 50;//50 * 100ms
            while((gpio_get_level(gpio_wakeup_pin) == 0) && (--secs_to_factory_reset))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (!secs_to_factory_reset)
            {
                led::blink_pattern(colors::kBlinkPatternFactoryReset, colors::kColorFactoryReset, duration_ms_t(1000));
                esp_zb_factory_reset();
                esp_restart();
                return;
            }
        }

        esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

        create_measure_ep(ep_list, MEASURE_EP);
        config_deep_sleep();
        esp_zb_device_register(ep_list);

        esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

        if (auto r = g_i2c_bus.Open(); !r)
        {
            FMT_PRINTLN("Failed to open i2c bus");
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            esp_restart();
            return;
        }

        if (auto r = SCD40::Open(g_i2c_bus); !r)
        {
            FMT_PRINTLN("Failed to open SCD40 with error: {}", r.error());
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            esp_restart();
            return;
        }else
            g_scd40.emplace(std::move(*r));

        //TODO: run this only if start from deep sleep
        g_co2_start_measure = clock_t::now();
        if (auto r = g_scd40->start(); !r)
        {
            FMT_PRINTLN("Failed to start SCD40 measuring with error: {}", r.error());
            esp_restart();
            return;
        }

        ESP_ERROR_CHECK(esp_zb_start(false));
        FMT_PRINTLN("ZB started, looping...");
        esp_zb_stack_main_loop();
    }

    void main()
    {
        led::setup();
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

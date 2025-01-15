/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"

#include "sensors/aht21.hpp"
#include "sensors/ens160.hpp"
#include <thread>

extern "C" void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    auto print_error = [](auto &e) { printf("i2c Error at %s: %s\n", e.pLocation, esp_err_to_name(e.code)); fflush(stdout); };

    i2c::I2CBusMaster bus(i2c::SDAType(gpio_num_t(10)), i2c::SCLType(gpio_num_t(11)));
    auto r = bus.Open();
    if (!r)
    {
        print_error(r.error());
        return;
    }
    printf("I2C bus opened\n");

    auto print_aht21_error = [&](auto &e) 
    { 
        printf("AHT21 Error at %s: %s\n", e.pLocation, AHT21::err_to_str(e.code));
        print_error(e.i2cErr);
    };
    auto print_ens160_error = [&](auto &e) 
    { 
        printf("ENS160 Error %s\n", ENS160::err_to_str(e.code));
        print_error(e.i2cErr);
    };

    auto print_ens160_status = [](ENS160::Status s) 
    { 
        printf("Status:\n");
        printf("New GPR: %d\n", int(s.new_gpr));
        printf("New Data: %d\n", int(s.new_data));
        printf("Validity: %d\n", int(s.validity));
        printf("Error: %d\n", int(s.error));
        printf("State: %d\n", int(s.state));
    };

    std::this_thread::sleep_for(std::chrono::seconds(1));
    ENS160 ens160(bus);
    if (auto r = ens160.GetOpMode(); !r)
    {
        print_ens160_error(r.error());
        return;
    }else
    {
        printf("ENS160: Current mode: %x\n", int(*r));
    }

    ENS160::Status stat;
    if (auto r = ens160.GetStatus(); !r)
    {
        print_ens160_error(r.error());
        return;
    }else
        stat = *r;
    print_ens160_status(stat);

    if (auto r = ens160.GetVersion(); !r)
    {
        print_ens160_error(r.error());
        return;
    }else
    {
        printf("Version: %d.%d.%d\n", r->maj, r->min, r->rel);
    }

    if (auto r = ens160.GoToSensing()/*DoOneShotMeasurements()*/; !r)
    {
        print_ens160_error(r.error());
        return;
    }

    AHT21 sensor(bus);
    if (auto r = sensor.Init(); !r)
    {
        print_aht21_error(r.error());
        return;
    }

    while(true)
    {
        if (auto r = sensor.UpdateMeasurements(); !r)
        {
            print_aht21_error(r.error());
            return;
        }

        auto r = sensor.GetLastMeasurements();
        if (r)
        {
            auto [_t, _h] = r.value();
            printf("AHT21 Temp: %f; Hum: %f\n", _t, _h);
            if (auto tr = ens160.WriteHostTemperature(_t); !tr)
            {
                print_ens160_error(tr.error());
                return;
            }
            if (auto tr = ens160.WriteHostRelativeHumidity(_h); !tr)
            {
                print_ens160_error(tr.error());
                return;
            }
        }

        printf("Measurements:\n");
        if (auto r = ens160.ReadTemperature(); !r)
        {
            print_ens160_error(r.error());
            return;
        }else
            printf("Temp: %.2f\n", *r);

        if (auto r = ens160.ReadRelativeHumidity(); !r)
        {
            print_ens160_error(r.error());
            return;
        }else
            printf("RH: %.2f%%\n", *r);

        if (auto r = ens160.ReadHostTemperature(); !r)
        {
            print_ens160_error(r.error());
            return;
        }else
            printf("Host Temp: %.2f\n", *r);

        if (auto r = ens160.ReadHostRelativeHumidity(); !r)
        {
            print_ens160_error(r.error());
            return;
        }else
            printf("Host RH: %.2f%%\n", *r);

        if (auto r = ens160.ReadAirQualityIndex(); !r)
        {
            print_ens160_error(r.error());
            return;
        }else
            printf("AQI: %d\n", *r);

        if (auto r = ens160.ReadTVOC(); !r)
        {
            print_ens160_error(r.error());
            return;
        }else
            printf("TVOC: %d\n", *r);

        if (auto r = ens160.ReadeCO2(); !r)
        {
            print_ens160_error(r.error());
            return;
        }else
            printf("eCO2: %d\n", *r);
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));


    printf("AHT21 initialized\n");
    std::atomic_flag sensorUpdateRunning{true};
    std::jthread sensor_update([&]{
            while(true)
            {
                if (auto r = sensor.UpdateMeasurements(); !r)
                {
                    print_aht21_error(r.error());
                    sensorUpdateRunning.clear(std::memory_order_relaxed);
                    return;
                }
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }
    });
        
    while(sensorUpdateRunning.test(std::memory_order_relaxed))
    {
        auto r = sensor.GetLastMeasurements();
        if (r)
        {
            auto [_t, _h] = r.value();
            printf("Temp: %f; Hum: %f\n", _t, _h);
            fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

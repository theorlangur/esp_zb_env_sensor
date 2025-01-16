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

void test_bmp280(i2c::I2CBusMaster &bus)
{
    auto d = *bus.Add(0x76);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    uint8_t val = d.ReadReg8(0xd0)->v;
    printf("BMP280\n");
    printf("Id: %x\n", val);
    fflush(stdout);
    //std::this_thread::sleep_for(std::chrono::seconds(20));

    using dig_T1_r = i2c::helpers::RegisterMultiByte<uint16_t, 0x88, i2c::helpers::RegAccess::Read>;
    using dig_T2_r = i2c::helpers::RegisterMultiByte<int16_t, 0x8a, i2c::helpers::RegAccess::Read>;
    using dig_T3_r = i2c::helpers::RegisterMultiByte<int16_t, 0x8c, i2c::helpers::RegAccess::Read>;
    using dig_P1_r = i2c::helpers::RegisterMultiByte<uint16_t, 0x8e, i2c::helpers::RegAccess::Read>;
    using dig_P2_r = i2c::helpers::RegisterMultiByte<int16_t, 0x90, i2c::helpers::RegAccess::Read>;
    using dig_P3_r = i2c::helpers::RegisterMultiByte<int16_t, 0x92, i2c::helpers::RegAccess::Read>;
    using dig_P4_r = i2c::helpers::RegisterMultiByte<int16_t, 0x94, i2c::helpers::RegAccess::Read>;
    using dig_P5_r = i2c::helpers::RegisterMultiByte<int16_t, 0x96, i2c::helpers::RegAccess::Read>;
    using dig_P6_r = i2c::helpers::RegisterMultiByte<int16_t, 0x98, i2c::helpers::RegAccess::Read>;
    using dig_P7_r = i2c::helpers::RegisterMultiByte<int16_t, 0x9a, i2c::helpers::RegAccess::Read>;
    using dig_P8_r = i2c::helpers::RegisterMultiByte<int16_t, 0x9c, i2c::helpers::RegAccess::Read>;
    using dig_P9_r = i2c::helpers::RegisterMultiByte<int16_t, 0x9e, i2c::helpers::RegAccess::Read>;

    uint16_t dig_T1, dig_P1;
    dig_T1_r{d}.Read(dig_T1);
    int16_t dig_T2, dig_T3;
    dig_T2_r{d}.Read(dig_T2);
    dig_T3_r{d}.Read(dig_T3);
    printf("BMP280: Temp calibration: T1=x%x; T2=x%x; T3=x%x\n", dig_T1, dig_T2, dig_T3);
    
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    dig_P1_r{d}.Read(dig_P1);
    dig_P2_r{d}.Read(dig_P2);
    dig_P3_r{d}.Read(dig_P3);
    dig_P4_r{d}.Read(dig_P4);
    dig_P5_r{d}.Read(dig_P5);
    dig_P6_r{d}.Read(dig_P6);
    dig_P7_r{d}.Read(dig_P7);
    dig_P8_r{d}.Read(dig_P8);
    dig_P9_r{d}.Read(dig_P9);
    printf("BMP280: Press calibration: P1=x%x; P2=x%x; P3=x%x; P4=x%x; P5=x%x; P6=x%x; P7=x%x; P8=x%x; P9=x%x\n", dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9);

    enum class Oversampling: uint8_t
    {
        Disabled = 0b000,
        O1 = 0b001,
        O2 = 0b010,
        O4 = 0b011,
        O8 = 0b100,
        O16 = 0b101,
        O16_2 = 0b110,
        O16_3 = 0b111,
    };
    struct ctrl_meas
    {
        uint8_t mode: 2 = 0b11;
        Oversampling oversampling_press: 3 = Oversampling::O1;
        Oversampling oversampling_temp: 3 = Oversampling::O1;
    };
    using ctrl_meas_r = i2c::helpers::Register<ctrl_meas, 0xf4, i2c::helpers::RegAccess::RW>;

    ctrl_meas_r{d}.Write(ctrl_meas{});

    struct temp_data{
        uint8_t raw[3];
    };
    int32_t t_fine;
    auto get_temp = [&](const temp_data &td)->float
    {
        int32_t adc_T = int32_t(td.raw[0]) << 12 | int32_t(td.raw[1]) << 4 | (td.raw[2] >> 4);
        int32_t var1, var2, T;
        var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1 ))) * ((int32_t)dig_T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        t_fine = var1 + var2;
        T = (t_fine * 5 + 128) >> 8;
        return float(T) / 100.f;
    };
    struct press_data{
        uint8_t raw[3];
    };
    auto get_press = [&](const press_data &pd)->float
    {
        int32_t adc_P = int32_t(pd.raw[0]) << 12 | int32_t(pd.raw[1]) << 4 | (pd.raw[2] >> 4);
        int64_t var1, var2, p;
        var1 = ((int64_t)t_fine) - 128000;
        var2 = var1 * var1 * (int64_t)dig_P6;
        var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
        var2 = var2 + (((int64_t)dig_P4) << 35);
        var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
        if (var1 == 0)
            return 0;
        p = 1048576 - adc_P;
        p = (((p << 31) - var2) * 3125)/var1;
        var1 = (((int64_t)dig_P9) * (p>>13)* (p>>13)) >> 25;
        var2 = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
        return float(p) / 256;
    };
    struct press_temp_data{
        press_data press;
        temp_data temp;
    };
    using press_temp_r = i2c::helpers::RegisterMultiByte<press_temp_data, 0xf7, i2c::helpers::RegAccess::Read>;

    struct status
    {
        uint8_t im_update: 1;
        uint8_t reserved : 2;
        uint8_t measuring: 1;
        uint8_t reserved2: 4;
    };

    enum class StandbyT: uint8_t
    {
        _0_5ms  = 0b000,
        _62_5ms = 0b001,
        _125ms  = 0b010,
        _250ms  = 0b011,
        _500ms  = 0b100,
        _1000ms = 0b101,
        _2000ms = 0b110,
        _4000ms = 0b111,
    };

    enum class Filter: uint8_t
    {
        Off = 0,
        _2  = 1,
        _4  = 2,
        _8  = 3,
        _16 = 4,
    };

    struct config
    {
        uint8_t spi3       : 1 = 0;
        uint8_t unused     : 1 = 0;
        Filter filter      : 3 = Filter::Off;
        StandbyT t_standby : 3 = StandbyT::_500ms;
    };
    //using status_r = i2c::helpers::Register<status, 0xf5, i2c::helpers::RegAccess::Read>;
    while(true)
    {
        //status s = *status_r{d}.Read();
        //printf("BMP280 Status before read: im update=%d; measuring=%d\n", int(s.im_update), int(s.measuring));
        press_temp_data raw_mixed;
        press_temp_r{d}.Read(raw_mixed);
        printf("BMP280: Temp %.2f; Press: %.2fPa\n", get_temp(raw_mixed.temp), get_press(raw_mixed.press));
        //s = *status_r{d}.Read();
        //printf("BMP280 Status after read: im update=%d; measuring=%d\n", int(s.im_update), int(s.measuring));
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

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
    test_bmp280(bus);

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

#include "scd40.hpp"
#include <thread>

const char* SCD40::err_to_str(ErrorCode e)
{
    switch(e)
    {
        case ErrorCode::Ok: return "Ok";
        case ErrorCode::Open: return "Open";
        case ErrorCode::ReadMeasurements: return "ReadMeasurements";
        case ErrorCode::GetSensorType: return "GetSensorType";
        case ErrorCode::GetSerialId: return "GetSerialId";
        case ErrorCode::Start: return "Start";
        case ErrorCode::StartLowPower: return "StartLowPower";
        case ErrorCode::Stop: return "Stop";
        case ErrorCode::IsDataReady: return "IsDataReady";
        case ErrorCode::WaitDataReady: return "WaitDataReady";
        case ErrorCode::SetTempOffset: return "SetTempOffset";
        case ErrorCode::GetTempOffset: return "GetTempOffset";
        case ErrorCode::SetAltitude: return "SetAltitude";
        case ErrorCode::GetAltitude: return "GetAltitude";
        case ErrorCode::SetPressure: return "SetPressure";
        case ErrorCode::GetPressure: return "GetPressure";
        case ErrorCode::StoreSettings: return "StoreSettings";
        case ErrorCode::FactoryReset: return "FactoryReset";
        case ErrorCode::ReInit: return "ReInit";
        case ErrorCode::SelfTest: return "SelfTest";
        case ErrorCode::Recalibrate: return "Recalibrate";
        case ErrorCode::SetAutoSelfCalibration: return "SetAutoSelfCalibration";
        case ErrorCode::GetAutoSelfCalibration: return "GetAutoSelfCalibration";
        case ErrorCode::SetAutoSelfCalibrationBaseline: return "SetAutoSelfCalibrationBaseline";
        case ErrorCode::GetAutoSelfCalibrationBaseline: return "GetAutoSelfCalibrationBaseline";
    }
}

SCD40::ExpectedValue<SCD40> SCD40::Open(i2c::I2CBusMaster &bus)
{
    auto r = bus.Add(kAddress);
    if (!r)
        return std::unexpected(Err{r.error(), ErrorCode::Open});
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    SCD40 res(std::move(*r));
    if (auto sr = res.stop(); !sr)
        return std::unexpected(sr.error());
    return std::move(res);
}

uint8_t SCD40::calc_crc(const word_t &w)
{
    constexpr uint8_t CRC8_POLYNOMIAL = 0x31;
    constexpr uint8_t CRC8_INIT = 0xff;

    const uint8_t *data = &w.msb;
    uint8_t crc = CRC8_INIT;
    for(uint16_t current_byte = 0; current_byte < (sizeof(word_t) - 1); ++current_byte)
    {
        crc ^= data[current_byte];
        for(uint8_t crc_bit = 8; crc_bit > 0;--crc_bit)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc <<= 1;
        }
    }
    return crc;
}

bool SCD40::check_crc(word_t &data)
{
    return calc_crc(data) == data.crc;
}

SCD40::SCD40(i2c::I2CDevice &&d): m_Device(std::move(d))
{
}

SCD40::ExpectedValue<SCD40::measurement_results_t> SCD40::read_measurements()
{
    if (!m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::read_measurements", .code = ESP_ERR_INVALID_STATE}, ErrorCode::ReadMeasurements});
    if (auto res = read_measurement{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::ReadMeasurements});
    else
    {
        auto &r = *res;
        return measurement_results_t{.co2 = r.co2, .temp = r.get_temp(), .rh = r.get_humidity()};
    }
}

SCD40::ExpectedValue<SCD40::SensorType> SCD40::get_sensor_type()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::get_sensor_type", .code = ESP_ERR_INVALID_STATE}, ErrorCode::GetSensorType});
    if (auto res = get_sensor_variant{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::GetSensorType});
    else
        return SCD40::SensorType{res->type};
}

SCD40::ExpectedValue<SCD40::serial_t> SCD40::get_serial_id()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::get_serial_id", .code = ESP_ERR_INVALID_STATE}, ErrorCode::GetSerialId});
    if (auto res = get_serial_number{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::GetSerialId});
    else
        return *res;
}

SCD40::ExpectedResult SCD40::start()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::start", .code = ESP_ERR_INVALID_STATE}, ErrorCode::Start});
    if (auto res = start_periodic_measurement{m_Device}.Send(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::Start});
    m_Measuring = true;
    return std::ref(*this);
}

SCD40::ExpectedResult SCD40::start_low_power()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::start_low_power", .code = ESP_ERR_INVALID_STATE}, ErrorCode::StartLowPower});
    if (auto res = start_low_power_periodic_measurement{m_Device}.Send(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::StartLowPower});
    m_Measuring = true;
    return std::ref(*this);
}

SCD40::ExpectedResult SCD40::stop()
{
    if (auto res = stop_periodic_measurement{m_Device}.Send(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::Stop});

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    m_Measuring = false;
    return std::ref(*this);
}

SCD40::ExpectedValue<bool> SCD40::is_data_ready()
{
    if (!m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::is_data_ready", .code = ESP_ERR_INVALID_STATE}, ErrorCode::IsDataReady});
    if (auto res = get_data_ready_status{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::IsDataReady});
    else
        return res->status != 0;
}

SCD40::ExpectedResult SCD40::wait_until_data_ready(i2c::duration_t d)
{
    if (!m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::wait_until_data_ready", .code = ESP_ERR_INVALID_STATE}, ErrorCode::WaitDataReady});
    using clock_t = std::chrono::system_clock;
    auto start_wait_time = clock_t::now();
    while(true)
    {
        if (auto res = get_data_ready_status{m_Device}.Recv(); !res)
            return std::unexpected(Err{res.error(), ErrorCode::WaitDataReady});
        else if (res->status != 0)
            return std::ref(*this);

        if (d != kForever && (std::chrono::duration_cast<std::chrono::milliseconds>(clock_t::now() - start_wait_time) > d))
            break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }
    return std::unexpected(Err{::Err{.pLocation="SCD40::wait_until_data_ready", .code = ESP_ERR_TIMEOUT}, ErrorCode::WaitDataReady});
}

SCD40::ExpectedResult SCD40::set_temp_offset(float off)
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::set_temp_offset", .code = ESP_ERR_INVALID_STATE}, ErrorCode::SetTempOffset});
    if (auto res = set_temperature_offset{m_Device}.Send(off); !res)
        return std::unexpected(Err{res.error(), ErrorCode::SetTempOffset});
    return std::ref(*this);
}

SCD40::ExpectedValue<float> SCD40::get_temp_offset()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::get_temp_offset", .code = ESP_ERR_INVALID_STATE}, ErrorCode::GetTempOffset});
    if (auto res = get_temperature_offset{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::GetTempOffset});
    else 
        return res->get_temp_offset();
}

SCD40::ExpectedResult SCD40::set_altitude(uint16_t a)
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::set_altitude", .code = ESP_ERR_INVALID_STATE}, ErrorCode::SetAltitude});
    if (auto res = set_sensor_altitude{m_Device}.Send(a); !res)
        return std::unexpected(Err{res.error(), ErrorCode::SetAltitude});
    return std::ref(*this);
}

SCD40::ExpectedValue<uint16_t> SCD40::get_altitude()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::get_altitude", .code = ESP_ERR_INVALID_STATE}, ErrorCode::GetAltitude});
    if (auto res = get_sensor_altitude{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::GetAltitude});
    else 
        return *res;
}

SCD40::ExpectedResult SCD40::set_pressure(float p)
{
    if (!m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::set_pressure", .code = ESP_ERR_INVALID_STATE}, ErrorCode::SetPressure});
    if (auto res = set_ambient_pressure{m_Device}.Send(p); !res)
        return std::unexpected(Err{res.error(), ErrorCode::SetPressure});
    return std::ref(*this);
}

SCD40::ExpectedValue<float> SCD40::get_pressure()
{
    if (!m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::get_pressure", .code = ESP_ERR_INVALID_STATE}, ErrorCode::GetPressure});
    if (auto res = get_ambient_pressure{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::GetPressure});
    else 
        return res->get();
}

SCD40::ExpectedResult SCD40::store_settings()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::store_settings", .code = ESP_ERR_INVALID_STATE}, ErrorCode::StoreSettings});
    if (auto res = save_settings{m_Device}.Send(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::StoreSettings});
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    return std::ref(*this);
}

SCD40::ExpectedResult SCD40::factory_reset()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::factory_reset", .code = ESP_ERR_INVALID_STATE}, ErrorCode::FactoryReset});
    if (auto res = perform_factory_reset{m_Device}.Send(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::FactoryReset});
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    return std::ref(*this);
}

SCD40::ExpectedResult SCD40::re_init()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::re_init", .code = ESP_ERR_INVALID_STATE}, ErrorCode::ReInit});
    if (auto res = reinit{m_Device}.Send(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::ReInit});
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    return std::ref(*this);
}

SCD40::ExpectedValue<bool> SCD40::self_test()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::self_test", .code = ESP_ERR_INVALID_STATE}, ErrorCode::SelfTest});
    if (auto res = perform_self_test{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::SelfTest});
    else//10seconds???
        return *res == 0;
}

SCD40::ExpectedValue<uint16_t> SCD40::recalibrate(uint16_t new_reference_co2)
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::recalibrate", .code = ESP_ERR_INVALID_STATE}, ErrorCode::Recalibrate});
    if (auto res = perform_forced_recalibration{m_Device}.Transmit(new_reference_co2); !res)
        return std::unexpected(Err{res.error(), ErrorCode::Recalibrate});
    else if (res->valid())
        return res->get();
    else
        return std::unexpected(Err{::Err{.pLocation = "SCD40::recalibrate", .code = ESP_FAIL}, ErrorCode::Recalibrate});
}

SCD40::ExpectedResult SCD40::set_auto_self_calibration(AutoSelfCalibration v)
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::set_auto_self_calibration", .code = ESP_ERR_INVALID_STATE}, ErrorCode::SetAutoSelfCalibration});
    if (auto res = set_automatic_self_calibration_enabled{m_Device}.Send(v); !res)
        return std::unexpected(Err{res.error(), ErrorCode::SetAutoSelfCalibration});
    return std::ref(*this);
}

SCD40::ExpectedValue<SCD40::AutoSelfCalibration> SCD40::get_auto_self_calibration()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::get_auto_self_calibration", .code = ESP_ERR_INVALID_STATE}, ErrorCode::GetAutoSelfCalibration});
    if (auto res = get_automatic_self_calibration_enabled{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::GetAutoSelfCalibration});
    else 
        return *res;
}

SCD40::ExpectedResult SCD40::set_auto_self_calibration_baseline(uint16_t co2)
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::set_auto_self_calibration_baseline", .code = ESP_ERR_INVALID_STATE}, ErrorCode::SetAutoSelfCalibrationBaseline});
    if (auto res = set_automatic_self_calibration_target{m_Device}.Send(co2); !res)
        return std::unexpected(Err{res.error(), ErrorCode::SetAutoSelfCalibrationBaseline});
    return std::ref(*this);
}
SCD40::ExpectedValue<uint16_t> SCD40::get_auto_self_calibration_baseline()
{
    if (m_Measuring)//wrong state
        return std::unexpected(Err{::Err{.pLocation = "SCD40::get_auto_self_calibration_baseline", .code = ESP_ERR_INVALID_STATE}, ErrorCode::GetAutoSelfCalibrationBaseline});
    if (auto res = get_automatic_self_calibration_target{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::GetAutoSelfCalibrationBaseline});
    else 
        return *res;
}

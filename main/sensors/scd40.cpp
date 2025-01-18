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
    }
}

SCD40::ExpectedValue<SCD40> SCD40::Open(i2c::I2CBusMaster &bus)
{
    auto r = bus.Add(kAddress);
    if (!r)
        return std::unexpected(Err{r.error(), ErrorCode::Open});
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return SCD40(std::move(*r));
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
    if (auto res = get_sensor_variant{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::GetSensorType});
    else
        return SCD40::SensorType{res->type};
}

SCD40::ExpectedValue<SCD40::serial_t> SCD40::get_serial_id()
{
    if (auto res = get_serial_number{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::GetSerialId});
    else
        return *res;
}

SCD40::ExpectedResult SCD40::start()
{
    if (auto res = start_periodic_measurement{m_Device}.Send(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::Start});
    return std::ref(*this);
}

SCD40::ExpectedResult SCD40::start_low_power()
{
    if (auto res = start_low_power_periodic_measurement{m_Device}.Send(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::StartLowPower});
    return std::ref(*this);
}

SCD40::ExpectedResult SCD40::stop()
{
    if (auto res = stop_periodic_measurement{m_Device}.Send(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::Stop});

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return std::ref(*this);
}

SCD40::ExpectedValue<bool> SCD40::is_data_ready()
{
    if (auto res = get_data_ready_status{m_Device}.Recv(); !res)
        return std::unexpected(Err{res.error(), ErrorCode::IsDataReady});
    else
        return res->status != 0;
}

SCD40::ExpectedResult SCD40::wait_until_data_ready(i2c::duration_t d)
{
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
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return std::unexpected(Err{::Err{.pLocation="SCD40::wait_until_data_ready", .code = ESP_ERR_TIMEOUT}, ErrorCode::WaitDataReady});
}

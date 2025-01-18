#include "scd40.hpp"

SCD40::ExpectedValue<SCD40> SCD40::Open(i2c::I2CBusMaster &bus)
{
    auto r = bus.Add(kAddress);
    if (!r)
        return std::unexpected(Err{r.error(), ErrorCode::Open});
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

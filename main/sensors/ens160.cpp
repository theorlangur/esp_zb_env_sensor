#include "ens160.hpp"
#include "freertos/idf_additions.h"

#define TRY_READ_REG_AND_RETURN(RegType, err_code) \
    if (auto r = RegType{m_Device}.Read(); !r) \
        return std::unexpected(Err{r.error(), err_code}); \
    else \
        return *r;

#define TRY_READ_REG(res, RegType, err_code) \
    if (auto r = RegType{m_Device}.Read(); !r) \
        return std::unexpected(Err{r.error(), err_code}); \
    else \
        res = *r;

#define TRY_WRITE_REG(RegType, res, err_code) \
    if (auto r = RegType{m_Device}.Write(res); !r) \
        return std::unexpected(Err{r.error(), err_code});

const char* ENS160::err_to_str(ErrorCode e)
{
    switch(e)
    {
        case ErrorCode::Ok: return "Ok";
        case ErrorCode::GetVersion: return "GetVersion";
        case ErrorCode::GetStatus: return "GetStatus";
        case ErrorCode::OneShot_GetMode: return "OneShot_GetMode";
        case ErrorCode::OneShot_SetMode: return "OneShot_SetMode";
        case ErrorCode::OneShot_Status: return "OneShot_Status";
        case ErrorCode::OneShot_Idle: return "OneShot_Idle";
        case ErrorCode::GoToSensing: return "GoToSensing";
        case ErrorCode::GoToIdle: return "GoToIdle";
        case ErrorCode::GoToDeepSleep: return "GoToDeepSleep";
        case ErrorCode::GetOpMode: return "GetOpMode";
        case ErrorCode::ReadTemp: return "ReadTemp";
        case ErrorCode::ReadRelHumid: return "ReadRelHumid";
        case ErrorCode::ReadAQI: return "ReadAQI";
        case ErrorCode::ReadTVOC: return "ReadTVOC";
        case ErrorCode::ReadeCO2: return "ReadeCO2";
    }
}

float ENS160::InternalTempToC(uint16_t v)
{
    return float(v) / 64 - 273.15f;
}

uint16_t ENS160::CToInternalTemp(float v)
{
    return uint16_t((v + 273.15f) * 64);
}

float ENS160::InternalRHToRH(uint16_t v)
{
    return float(v) / 512;
}

uint16_t ENS160::RHToInternalRH(float v)
{
    return uint16_t(v * 512);
}

ENS160::ENS160(i2c::I2CBusMaster &bus, uint8_t addr):
    m_Device(*bus.Add(addr))
{
}

ENS160::ExpectedValue<ENS160::Version> ENS160::GetVersion() const
{
    TRY_READ_REG_AND_RETURN(VerReg, ErrorCode::GetVersion);
}

ENS160::ExpectedValue<ENS160::Status> ENS160::GetStatus() const
{
    TRY_READ_REG_AND_RETURN(StatusReg, ErrorCode::GetStatus);
}

ENS160::ExpectedResult ENS160::DoOneShotMeasurements()
{
    OpMode m;
    TRY_READ_REG(m, OpModeReg, ErrorCode::OneShot_GetMode);
    if (m != OpMode::Standard)
    {
        TRY_WRITE_REG(OpModeReg, OpMode::Standard, ErrorCode::OneShot_SetMode);
        Status s;
        TRY_READ_REG(s, StatusReg, ErrorCode::OneShot_Status);
        while(!s.new_data)
        {
            vTaskDelay(2);
            TRY_READ_REG(s, StatusReg, ErrorCode::OneShot_Status);
        }
    }
    TRY_WRITE_REG(OpModeReg, OpMode::Idle, ErrorCode::OneShot_Idle);
    return std::ref(*this);
}

ENS160::ExpectedResult ENS160::GoToSensing()
{
    TRY_WRITE_REG(OpModeReg, OpMode::Standard, ErrorCode::GoToSensing);
    return std::ref(*this);
}

ENS160::ExpectedResult ENS160::GoToIdle()
{
    TRY_WRITE_REG(OpModeReg, OpMode::Idle, ErrorCode::GoToIdle);
    return std::ref(*this);
}

ENS160::ExpectedResult ENS160::GoToDeepSleep()
{
    TRY_WRITE_REG(OpModeReg, OpMode::DeepSleep, ErrorCode::GoToDeepSleep);
    return std::ref(*this);
}

ENS160::ExpectedValue<ENS160::OpMode> ENS160::GetOpMode() const
{
    TRY_READ_REG_AND_RETURN(OpModeReg, ErrorCode::GetOpMode);
}

ENS160::ExpectedValue<float> ENS160::ReadTemperature() const
{
    uint16_t t;
    TRY_READ_REG(t, DataTempReg, ErrorCode::ReadTemp);
    return InternalTempToC(t);
}

ENS160::ExpectedValue<float> ENS160::ReadRelativeHumidity() const
{
    uint16_t t;
    TRY_READ_REG(t, DataRHReg, ErrorCode::ReadRelHumid);
    return InternalRHToRH(t);
}

ENS160::ExpectedValue<uint8_t> ENS160::ReadAirQualityIndex() const
{
    TRY_READ_REG_AND_RETURN(AqiReg, ErrorCode::ReadAQI);
}

ENS160::ExpectedValue<uint16_t> ENS160::ReadTVOC() const
{
    TRY_READ_REG_AND_RETURN(TVOCReg, ErrorCode::ReadTVOC);
}

ENS160::ExpectedValue<uint16_t> ENS160::ReadeCO2() const
{
    TRY_READ_REG_AND_RETURN(eCO2Reg, ErrorCode::ReadeCO2);
}

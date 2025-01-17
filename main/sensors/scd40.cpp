#include "scd40.hpp"

SCD40::ExpectedValue<SCD40> SCD40::Open(i2c::I2CBusMaster &bus)
{
    auto r = bus.Add(kAddress);
    if (!r)
        return std::unexpected(Err{r.error(), ErrorCode::Open});
    return SCD40(std::move(*r));
}

SCD40::SCD40(i2c::I2CDevice &&d): m_Device(std::move(d))
{
}

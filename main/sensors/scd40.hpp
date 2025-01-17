#ifndef SCD40_HPP_
#define SCD40_HPP_

#include "ph_i2c.hpp"

class SCD40
{
public:
    static constexpr const uint8_t kAddress = 0x62;
    
    enum class ErrorCode: uint8_t
    {
        Ok,
        Open
    };

    using Ref = std::reference_wrapper<SCD40>;
    struct Err
    {
        ::Err i2cErr;
        ErrorCode code;
    };
    using ExpectedResult = std::expected<Ref, Err>;

    template<class V>
    using ExpectedValue = std::expected<V, Err>;

    static ExpectedValue<SCD40> Open(i2c::I2CBusMaster &bus);

    SCD40(SCD40 &&d) = default;
    SCD40(const SCD40 &d) = delete;
private:
    SCD40(i2c::I2CDevice &&d);
    struct word_t
    {
        uint8_t msb;
        uint8_t lsb;
        uint8_t crc;
    };
    static uint8_t calc_crc(uint16_t data);
    static bool check_crc(word_t &data);

    enum class Regs: uint16_t
    {
        start_periodic_measurement = 0x21b1,
        read_measurement = 0xec05,
        stop_periodic_measurement = 0x3f86,

        set_temperature_offset = 0x241d,
        get_temperature_offset = 0x2318,
        set_sensor_altitude = 0x2427,
        get_sensor_altitude = 0x2322,
        set_ambient_pressure = 0xe000,
        get_ambient_pressure = 0xe000,

        perform_forced_recalibration = 0x362f,
        set_automatic_self_calibration_enabled = 0x2416,
        get_automatic_self_calibration_enabled = 0x2313,
        set_automatic_self_calibration_target = 0x243a,
        get_automatic_self_calibration_target = 0x233f,

        start_low_power_periodic_measurement = 0x21ac,
        get_data_ready_status = 0xe4b8,

        persist_settings = 0x3615,
        get_serial_number = 0x3682,
        perform_self_test = 0x3639,
        perform_factory_reset = 0x3632,
        reinit = 0x3646,
        get_sensor_variant = 0x202f,

        //SCD41
        measure_single_shot = 0x219d,
        measure_single_shot_rht_only = 0x2196,
        power_down = 0x36e0,
        wake_up = 0x36f6,
        set_automatic_self_calibration_initial_period = 0x2445,
        get_automatic_self_calibration_initial_period = 0x2340,
        set_automatic_self_calibration_standard_period = 0x244e,
        get_automatic_self_calibration_standard_period = 0x234b,
    };

    struct RegisterBase
    {
        i2c::I2CDevice &d;
    };

    template<Regs r>
    struct RegCmd: RegisterBase
    {
        i2c::I2CDevice::ExpectedResult Send()
        {
            uint8_t data[] = {uint8_t(uint16_t(r) >> 8), uint8_t(uint16_t(r) & 0x00ff)};
            return d.Send(data, sizeof(data), i2c::helpers::kTimeout);
        }
    };

    template<Regs r, class Val> requires (sizeof(Val) == sizeof(uint16_t))
    struct RegWrite: RegisterBase
    {
        i2c::I2CDevice::ExpectedResult Send(Val const& v)
        {
            uint8_t data[] = 
            {     uint8_t(uint16_t(r) >> 8)                        //reg MSB
                , uint8_t(uint16_t(r) & 0x00ff)                    //reg LSB
                , uint8_t(reinterpret_cast<uint16_t&>(v) >> 8)     //data MSB
                , uint8_t(reinterpret_cast<uint16_t&>(v) & 0x00ff) //data LSB
                , calc_crc(reinterpret_cast<uint16_t&>(v))         //data CRC
            };
            return d.Send(data, sizeof(data), i2c::helpers::kTimeout);
        }
    };

    template<Regs r, class Val> requires (sizeof(Val) % sizeof(uint16_t) == 0)
    struct RegRead: RegisterBase
    {
        std::expected<Val, ::Err> Recv()
        {
            uint8_t reg_data[] = 
            {     uint8_t(uint16_t(r) >> 8)                        //reg MSB
                , uint8_t(uint16_t(r) & 0x00ff)                    //reg LSB
            };
            word_t data[sizeof(Val) / sizeof(uint16_t)];
            if (auto ret = d.SendRecv(reg_data, sizeof(reg_data), data, sizeof(data), i2c::helpers::kTimeout); !ret)
                return std::unexpected(ret.error());

            Val res;
            uint16_t *pDst = (uint16_t *)&res;
            for(word_t &w : data)
            {
                if (!check_crc(w))
                    return std::unexpected(::Err{"RegRead", ESP_ERR_INVALID_CRC});
                *pDst = w.msb << 8 | w.lsb;
                ++pDst;
            }
            return res;
        }
    };

    template<Regs r, class SendType, class RecvType> requires ((sizeof(SendType) % sizeof(uint16_t) == 0) && (sizeof(RecvType) % sizeof(uint16_t) == 0))
    struct RegReadWrite: RegisterBase
    {
        std::expected<RecvType, ::Err> Transmit(SendType s)
        {
            uint8_t reg_data[2 + 3 * (sizeof(SendType) / sizeof(uint16_t))] = 
            {     uint8_t(uint16_t(r) >> 8)                        //reg MSB
                , uint8_t(uint16_t(r) & 0x00ff)                    //reg LSB
                , uint8_t()
                , uint8_t(reinterpret_cast<uint16_t&>(s))
                , calc_crc(reinterpret_cast<uint16_t&>(s))
            };
            uint8_t *pRegDst = reg_data + 2;
            for(uint16_t *pSrc = (uint16_t *)&s; pSrc != (uint16_t *)(&s + 1); ++pSrc, pRegDst += 3)
            {
                pRegDst[0] = *pSrc >> 8;
                pRegDst[1] = *pSrc;
                pRegDst[2] = calc_crc(*pSrc);
            }
            word_t data[sizeof(RecvType) / sizeof(uint16_t)];
            if (auto ret = d.SendRecv(reg_data, sizeof(reg_data), data, sizeof(data), i2c::helpers::kTimeout); !ret)
                return std::unexpected(ret.error());

            RecvType res;
            uint16_t *pDst = (uint16_t *)&res;
            for(word_t &w : data)
            {
                if (!check_crc(w))
                    return std::unexpected(::Err{"RegReadWrite", ESP_ERR_INVALID_CRC});
                *pDst = w.msb << 8 | w.lsb;
                ++pDst;
            }
            return res;
        }
    };

    using start_periodic_measurement = RegCmd<Regs::start_periodic_measurement>;
    struct measurement_t
    {
        uint16_t co2;
        uint16_t t;
        uint16_t rh;

        float get_temp() const { return (float(t) / float(0xffff)) * 175 - 45; }
        float get_humidity() const { return (float(t) / float(0xffff)) * 100; }
    };
    using read_measurement = RegRead<Regs::read_measurement, measurement_t>;
    using stop_periodic_measurement = RegCmd<Regs::stop_periodic_measurement>;
    
    struct t_offset_t
    {
        uint16_t t;
        void set_temp_offset(float _t) { t = _t * 0xffff / 175; }
        float get_temp_offset() const { return float(t * 175) / float(0xffff); }
    };
    using set_temperature_offset = RegWrite<Regs::set_temperature_offset, t_offset_t>;
    using get_temperature_offset = RegRead<Regs::get_temperature_offset, t_offset_t>;

    using set_sensor_altitude = RegWrite<Regs::set_sensor_altitude, uint16_t>;
    using get_sensor_altitude = RegRead<Regs::get_sensor_altitude, uint16_t>;

    struct ambient_pressure_t
    {
        uint16_t t;
        void set(float _t) { t = _t / 100; }
        float get() const { return float(t) * 100; }
    };
    using set_ambient_pressure = RegWrite<Regs::set_ambient_pressure, ambient_pressure_t>;
    using get_ambient_pressure = RegRead<Regs::get_ambient_pressure, ambient_pressure_t>;

    struct frc_t
    {
        uint16_t v;

        bool valid() const { return v == 0xffff; }
        uint16_t get() const { return v - 0x8000; }
    };
    using perform_forced_recalibration = RegReadWrite<Regs::perform_forced_recalibration, uint16_t, frc_t>;
    enum AutoSelfCalibration: uint16_t
    {
        Disabled = 0,
        Enabled = 1,
    };
    using set_automatic_self_calibration_enabled = RegWrite<Regs::set_automatic_self_calibration_enabled, AutoSelfCalibration>;
    using get_automatic_self_calibration_enabled = RegRead<Regs::get_automatic_self_calibration_enabled, AutoSelfCalibration>;

    using set_automatic_self_calibration_target = RegWrite<Regs::set_automatic_self_calibration_target, uint16_t>;
    using get_automatic_self_calibration_target = RegRead<Regs::get_automatic_self_calibration_target, uint16_t>;

    using start_low_power_periodic_measurement = RegCmd<Regs::start_low_power_periodic_measurement>;
    struct data_ready_t
    {
        uint16_t status: 11;
        uint16_t unused: 5;
    };
    using get_data_ready_status = RegWrite<Regs::get_data_ready_status, data_ready_t>;
    using save_settings = RegCmd<Regs::persist_settings>;
    struct serial_t
    {
        uint16_t w[3];
    };
    using get_serial_number = RegRead<Regs::get_serial_number, serial_t>;
    using perform_self_test = RegRead<Regs::perform_self_test, uint16_t>;
    using perform_factory_reset = RegCmd<Regs::perform_factory_reset>;
    using reinit = RegCmd<Regs::reinit>;

    enum class SensorType: uint16_t
    {
        SCD40 = 0,
        SCD41 = 1
    };
    struct sensor_variant_t
    {
        uint16_t unused: 12;
        SensorType type: 4;
    };
    using get_sensor_variant = RegRead<Regs::get_sensor_variant, sensor_variant_t>;

    i2c::I2CDevice m_Device;
};

#endif

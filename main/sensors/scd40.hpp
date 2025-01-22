#ifndef SCD40_HPP_
#define SCD40_HPP_

#include "ph_i2c.hpp"
#include <thread>

template<class C>
concept word_conformant_c =  requires { requires (sizeof(C) % sizeof(uint16_t) == 0); };

class SCD40
{
public:
    static constexpr const uint8_t kAddress = 0x62;
    
    enum class ErrorCode: uint8_t
    {
        Ok
        , Open
        , ReadMeasurements
        , GetSensorType
        , GetSerialId
        , Start
        , StartLowPower
        , Stop
        , IsDataReady
        , WaitDataReady
        , SetTempOffset
        , GetTempOffset
        , SetAltitude
        , GetAltitude
        , SetPressure
        , GetPressure
        , StoreSettings
        , FactoryReset
        , ReInit
        , Recalibrate
        , SelfTest
        , SetAutoSelfCalibration
        , GetAutoSelfCalibration
        , SetAutoSelfCalibrationBaseline
        , GetAutoSelfCalibrationBaseline
    };
    static const char* err_to_str(ErrorCode e);

    enum class SensorType: uint16_t
    {
        SCD40 = 0,
        SCD41 = 1
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

    struct measurement_results_t
    {
        uint16_t co2;
        float temp;
        float rh;
    };

    struct serial_t
    {
        uint16_t w[3];
    };

    enum AutoSelfCalibration: uint16_t
    {
        Disabled = 0,
        Enabled = 1,
    };

    static ExpectedValue<SCD40> Open(i2c::I2CBusMaster &bus);

    SCD40(SCD40 &&d) = default;
    SCD40(const SCD40 &d) = delete;

    ExpectedResult start();
    ExpectedResult start_low_power();
    ExpectedResult stop();
    ExpectedValue<measurement_results_t> read_measurements();
    ExpectedValue<SensorType> get_sensor_type();
    ExpectedValue<serial_t> get_serial_id();
    ExpectedValue<bool> is_data_ready();
    ExpectedResult wait_until_data_ready(i2c::duration_t d = kForever);

    ExpectedResult set_temp_offset(float off);
    ExpectedValue<float> get_temp_offset();

    ExpectedResult set_altitude(uint16_t a);
    ExpectedValue<uint16_t> get_altitude();

    ExpectedResult set_pressure(float p);
    ExpectedValue<float> get_pressure();

    ExpectedResult store_settings();
    ExpectedResult factory_reset();
    ExpectedResult re_init();
    //returns true if all is ok, false - otherwise
    ExpectedValue<bool> self_test();

    ExpectedValue<uint16_t> recalibrate(uint16_t new_reference_co2);

    ExpectedResult set_auto_self_calibration(AutoSelfCalibration v);
    ExpectedValue<AutoSelfCalibration> get_auto_self_calibration();

    ExpectedResult set_auto_self_calibration_baseline(uint16_t co2);
    ExpectedValue<uint16_t> get_auto_self_calibration_baseline();
private:
    SCD40(i2c::I2CDevice &&d);
    struct word_t
    {
        word_t() = default;
        word_t(uint16_t d);

        uint8_t msb;
        uint8_t lsb;
        uint8_t crc;
    };
    static uint8_t calc_crc(const word_t &w);
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

    template<size_t N>
    struct cmd_data_t
    {
        cmd_data_t(Regs cmd): cmd_msb(uint16_t(cmd) >> 8), cmd_lsb(uint16_t(cmd) & 0x0ff){}
        template<class V> requires word_conformant_c<V> && (sizeof(V) == sizeof(uint16_t) * N)
        cmd_data_t(Regs cmd, V const& v): cmd_data_t(cmd)
        {
            const uint16_t *pSrc = reinterpret_cast<const uint16_t*>(&v);
            for(word_t &w : data)
            {
                w.msb = *pSrc >> 8;
                w.lsb = *pSrc & 0x0ff;
                w.crc = calc_crc(w);
            }
        }

        operator uint8_t*() { return &cmd_msb; }
        operator const uint8_t*() const { return &cmd_msb; }

        uint8_t cmd_msb;
        uint8_t cmd_lsb;
        [[no_unique_address]] word_t data[N];
    };
    template<class V>
    cmd_data_t(Regs cmd, V const& v) -> cmd_data_t<sizeof(V) / sizeof(uint16_t)>;

    template<class V>
    struct recv_data_t
    {
        operator uint8_t *() { return (uint8_t*)this; }

        bool read(V &dst)
        {
            uint16_t *pDst = (uint16_t *)&dst;
            for(word_t &w : data)
            {
                if (!check_crc(w))
                    return false;
                *pDst = w.msb << 8 | w.lsb;
                ++pDst;
            }
            return true;
        }
        word_t data[sizeof(V) / sizeof(uint16_t)];
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
            cmd_data_t<0> data{r};
            return d.Send(data, sizeof(data), i2c::helpers::kTimeout);
        }
    };

    template<Regs r, class Val> requires word_conformant_c<Val>
    struct RegWrite: RegisterBase
    {
        i2c::I2CDevice::ExpectedResult Send(Val const& v)
        {
            cmd_data_t data{r, v};
            return d.Send(data, sizeof(data), i2c::helpers::kTimeout);
        }
    };

    template<Regs r, class Val, size_t durMS = 0> requires word_conformant_c<Val>
    struct RegRead: RegisterBase
    {
        std::expected<Val, ::Err> Recv()
        {
            cmd_data_t<0> reg_data{r};
            recv_data_t<Val> data;
            if (auto ret = d.SendRecv(reg_data, sizeof(reg_data), data, sizeof(data), i2c::helpers::kTimeout); !ret)
                return std::unexpected(ret.error());

            Val res;
            if (!data.read(res))
                return std::unexpected(::Err{"RegRead", ESP_ERR_INVALID_CRC});
            return res;
        }
    };

    template<Regs r, class SendType, class RecvType> requires word_conformant_c<SendType> && word_conformant_c<RecvType>
    struct RegReadWrite: RegisterBase
    {
        std::expected<RecvType, ::Err> Transmit(SendType s)
        {
            cmd_data_t reg_data{r, s};
            recv_data_t<RecvType> data;
            if (auto ret = d.SendRecv(reg_data, sizeof(reg_data), data, sizeof(data), i2c::helpers::kTimeout); !ret)
                return std::unexpected(ret.error());

            RecvType res;
            if (!data.read(res))
                return std::unexpected(::Err{"RegReadWrite", ESP_ERR_INVALID_CRC});
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
    using read_measurement = RegRead<Regs::read_measurement, measurement_t, 1>;
    using stop_periodic_measurement = RegCmd<Regs::stop_periodic_measurement>;
    
    struct t_offset_t
    {
        uint16_t t;
        t_offset_t() = default;
        t_offset_t(uint16_t v):t(v){}
        t_offset_t(float v){ set_temp_offset(v); }

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
        ambient_pressure_t() = default;
        ambient_pressure_t(float v){set(v);}
        ambient_pressure_t(uint16_t v):t(v){}

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
    using get_data_ready_status = RegRead<Regs::get_data_ready_status, data_ready_t, 1>;
    using save_settings = RegCmd<Regs::persist_settings>;
    using get_serial_number = RegRead<Regs::get_serial_number, serial_t, 1>;
    using perform_self_test = RegRead<Regs::perform_self_test, uint16_t>;
    using perform_factory_reset = RegCmd<Regs::perform_factory_reset>;
    using reinit = RegCmd<Regs::reinit>;

    struct sensor_variant_t
    {
        uint16_t unused: 12;
        SensorType type: 4;
    };
    using get_sensor_variant = RegRead<Regs::get_sensor_variant, sensor_variant_t, 1>;

    i2c::I2CDevice m_Device;
    bool m_Measuring = false;
};

template<>
struct tools::formatter_t<SCD40::Err>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, SCD40::Err const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "{} at {}" , v.i2cErr, SCD40::err_to_str(v.code));
    }
};

#endif

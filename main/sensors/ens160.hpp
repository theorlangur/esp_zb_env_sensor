#ifndef ENS160_HPP_
#define ENS160_HPP_

#include "ph_i2c.hpp"

class ENS160
{
public:
    static constexpr const uint8_t kAddressHigh = 0x53;//default
    static constexpr const uint8_t kAddressLow = 0x52;

    enum class ErrorCode: uint8_t
    {
        Ok,
        GetVersion,
        GetStatus,
        OneShot_GetMode,
        OneShot_SetMode,
        OneShot_Status,
        OneShot_Idle,
        GoToSensing,
        GoToIdle,
        GoToDeepSleep,
        GetOpMode,
        ReadTemp,
        ReadRelHumid,
        ReadAQI,
        ReadTVOC,
        ReadeCO2,
    };
    static const char* err_to_str(ErrorCode e);

    using Ref = std::reference_wrapper<ENS160>;
    struct Err
    {
        ::Err i2cErr;
        ErrorCode code;
    };
    using ExpectedResult = std::expected<Ref, Err>;

    template<class V>
    using ExpectedValue = std::expected<V, Err>;

    struct Version
    {
        uint8_t maj;
        uint8_t min;
        uint8_t rel;
    };
    static_assert(sizeof(Version) == 3);

    struct Config
    {
        enum class IntCfg: uint8_t { OpenDrain = 0, PushPull = 1 };
        enum class IntPol: uint8_t { ActiveLow = 0, ActiveHigh = 1 };
        uint8_t int_en   : 1 = 0;
        uint8_t int_dat  : 1 = 0;
        uint8_t reserved1: 1 = 0;
        uint8_t int_gpr  : 1 = 0;
        uint8_t reserved2: 1 = 0;
        IntCfg  int_cfg  : 1 = IntCfg::OpenDrain;
        IntPol  int_pol  : 1 = IntPol::ActiveLow;
        uint8_t reserved3: 1 = 0;
    };
    static_assert(sizeof(Config) == sizeof(uint8_t));

    struct Status
    {
        enum class Validity: uint8_t { NormalOp = 0, WarmUp = 1, InitialStartUp = 2, InvalidOutput = 3 };
        uint8_t new_gpr  : 1;
        uint8_t new_data : 1;
        Validity validity: 2;
        uint8_t reserved : 2;
        uint8_t error    : 1;
        uint8_t state    : 1;
    };
    static_assert(sizeof(Status) == sizeof(uint8_t));

    enum OpMode: uint8_t
    {
        DeepSleep = 0,
        Idle = 1,
        Standard = 2,
        Reset = 0xf0
    };

    ENS160(i2c::I2CBusMaster &bus, uint8_t addr = kAddressHigh);

    ExpectedValue<Version> GetVersion() const;
    ExpectedValue<Status> GetStatus() const;

    ExpectedResult DoOneShotMeasurements();

    ExpectedResult GoToSensing();
    ExpectedResult GoToIdle();
    ExpectedResult GoToDeepSleep();

    ExpectedValue<OpMode> GetOpMode() const;

    ExpectedValue<float> ReadTemperature() const;
    ExpectedValue<float> ReadRelativeHumidity() const;
    ExpectedValue<uint8_t> ReadAirQualityIndex() const;
    ExpectedValue<uint16_t> ReadTVOC() const;
    ExpectedValue<uint16_t> ReadeCO2() const;
private:
    enum class Regs: uint8_t
    {
        PART_ID = 0x00,
        OPMODE = 0x10,
        CONFIG = 0x11,
        COMMAND = 0x12,
        TEMP_IN = 0x13,
        RH_IN = 0x15,
        DEVICE_STATUS = 0x20,
        DATA_AQI = 0x21,
        DATA_TVOC = 0x22,
        DATA_ECO2 = 0x24,
        DATA_T = 0x30,
        DATA_RH = 0x32,
        DATA_MISR = 0x38,
        GPR_WRITE0 = 0x40,
        GPR_WRITE1 = 0x41,
        GPR_WRITE2 = 0x42,
        GPR_WRITE3 = 0x43,
        GPR_WRITE4 = 0x44,
        GPR_WRITE5 = 0x45,
        GPR_WRITE6 = 0x46,
        GPR_WRITE7 = 0x47,
        GPR_READ0 = 0x48,
        GPR_READ1 = 0x49,
        GPR_READ2 = 0x4a,
        GPR_READ3 = 0x4b,
        GPR_READ4 = 0x4c,
        GPR_READ5 = 0x4d,
        GPR_READ6 = 0x4e,
        GPR_READ7 = 0x4f,
    };

    enum Command: uint8_t
    {
        NOP = 0,
        GetAppVer = 0x0E,
        ClearGPRRegs = 0xCC
    };

    using PartIdReg  = i2c::helpers::Register<uint16_t, Regs::PART_ID, i2c::helpers::RegAccess::Read>;
    using OpModeReg  = i2c::helpers::Register<OpMode, Regs::OPMODE, i2c::helpers::RegAccess::RW>;
    using ConfigReg  = i2c::helpers::Register<Config, Regs::CONFIG, i2c::helpers::RegAccess::RW>;
    using CommandReg = i2c::helpers::Register<Command, Regs::COMMAND, i2c::helpers::RegAccess::RW>;
    using TempInReg  = i2c::helpers::Register<uint16_t, Regs::TEMP_IN, i2c::helpers::RegAccess::RW>;
    using RelHumReg  = i2c::helpers::Register<uint16_t, Regs::RH_IN, i2c::helpers::RegAccess::RW>;
    using VerReg     = i2c::helpers::Register<Version, Regs::GPR_READ4, i2c::helpers::RegAccess::Read>;
    using StatusReg  = i2c::helpers::Register<Status, Regs::DEVICE_STATUS, i2c::helpers::RegAccess::Read>;
    using AqiReg     = i2c::helpers::Register<uint8_t, Regs::DATA_AQI, i2c::helpers::RegAccess::Read>;
    using TVOCReg    = i2c::helpers::Register<uint16_t, Regs::DATA_TVOC, i2c::helpers::RegAccess::Read>;
    using eCO2Reg    = i2c::helpers::Register<uint16_t, Regs::DATA_ECO2, i2c::helpers::RegAccess::Read>;
    using DataTempReg= i2c::helpers::Register<uint16_t, Regs::DATA_T, i2c::helpers::RegAccess::Read>;
    using DataRHReg  = i2c::helpers::Register<uint16_t, Regs::DATA_RH, i2c::helpers::RegAccess::Read>;

    static float InternalTempToC(uint16_t v);
    static uint16_t CToInternalTemp(float v);
    static float InternalRHToRH(uint16_t v);
    static uint16_t RHToInternalRH(float v);

    mutable i2c::I2CDevice m_Device;
};

#endif

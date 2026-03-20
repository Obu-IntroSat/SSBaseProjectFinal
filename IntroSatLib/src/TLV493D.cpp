#include <TLV493D.h>
#include <cstring>

#define TEMP_MULT         1.1f
#define TEMP_OFFSET       340
#define I2C_TIMEOUT_MS    100U
#define STARTUP_DELAY_MS  50U
#define MAX_INIT_RETRIES  5



namespace IntroSatLib {

struct ModeConfig {
    uint8_t fast;
    uint8_t lp;
    uint8_t lpPeriod;  // LP bit in MOD2: 0=100ms period, 1=12ms period
};

static const ModeConfig MODE_CONFIGS[] = {
    {0, 0, 0},  // POWERDOWN:        FAST=0 LOW=0
    {1, 0, 0},  // FASTMODE:         FAST=1 LOW=0
    {0, 1, 1},  // LOWPOWER:         FAST=0 LOW=1, LP=1 (12ms)
    {0, 1, 0},  // ULTRALOWPOWER:    FAST=0 LOW=1, LP=0 (100ms)
    {1, 1, 0},  // MASTERCONTROLLED: FAST=1 LOW=1
};

static const uint32_t MODE_DELAYS_MS[] = { 1000, 1, 15, 110, 1 };


TLV493D::TLV493D(I2C_HandleTypeDef *hi2c)
    : BaseDevice(hi2c, BASE_ADDRESS),
	  _i2cPowerDown(* new I2CDevice(hi2c, 0xFF)),
	  _i2cReset(* new I2CDevice(hi2c, 0x00))
{ }


TLV493D_Status TLV493D::init(TLV493D_Mode mode)
{
    HAL_Delay(STARTUP_DELAY_MS);

    int errorCount = 0;

    while (errorCount < MAX_INIT_RETRIES)
    {
        // Step 1: Recovery frame — S 0xFF P
        // Frees the bus in case of aborted communication
        sendRecovery();
        HAL_Delay(2);

        // Step 2: Reset frame — S 0x00 0xFF P
        // After stop condition SDA is pulled HIGH by pull-up resistor
        // Sensor reads SDA = HIGH for t1 > 14us after stop -> sets ADDR=1 -> address 0x5E
        sendReset();

        // Step 3: Wait for sensor to latch address
        // t1 > 14us (Figure 15), we wait 1ms to be safe
        HAL_Delay(1);

        // Step 4: Read sensorbitmap registers 0x00..0x09
        if (!readRegs())
        {
            errorCount++;
            // Error recovery: wait and retry
            HAL_Delay(20);
            continue;
        }

        // Step 5: Store factory bits from read registers into write registers
        // regWrite[0] = 0x00 (reserved, must stay 0)
        _regWrite[0] = 0x00;

        // regWrite[1] = MOD1
        // bits[4:3] must come from regRead[7] bits[4:3] (factory settings)
        _regWrite[1] = (_regRead[7] & 0x18);

        // regWrite[2] = MOD2 / Reserved
        // all bits come from regRead[8] (factory settings)
        _regWrite[2] = _regRead[8];

        // regWrite[3] = Reserved
        // bits[4:0] come from regRead[9] bits[4:0] (factory settings)
        _regWrite[3] = (_regRead[9] & 0x1F);

        // Step 6: Enable parity test (PT bit = bit5 in MOD2/regWrite[2])
        _regWrite[2] |= (1 << 5);

        // Step 7: Configure sensor mode -> Exit: Sensor OK
        if (setMode(mode) == TLV493D_Status::OK)
        {
            return TLV493D_Status::OK;
        }

        errorCount++;
        HAL_Delay(20);
    }

    return TLV493D_Status::ERR_I2C;
}


TLV493D_Status TLV493D::setMode(TLV493D_Mode mode)
{
    applyModeConfig(mode);
    calcParity();

    if (!writeRegs()) {
        return TLV493D_Status::ERR_I2C;
    }

    _mode = mode;
    return TLV493D_Status::OK;
}


TLV493D_Status TLV493D::read()
{
    TLV493D_Status ret = TLV493D_Status::OK;
    bool wasPowerdown = (_mode == TLV493D_Mode::POWERDOWN);

    if (wasPowerdown) {
        if (setMode(TLV493D_Mode::MASTERCONTROLLED) != TLV493D_Status::OK) {
            return TLV493D_Status::ERR_I2C;
        }
        HAL_Delay(getMeasurementDelay_ms());
    }

    uint8_t buf[7] = {0};
    if (_i2c.read(buf, 7) != HAL_OK) {
        return TLV493D_Status::ERR_I2C;
    }

    // buf[0] = Bx[11:4]
    // buf[1] = By[11:4]
    // buf[2] = Bz[11:4]
    // buf[3] = Temp[11:8] | FRM | CH
    // buf[4] = Bx[3:0] | By[3:0]
    // buf[5] = Res | T | FF | PD | Bz[3:0]
    // buf[6] = Temp[7:0]

    _rawX    = concat12bit(buf[0], buf[4] >> 4, true);
    _rawY    = concat12bit(buf[1], buf[4], true);
    _rawZ    = concat12bit(buf[2], buf[5], true);
    _rawTemp = concat12bit(buf[3] >> 4, buf[6], false);

    // CH must be 0x00 and PD must be 1 to confirm conversion complete
    uint8_t ch = buf[3] & 0x03;

    if (ch != 0) {
        ret = TLV493D_Status::ERR_FRAME;
    }

    if (wasPowerdown) {
        setMode(TLV493D_Mode::POWERDOWN);
    }

    return ret;
}


float TLV493D::X()   const { return static_cast<float>(_rawX) * 0.98f; }
float TLV493D::Y()   const { return static_cast<float>(_rawY) * 0.98f; }
float TLV493D::Z()   const { return static_cast<float>(_rawZ) * 0.98f; }
float TLV493D::Temp() const { return static_cast<float>(_rawTemp - TEMP_OFFSET) * TEMP_MULT + 25.0f; }

uint32_t TLV493D::getMeasurementDelay_ms() const
{
    return MODE_DELAYS_MS[static_cast<uint8_t>(_mode)];
}


// Recovery frame: S 0xFF P
// Sent to address 0x7F (broadcast) to free the bus
void TLV493D::sendRecovery()
{
    uint8_t data = 0xFF;
    // 0xFF as address byte with read bit = general call to release stuck SDA
    _i2cPowerDown.write(&data, 0);
}


// Reset frame: S 0x00 0xFF P  (Figure 15, User Manual)
// Broadcast address 0x00, data byte 0xFF
// After P (stop): SDA released, pull-up holds SDA HIGH
// Sensor reads SDA HIGH for t1 > 14us -> latches ADDR=1 -> I2C address = 0x5E
void TLV493D::sendReset()
{
    uint8_t data = 0xFF;
    _i2cReset.write(&data, 1);
}


bool TLV493D::readRegs()
{
    return _i2c.read(_regRead, 10) == HAL_OK;
}


bool TLV493D::writeRegs()
{
	return _i2c.write(_regWrite, 4) == HAL_OK;
}


// Odd parity over all 4 write registers
// Sum of all 32 bits must be odd (User Manual section 7.2.2.1, P bit)
void TLV493D::calcParity()
{
    // Set parity bit to 1 first so algorithm produces odd result
    _regWrite[1] |= 0x80;

    uint8_t parity = 0;
    for (int i = 0; i < 4; i++) {
        parity ^= _regWrite[i];
    }
    parity = parity ^ (parity >> 1);
    parity = parity ^ (parity >> 2);
    parity = parity ^ (parity >> 4);

    if (parity & 0x01) {
        _regWrite[1] |= 0x80;
    } else {
        _regWrite[1] &= ~0x80;
    }
}


// Apply mode bits to regWrite[1] (MOD1) and regWrite[2] (MOD2)
// MOD1 bit1 = FAST, bit0 = LOW
// MOD2 bit6 = LP (low-power period: 0=100ms, 1=12ms)
void TLV493D::applyModeConfig(TLV493D_Mode mode)
{
    const ModeConfig &cfg = MODE_CONFIGS[static_cast<uint8_t>(mode)];

    // Clear FAST and LOW bits in MOD1
    _regWrite[1] &= ~(0x03);
    if (cfg.fast) _regWrite[1] |= (1 << 1);
    if (cfg.lp)   _regWrite[1] |= (1 << 0);

    // Clear LP bit in MOD2, set if needed
    _regWrite[2] &= ~(1 << 6);
    if (cfg.lpPeriod) _regWrite[2] |= (1 << 6);
}


int16_t TLV493D::concat12bit(uint8_t upper, uint8_t lower, bool upperFull)
{
    int16_t value = 0;
    if (upperFull) {
        value  = static_cast<int16_t>(upper) << 8;
        value |= (lower & 0x0F) << 4;
    } else {
        value  = (upper & 0x0F) << 12;
        value |= static_cast<int16_t>(lower) << 4;
    }
    value >>= 4;
    return value;
}

}

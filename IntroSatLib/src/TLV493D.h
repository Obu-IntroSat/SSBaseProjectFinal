#pragma once

#include <stdint.h>
#include "./BaseDevice.h"


namespace IntroSatLib {

	enum class TLV493D_Mode : uint8_t {
		POWERDOWN        = 0,
		FASTMODE         = 1,
		LOWPOWER         = 2,
		ULTRALOWPOWER    = 3,
		MASTERCONTROLLED = 4
	};

	enum class TLV493D_Status : uint8_t {
		OK        = 0,
		ERR_I2C   = 1,
		ERR_FRAME = 2
	};

	class TLV493D : public BaseDevice {

	private:
		static const uint8_t BASE_ADDRESS = 0x5E;

	public:

		TLV493D(I2C_HandleTypeDef *hi2c);

		TLV493D_Status init(TLV493D_Mode mode = TLV493D_Mode::LOWPOWER);
		TLV493D_Status setMode(TLV493D_Mode mode);
		TLV493D_Status read();

		float    X()   const;
		float    Y()   const;
		float    Z()   const;
		float    Temp() const;

		uint32_t getMeasurementDelay_ms() const;

	private:
		I2CDevice &_i2cPowerDown;
		I2CDevice &_i2cReset;
		TLV493D_Mode       _mode;

		uint8_t  _regRead[10] = { 0 };
		uint8_t  _regWrite[4] = { 0 };

		int16_t  _rawX;
		int16_t  _rawY;
		int16_t  _rawZ;
		int16_t  _rawTemp;

		void     sendRecovery();
		void     sendReset();
		bool     readRegs();
		bool     writeRegs();
		void     calcParity();
		void     applyModeConfig(TLV493D_Mode mode);
		int16_t  concat12bit(uint8_t upper, uint8_t lower, bool upperFull);
	};
}

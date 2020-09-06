/**
 * @file PCA9685_Driver.cpp
 * @author rneurink
 * @brief Arduino Library to interact with the PCA9685
 * @version 0.1
 * @date 2020-09-06
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "PCA9685_Driver.h"
#include <Wire.h>

#define ENABLE_DEBUG_OUTPUT

/**
 * @brief Construct a new pca9685 driver::pca9685 driver object
 * 
 */
PCA9685_Driver::PCA9685_Driver()
    : _i2c_address(PCA9685_I2C_ADDR), _i2c(&Wire) {}

/**
 * @brief Construct a new pca9685 driver::pca9685 driver object
 * 
 * @param i2c_address The address to talk to the chip. Default is 0x40 (7 bit) or 0x80 (8 bit)
 */
PCA9685_Driver::PCA9685_Driver(uint8_t i2c_address)
    : _i2c_address(i2c_address), _i2c(&Wire) {}

/**
 * @brief Construct a new pca9685 driver::pca9685 driver object
 * 
 * @param i2c_address The address to talk to the chip. Default is 0x40 (7 bit) or 0x80 (8 bit)
 * @param i2c The reference to the I2C TwoWire hardware abstraction layer
 */
PCA9685_Driver::PCA9685_Driver(uint8_t i2c_address, TwoWire &i2c)
    : _i2c_address(i2c_address), _i2c(&i2c) {}

/**
 * @brief Initializes the I2C HAL and the PCA9685
 * 
 * @param frequency the pwm output frequency to use. Defaults to 1000 Hz
 */
void PCA9685_Driver::begin(float frequency) {
    _i2c->begin();
    restart();

    setOscillFrequency(OSC_FREQ);
    setPWMFrequency(frequency);
}

/**
 * @brief Send a restart command to the PCA9685. 
 * 
 * @param addressType the I2C address type to write to
 */
void PCA9685_Driver::restart(EAddressType addressType) {
    writeByte(addressType, PCA9685_MODE1, MODE1_RESTART);
    delay(10);
}

/**
 * @brief Sets the PCA9685 to sleep
 * 
 * @param addressType the I2C address type to write to
 */
void PCA9685_Driver::sleep(EAddressType addressType) {
    writeByte(addressType, PCA9685_MODE1, (readByte(PCA9685_MODE1) | MODE1_SLEEP));
}

/**
 * @brief Resets the PCA9685 sleep bit
 * 
 * @param addressType the I2C address type to write to
 */
void PCA9685_Driver::wakeUp(EAddressType addressType) {
    writeByte(addressType, PCA9685_MODE1, (readByte(PCA9685_MODE1) & ~MODE1_SLEEP));
}

/**
 * @brief Sets the mode1 register to a given value
 * 
 * @param value the value to set the mode1 register to
 * @param addressType the I2C address type to write to 
 */
void PCA9685_Driver::setMode1(uint8_t value, EAddressType addressType) {
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Setting mode1, old value: ");
    Serial.println(readByte(PCA9685_MODE1), HEX);
#endif
    writeByte(addressType, PCA9685_MODE1, value);
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("mode1, setting value: ");
    Serial.print(value, HEX);
    Serial.print(" reading: ");
    Serial.println(readByte(PCA9685_MODE1), HEX);
#endif
}

/**
 * @brief Sets the mode2 register to a given value
 * 
 * @param value the value to set the mode2 register to
 * @param addressType the I2C address type to write to 
 */
void PCA9685_Driver::setMode2(uint8_t value, EAddressType addressType) {
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Setting mode2, old value: ");
    Serial.println(readByte(PCA9685_MODE2), HEX);
#endif
    writeByte(addressType, PCA9685_MODE2, value);
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("mode2, setting value: ");
    Serial.print(value, HEX);
    Serial.print(" reading: ");
    Serial.println(readByte(PCA9685_MODE2), HEX);
#endif
}

/**
 * @brief Reads the value from the specified register
 * 
 * @param reg_addr the register address to read from
 * @return uint8_t the value from the specified register
 */
uint8_t PCA9685_Driver::readRegister(uint8_t reg_addr) {
    return readByte(reg_addr);
}

/**
 * @brief Writes a given value to the specified register
 * 
 * @param reg_addr the register address to write to
 * @param data the data to write to the register
 * @param addressType the I2C address type to write to 
 */
void PCA9685_Driver::writeRegister(uint8_t reg_addr, uint8_t data, EAddressType addressType) {
    writeByte(addressType, reg_addr, data);
}

/**
 * @brief Enables the external clock on the driver and sets specified PWM frequency
 * 
 * @param frequency The frequency of the external input
 * @param pwmFrequency The PWM frequency used to drive the outputs
 */
void PCA9685_Driver::setExtClock(float frequency, float pwmFrequency, EAddressType addressType) {
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.println("Setting ext clock frequency to: " + String(frequency));
    Serial.println("Setting pwm frequency to: " + String(pwmFrequency));
#endif

    uint8_t mode = (readByte(PCA9685_MODE1) & ~MODE1_RESTART) | MODE1_SLEEP; // Clear restart and set sleep bit
    writeByte(addressType, PCA9685_MODE1, mode);

    writeByte(addressType, PCA9685_MODE1, (mode |= MODE1_EXTCLK));

    setOscillFrequency(frequency);

    int upper_limit = (int)(_osillator_freq / 16384);
    pwmFrequency = pwmFrequency < 1 ? 1 : pwmFrequency > upper_limit ? upper_limit : pwmFrequency; // Limit max lower and upper frequency

    float prescale_value = ((_osillator_freq / (4096 * pwmFrequency) + 0.5) - 1); // Adding 0.5 to force rounding to the nearest int

    // Limit prescale_value to be sure.
    prescale_value = prescale_value < 0x03 ? 0x03 : prescale_value > 0xff ? 0xff : prescale_value;

#ifdef ENABLE_DEBUG_OUTPUT
    Serial.println("Osc freq: " + String(_osillator_freq) + " upper limit: " + String(upper_limit));
    Serial.println("Prescale value: " + String((uint8_t)prescale_value));
#endif

    // Set the prescaler according to the new ext frequency and specified pwm frequency
    writeByte(addressType, PCA9685_PRESCALE, (uint8_t)prescale_value);

    // Clear sleep and restart 
    writeByte(addressType, PCA9685_MODE1, (mode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI);

#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("New Mode1 value: ");
    Serial.println(readByte(PCA9685_MODE1), HEX);
#endif
}

/**
 * @brief Sets the frequency used to drive the outputs
 * 
 * @param frequency the frequency in Hz to try to match
 * @param addressType the I2C address type to write to
 * @return float the actual frequency that the output will generate. Measure and set the oscillator to create a more accurate output
 */
float PCA9685_Driver::setPWMFrequency(float frequency, EAddressType addressType) {
    // Limit max lower and upper frequency. With a 50MHz max external clock a frequency of 3052 Hz can be achieved. 
    // With the internal oscillator the maximum is 1526 Hz
    // prescaler can be calulated with `prescale_value = round(osc_clock/(4096 * frequency)) - 1`
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.println("Setting pwm frequency to: " + String(frequency));
#endif
    int upper_limit = (int)(_osillator_freq / 16384);
    frequency = frequency < 1 ? 1 : frequency > upper_limit ? upper_limit : frequency; // Limit max lower and upper frequency

    float prescale_value = ((_osillator_freq / (4096 * frequency) + 0.5) - 1); // Adding 0.5 to force rounding to the nearest int

    // Limit prescale_value to be sure.
    prescale_value = prescale_value < 0x03 ? 0x03 : prescale_value > 0xff ? 0xff : prescale_value;

#ifdef ENABLE_DEBUG_OUTPUT
    Serial.println("Osc freq: " + String(_osillator_freq) + " upper limit: " + String(upper_limit));
    Serial.println("Prescale value: " + String((uint8_t)prescale_value));
#endif

    // The prescale value can only be set when in sleep
    uint8_t current_mode1 = readByte(PCA9685_MODE1);
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Current Mode1 value: ");
    Serial.println(readByte(PCA9685_MODE1), HEX);
#endif
    writeByte(addressType, PCA9685_MODE1, ((current_mode1 & ~MODE1_RESTART) | MODE1_SLEEP ));
    writeByte(addressType, PCA9685_PRESCALE, (uint8_t)prescale_value);
    writeByte(addressType, PCA9685_MODE1, current_mode1); // Set back the old mode1 values

    delay(5); // 500us delay mandatory but make it a bit higher to be sure
    writeByte(addressType, PCA9685_MODE1, current_mode1 | MODE1_RESTART | MODE1_AI); // Restart and turn on auto increment
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("New Mode1 value: ");
    Serial.println(readByte(PCA9685_MODE1), HEX);
#endif

    return (_osillator_freq / (4096 * ((uint8_t)prescale_value + 1)));
}

/**
 * @brief Return the PWM frequency according to the oscillator frequency and the prescaler
 * 
 * @return float the PWM frequency 
 */
float PCA9685_Driver::getPWMFrequency() {
    return (_osillator_freq / (4096 * (readByte(PCA9685_PRESCALE) + 1)));
}

/**
 * @brief Sets the output mode of the PCA9685 to open drain or totem pole
 * 
 * @param type The output type to use
 * @param addressType the I2C address type to write to
 */
void PCA9685_Driver::setOutputType(EOutputType type, EAddressType addressType) {
    uint8_t mode = readByte(PCA9685_MODE2);
#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("Setting output mode: " + String(type == EOutputType::TotemPole ? "TotemPole" : "OpenDrain") + " current Mode2 value: ");
    Serial.println(mode, HEX);
#endif
    switch (type)
    {
    case EOutputType::TotemPole :
        mode | MODE2_OUTDRV;
        break;
    case EOutputType::OpenDrain :
    default:
        mode & ~MODE2_OUTDRV;
        break;
    }
    writeByte(addressType, PCA9685_MODE2, mode);

#ifdef ENABLE_DEBUG_OUTPUT
    Serial.print("New Mode2 value: ");
    Serial.println(readByte(PCA9685_MODE2), HEX);
#endif
}

/**
 * @brief Sets the output of the driver. 
 * 
 * @param output The output pin to write to from 0..15
 * @param on 0..4095 Specifies the point in the cycle where to turn the output on
 * @param off 0..4095 Specifies the point in the cycle where to turn the output off
 * @param addressType the I2C address type to write to
 */
void PCA9685_Driver::setPWMOutput(uint8_t output, uint16_t on, uint16_t off, EAddressType addressType) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.println("Setting PWM " + String(output) + ": on " + String(on) + " off " + String(off));
#endif

    _i2c->beginTransmission(getAddress(addressType));
    _i2c->write(PCA9685_LED0_ON_L + 4 * output); // Select output based on the first register. Each output has 4 registers
    _i2c->write(on & 0xFF); // Low on byte
    _i2c->write((on >> 8) & 0xFF); // High on byte
    _i2c->write(off & 0xFF); // Low off byte
    _i2c->write((off >> 8) & 0xFF); // High off byte
    _i2c->endTransmission();
}

/**
 * @brief Sets all outputs of the driver
 * 
 * @param on 0..4095 Specifies the point in the cycle where to turn the output on
 * @param off 0..4095 Specifies the point in the cycle where to turn the output off
 * @param addressType the I2C address type to write to
 */
void PCA9685_Driver::setAllPWMOutputs(uint16_t on, uint16_t off, EAddressType addressType) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.println("Setting all PWM: on " + String(on) + " off " + String(off));
#endif

    _i2c->beginTransmission(getAddress(addressType));
    _i2c->write(PCA9685_ALL_LED_ON_L);
    _i2c->write(on & 0xFF); // Low on byte
    _i2c->write((on >> 8) & 0xFF); // High on byte
    _i2c->write(off & 0xFF); // Low off byte
    _i2c->write((off >> 8) & 0xFF); // High off byte
    _i2c->endTransmission();
}


/**
 * @brief Returns the oscillator frequency that is used in the output calculations 
 * 
 * @return uint32_t the oscillator frequency. This is a datasheet value or the specified value with setOscillFrequency as the frequency is not measured
 */
uint32_t PCA9685_Driver::getOscillFrequency() {
    return _osillator_freq;
}

/**
 * @brief Sets the oscillator frequency used in the calculations of the pwm output frequency. Make sure to set this when using an external clock.
 * 
 * @param frequency the frequency in Hz
 */
void PCA9685_Driver::setOscillFrequency(uint32_t frequency) {
    _osillator_freq = frequency;
}


/*
 *  PRIVATE
 */ 

/**
 * @brief Resolves the EAddressType into an address
 * 
 * @param addressType the I2C address type to write to. This translates to the stored addresses in the class
 * @return uint8_t the I2C address translated from the addressType
 */
uint8_t PCA9685_Driver::getAddress(EAddressType addressType) {
    uint8_t i2c_address;
    switch (addressType)
    {
    case EAddressType::AllCall:
        i2c_address = _i2c_allCall_address;
        break;
    case EAddressType::SubCall1:
        i2c_address = _i2c_sub1_address;
        break;
    case EAddressType::SubCall2:
        i2c_address = _i2c_sub2_address;
        break;
    case EAddressType::SubCall3:
        i2c_address = _i2c_sub3_address;
        break;
    case EAddressType::Normal:
    default:
    i2c_address = _i2c_address;
        break;
    }
    return i2c_address;
}

/**
 * @brief Reads a register from the normal I2C address stored in _i2c_address
 * 
 * @param reg_addr the register address to read
 * @return uint8_t the register contents
 */
uint8_t PCA9685_Driver::readByte(uint8_t reg_addr) {
    _i2c->beginTransmission(_i2c_address);
    _i2c->write(reg_addr);
    _i2c->endTransmission();

    _i2c->requestFrom((uint8_t)_i2c_address, (uint8_t)1);
    return _i2c->read();
}

/**
 * @brief Writes a value to a register on the given I2C address
 * 
 * @param addressType the I2C address type to write to. This translates to the stored addresses in the class
 * @param reg_addr the register address to write to
 * @param data the data to write to the register
 */
void PCA9685_Driver::writeByte(EAddressType addressType, uint8_t reg_addr, uint8_t data) {
    _i2c->beginTransmission(getAddress(addressType));
    _i2c->write(reg_addr);
    _i2c->write(data);
    _i2c->endTransmission();
}
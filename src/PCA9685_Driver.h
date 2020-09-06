/**
 * @file PCA9685_Driver.h
 * @author rneurink
 * @brief Arduino Library to interact with the PCA9685
 * @version 0.1
 * @date 2020-09-06
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef _PCA9685_H
#define _PCA9685_H

#include <Arduino.h>
#include <Wire.h>

#define OSC_FREQ 25000000

#define PCA9685_I2C_ADDR 0x40
#define PCA9685_I2C_ALL_CALL 0x70
#define PCA9685_SUB_CALL_1 0x71
#define PCA9685_SUB_CALL_2 0x72
#define PCA9685_SUB_CALL_3 0x73 

// Registers
#define PCA9685_MODE1           0x00 // Mode register 1
#define PCA9685_MODE2           0x01 // Mode register 2
#define PCA9685_SUBADR1         0x02 // I2C-bus subaddress 1
#define PCA9685_SUBADR2         0x03 // I2C-bus subaddress 2
#define PCA9685_SUBADR3         0x04 // I2C-bus subaddress 3
#define PCA9685_ALLCALLADR      0x05 // LED All Call I2C-bus address
#define PCA9685_LED0_ON_L       0x06 // LED0 control byte 0
#define PCA9685_LED0_ON_H       0x07 // LED0 control byte 1
#define PCA9685_LED0_OFF_L      0x08 // LED0 control byte 2
#define PCA9685_LED0_OFF_H      0x09 // LED0 control byte 3
// All 16 led registers up to 0x45

#define PCA9685_ALL_LED_ON_L    0xFA // load al the LEDn_ON registers, low byte
#define PCA9685_ALL_LED_ON_H    0xFB // load al the LEDn_ON registers, high byte
#define PCA9685_ALL_LED_OFF_L   0xFC // load al the LEDn_OFF registers, low byte
#define PCA9685_ALL_LED_OFF_H   0xFD // load al the LEDn_OFF registers, high byte
#define PCA9685_PRESCALE        0xFE // prescaler for the PWM output frequency
#define PCA9685_TESTMODE        0xFF // defines the test mode to be entered

// MODE1 Bits
#define MODE1_ALLCALL   (1 << 0) // Default 1, respond to LED ALL Call I2C-bus address
#define MODE1_SUB3      (1 << 1) // Default 0, respond to I2C-bus subaddress 3
#define MODE1_SUB2      (1 << 2) // Default 0, respond to I2C-bus subaddress 3
#define MODE1_SUB1      (1 << 3) // Default 0, respond to I2C-bus subaddress 3
#define MODE1_SLEEP     (1 << 4) // Default 1, low power mode. Oscillator off
#define MODE1_AI        (1 << 5) // Default 0, register auto increment
#define MODE1_EXTCLK    (1 << 6) // Default 0, use external clock (sticky)
#define MODE1_RESTART   (1 << 7) // Default 0, enable restart 

// MODE2 Bits
#define MODE2_OUTNE_L   (1 << 0) // Default 0, output enable setup, low byte
#define MODE2_OUTNE_H   (1 << 1) // Default 0, output enable setup, high byte
#define MODE2_OUTDRV    (1 << 2) // Default 1, output configuration totem pole on 1, open drain on 0
#define MODE2_OCH       (1 << 3) // Default 1, outputs change on ack on 1, on stop on 0
#define MODE2_INVRT     (1 << 4) // Default 0, invert logic state

enum EAddressType {
    Normal,
    AllCall,
    SubCall1,
    SubCall2,
    SubCall3
};

enum EOutputType {
    OpenDrain,
    TotemPole
};

/**
 * @brief Class that stores states and provides functions to interact with the PCA9685
 * 
 */
class PCA9685_Driver {
public:
    PCA9685_Driver();
    PCA9685_Driver(uint8_t i2c_address);
    PCA9685_Driver(uint8_t i2c_address, TwoWire &i2c);
    void begin(float frequency = 1000);
    void restart(EAddressType addressType = EAddressType::Normal);
    void sleep(EAddressType addressType = EAddressType::Normal);
    void wakeUp(EAddressType addressType = EAddressType::Normal);

    void setMode1(uint8_t value, EAddressType addressType = EAddressType::Normal);
    void setMode2(uint8_t value, EAddressType addressType = EAddressType::Normal);
    uint8_t readRegister(uint8_t reg_addr);
    void writeRegister(uint8_t reg_addr, uint8_t data, EAddressType addressType = EAddressType::Normal);

    void setExtClock(float frequency, float pwmFrequency = 0, EAddressType addressType = EAddressType::Normal);
    float setPWMFrequency(float frequency, EAddressType addressType = EAddressType::Normal);
    float getPWMFrequency();

    void setOutputType(EOutputType type, EAddressType addressType = EAddressType::Normal);
    void setPWMOutput(uint8_t output, uint16_t on, uint16_t off, EAddressType addressType = EAddressType::Normal);
    void setAllPWMOutputs(uint16_t on, uint16_t off, EAddressType addressType = EAddressType::Normal);

    uint32_t getOscillFrequency();
    void setOscillFrequency(uint32_t frequency);

private:
    TwoWire *_i2c;
    uint8_t _i2c_address;

    uint8_t _i2c_allCall_address = PCA9685_I2C_ALL_CALL;
    uint8_t _i2c_sub1_address = PCA9685_SUB_CALL_1;
    uint8_t _i2c_sub2_address = PCA9685_SUB_CALL_2;
    uint8_t _i2c_sub3_address = PCA9685_SUB_CALL_3;

    uint32_t _osillator_freq;

    uint8_t getAddress(EAddressType addressType);
    uint8_t readByte(uint8_t reg_addr);
    void writeByte(EAddressType addressType, uint8_t reg_addr, uint8_t data);
    
};

#endif //_PCA9685_H
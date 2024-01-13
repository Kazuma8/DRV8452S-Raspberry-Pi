/*  DRV8461_Registers.h

    Register addresses and settings for the Texas Instruments DRV8461 stepper
    driver.

*/
#pragma once

#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>

#ifndef DRV8461_REGISTERS_H
#define DRV8461_REGISTERS_H


// REGISTER ADDRESSES ************************************************************************************************// 
enum class DRV8461_REG_ADDR : uint8_t {
  DRV8461_REG_FAULT        = 0x00,            // Fault status register.
  DRV8461_REG_DIAG1        = 0x01,            // DIAG status 1.
  DRV8461_REG_DIAG2        = 0x02,            // DIAG status 2.
  DRV8461_REG_DIAG3        = 0x03,            // DIAG status 3.
  DRV8461_REG_CTRL1        = 0x04,            // Control register 1.
  DRV8461_REG_CTRL2        = 0x05,            // Control register 2.
  DRV8461_REG_CTRL3        = 0x06,            // Control register 3.
  DRV8461_REG_CTRL4        = 0x07,            // Control register 4.
  DRV8461_REG_CTRL5        = 0x08,            // Control register 5.
  DRV8461_REG_CTRL6        = 0x09,            // Control register 6.
  DRV8461_REG_CTRL7        = 0x0A,            // Control register 7.
  DRV8461_REG_CTRL8        = 0x0B,            // Control register 8.
  DRV8461_REG_CTRL9        = 0x0C,            // Control register 9.
  DRV8461_REG_CTRL10       = 0x0D,            // Control Register 10.
  DRV8461_REG_CTRL11       = 0x0E,            // Control Register 11.
  DRV8461_REG_CTRL12       = 0x0F,            // Control Register 12.
  DRV8461_REG_CTRL13       = 0x10,            // Control Register 13.
  DRV8461_REG_INDEX1       = 0x11,            // Index Register 1.
  DRV8461_REG_INDEX2       = 0x12,            // Index Register 2.
  DRV8461_REG_INDEX3       = 0x13,            // Index Register 3.
  DRV8461_REG_INDEX4       = 0x14,            // Index Register 4.
  DRV8461_REG_INDEX5       = 0x15,            // Index Register 5.
  DRV8461_REG_CUSTOM_CTRL1 = 0x16,            // Custom Microstep Register 1.
  DRV8461_REG_CUSTOM_CTRL2 = 0x17,            // Custom Microstep Register 2
  DRV8461_REG_CUSTOM_CTRL3 = 0x18,            // Custom Microstep Register 3
  DRV8461_REG_CUSTOM_CTRL4 = 0x19,            // Custom Microstep Register 4
  DRV8461_REG_CUSTOM_CTRL5 = 0x1A,            // Custom Microstep Register 5
  DRV8461_REG_CUSTOM_CTRL6 = 0x1B,            // Custom Microstep Register 6
  DRV8461_REG_CUSTOM_CTRL7 = 0x1C,            // Custom Microstep Register 7
  DRV8461_REG_CUSTOM_CTRL8 = 0x1D,            // Custom Microstep Register 8
  DRV8461_REG_CUSTOM_CTRL9 = 0x1E,            // Custom Microstep Register 9
  DRV8461_REG_ATQ_CTRL1    = 0x1F,            // Auto Torque Register 1.
  DRV8461_REG_ATQ_CTRL2    = 0x20,            // Auto Torque Register 2.
  DRV8461_REG_ATQ_CTRL3    = 0x21,            // Auto Torque Register 3.
  DRV8461_REG_ATQ_CTRL4    = 0x22,            // Auto Torque Register 4.
  DRV8461_REG_ATQ_CTRL5    = 0x23,            // Auto Torque Register 5.
  DRV8461_REG_ATQ_CTRL6    = 0x24,            // Auto Torque Register 6.
  DRV8461_REG_ATQ_CTRL7    = 0x25,            // Auto Torque Register 7.
  DRV8461_REG_ATQ_CTRL8    = 0x26,            // Auto Torque Register 8.
  DRV8461_REG_ATQ_CTRL9    = 0x27,            // Auto Torque Register 9.
  DRV8461_REG_ATQ_CTRL10   = 0x28,            // Auto Torque Register 10.
  DRV8461_REG_ATQ_CTRL11   = 0x29,            // Auto Torque Register 11.
  DRV8461_REG_ATQ_CTRL12   = 0x2A,            // Auto Torque Register 12.
  DRV8461_REG_ATQ_CTRL13   = 0x2B,            // Auto Torque Register 13.
  DRV8461_REG_ATQ_CTRL14   = 0x2C,            // Auto Torque Register 14.
  DRV8461_REG_ATQ_CTRL15   = 0x2D,            // Auto Torque Register 15.
  DRV8461_REG_ATQ_CTRL16   = 0x2E,            // Auto Torque Register 16.
  DRV8461_REG_ATQ_CTRL17   = 0x2F,            // Auto Torque Register 17.
  DRV8461_REG_ATQ_CTRL18   = 0x30,            // Auto Torque Register 18.
  DRV8461_REG_SS_CTRL1     = 0x31,            // Silent Step Register 1.
  DRV8461_REG_SS_CTRL2     = 0x32,            // Silent Step Register 2.
  DRV8461_REG_SS_CTRL3     = 0x33,            // Silent Step Register 3.
  DRV8461_REG_SS_CTRL4     = 0x34,            // Silent Step Register 4.
  DRV8461_REG_SS_CTRL5     = 0x35,            // Silent Step Register 5.
  DRV8461_REG_CTRL14       = 0x3C,            // Control Register 14.
};

///FROM POLOLU FILE

/// This class provides low-level functions for reading and writing from the SPI
/// interface of a DRV8434S stepper motor controller IC.
///
/// Most users should use the HighPowerStepperDriver class, which provides a
/// higher-level interface, instead of this class.
class DRV8434SSPI
{
public:
  /// Configures this object to use the specified pin as a chip select pin.
  ///
  /// You must use a chip select pin; the DRV8434S requires it.
  void setChipSelectPin(uint8_t pin)
  {
    csPin = pin;
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
  }

  /// Reads the register at the given address and returns its raw value.
  uint8_t readReg(uint8_t address)
  {
    // Arduino out / DRV8434 in: First byte contains read/write bit and register
    // address; second byte is unused.
    // Arduino in / DRV8434 out: First byte contains status; second byte
    // contains data in register being read.

    selectChip();
    lastStatus = transfer((0x20 | (address & 0b11111)) << 1);
    uint8_t data = transfer(0);
    deselectChip();
    return data;
  }

  /// Reads the register at the given address and returns its raw value.
  uint16_t readReg(DRV8461_REG_ADDR address)
  {
    return readReg((uint8_t)address);
  }

  /// Writes the specified value to a register.
  uint8_t writeReg(uint8_t address, uint8_t value)
  {
    // Arduino out / DRV8434 in: First byte contains read/write bit and register
    // address; second byte contains data to write to register.
    // Arduino in / DRV8434 out: First byte contains status; second byte
    // contains old (existing) data in register being written to.

    selectChip();
    lastStatus = transfer((address & 0b11111) << 1);
    uint8_t oldData = transfer(value);
    // The CS line must go low after writing for the value to actually take
    // effect.
    deselectChip();
    return oldData;
  }

  /// Writes the specified value to a register.
  void writeReg(DRV8461_REG_ADDR address, uint8_t value)
  {
    writeReg((uint8_t)address, value);
  }

private:

  SPISettings settings = SPISettings(500000, MSBFIRST, SPI_MODE1);

  uint8_t transfer(uint8_t value)
  {
    return SPI.transfer(value);
  }

  void selectChip()
  {
    digitalWrite(csPin, LOW);
    SPI.beginTransaction(settings);
  }

  void deselectChip()
  {
   SPI.endTransaction();
   digitalWrite(csPin, HIGH);
  }

  uint8_t csPin;

public:

  /// The status reported by the driver during the last read or write.  This
  /// status is the same as that which would be returned by reading the FAULT
  /// register with DRV8434S::readFault(), except the upper two bits are always
  /// 1.
  uint8_t lastStatus = 0;
};



///FAULT
// FAULT REGISTER SETINGS ********************************************************************************************// 
enum class DRV8461_FAULT_Reg_Val : uint8_t {
  DRV8461_FAULT_FAULT = 0x80,          // Fault bit is high when nFAULT output is low.
  DRV8461_FAULT_SPI_ERR = 0x40,        // Indicates SPI protocol errors.
  DRV8461_FAULT_UVLO = 0x20,           // Indicates undervoltage lockout fault condition.
  DRV8461_FAULT_CPUV = 0x10,           // Indicates charge pump undervoltage fault condition.
  DRV8461_FAULT_OCP = 0x08,            // Indicates overcurrent fault condition.
  DRV8461_FAULT_STL = 0x04,            // Indicates motor stall condition.
  DRV8461_FAULT_TF = 0x02,             // Indicates overtemperature warning/shutdown condition.
  DRV8461_FAULT_OL = 0x01,             // Indicates open-load condition.
};




///DIAG
// DIAG STATUS 1 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_DIAG1_Reg_Val : uint8_t {
  DRV8461_DIAG1_OCP_LS2_B = 0x80,      // Indicates overcurrent fault on the low-side FET of half bridge 2 in BOUT.
  DRV8461_DIAG1_OCP_HS2_B = 0x40,      // Indicates overcurrent fault on the high-side FET of half bridge 2 in BOUT.
  DRV8461_DIAG1_OCP_LS1_B = 0x20,      // Indicates overcurrent fault on the low-side FET of half bridge 1 in BOUT.
  DRV8461_DIAG1_OCP_HS1_B = 0x10,      // Indicates overcurrent fault on the high-side FET of half bridge 1 in BOUT.
  DRV8461_DIAG1_OCP_LS2_A = 0x08,      // Indicates overcurrent fault on the low-side FET of half bridge 2 in AOUT.
  DRV8461_DIAG1_OCP_HS2_A = 0x04,      // Indicates overcurrent fault on the high-side FET of half bridge 2 in AOUT.
  DRV8461_DIAG1_OCP_LS1_A = 0x02,      // Indicates overcurrent fault on the low-side FET of half bridge 1 in AOUT.
  DRV8461_DIAG1_OCP_HS1_A = 0x01,      // Indicates overcurrent fault on the high-side FET of half bridge 1 in AOUT.
};


// DIAG STATUS 2 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_DIAG2_Reg_Val : uint8_t {
  DRV8461_DIAG2_STSL = 0x80,           // Indicates operating in standstill power saving mode
  DRV8461_DIAG2_OTW = 0x40,            // Indicates overtemperature warning.
  DRV8461_DIAG2_OTS = 0x20,            // Indicates overtemperature shutdown.
  DRV8461_DIAG2_STL_LRN_OK = 0x10,     // Indicates stall detection learning is successful.
  DRV8461_DIAG2_STALL = 0x08,          // Indicates motor stall condition.
  DRV8461_DIAG2_OL_B = 0x02,           // Indicates open-load detection on BOUT.
  DRV8461_DIAG2_OL_A = 0x01,           // Indicates open-load detection on AOUT
};


// DIAG STATUS 3 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_DIAG3_Reg_Val : uint8_t {
  DRV8461_DIAG3_NHOME = 0x40,          // Indicates indexer is at a position other than home.
  DRV8461_DIAG3_CNT_OFLW = 0x20,       // Indicates ATQ_CNT is more than ATQ_UL.
  DRV8461_DIAG3_CNT_UFLW = 0x10,       // Indicates ATQ_CNT is less than ATQ_LL.
  DRV8461_DIAG3_NPOR = 0x02,           // Low indicates a prior VCC UVLO event, High indicates that NPOR has been cleared by CLR_FLT or nSLEEP after a VCC UVLO.
};



//CONTROL
// CONTROL 1 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_CTRL1_Reg_Val : uint8_t {
  DRV8461_CTRL1_EN_OUT  = 0x80,        // Write '0' to disable all outputs.
  DRV8461_CTRL1_SR      = 0x40,        // 0 = Output Rise/Fall time = 140ns, 1 = 70ns.
  DRV8461_CTRL1_IDX_RST = 0x20,        // Resets the indexer to 45˚ electrical angle, but keeps contents of registers.
  DRV8461_CTRL1_TOFF    = 0x18,        // ??? (default is 01b = 19 us).
  DRV8461_CTRL2_DECAY   = 0x07,        // ??? (default is 111b = Smart tune Ripple Control).
};

// Specific values for TOFF
enum class DRV8461_PWM_TOFF : uint8_t {
  DRV8461_TOFF_9_5US = 0b00,           // 9.5 μs.
  DRV8461_TOFF_19US  = 0b01,           // 19 us.
  DRV8461_TOFF_27US  = 0b10,           // 27 us.
  DRV8461_TOFF_35US  = 0b11,           // 35 us.
};

//Specitic Values for Decay Mode
enum class DRV8461_Decay_Mode : uint8_t {
  DRV8461_DECAY_SLOW_SLOW     = 0b000,  // Increasing SLOW, decreasing SLOW.
  DRV8461_DECAY_SLOW_MIX30    = 0b001,  // Increasing SLOW, decreasing MIXED 30%.
  DRV8461_DECAY_SLOW_MIX60    = 0b010,  // Increasing SLOW, decreasing MIXED 60%.
  DRV8461_DECAY_SLOW_FAST     = 0b011,  // Increasing SLOW, decreasing FAST.
  DRV8461_DECAY_MIX30_MIX30   = 0b100,  // Increasing MIXED 30%, decreasing MIXED 30%.
  DRV8461_DECAY_MIX60_MIX60   = 0b101,  // Increasing MIXED 60%, decreasing MIXED 60%.
  DRV8461_DECAY_DYNAMIC       = 0b110,  // Smart tune Dynamic Decay.
  DRV8461_DECAY_SMART_RIPPLE  = 0b111,  // Smart tune Ripple Control (default).
};


// CONTROL 2 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_CTRL2_Reg_Val : uint8_t {
  DRV8461_CTRL2_DIR = 0x80,            // Direction input (when SPI_DIR = 1).
  DRV8461_CTRL2_STEP = 0x40,           // Step input (when SPI_SETP = 1), logic '1' advances one step (self-clearing).
  DRV8461_CTRL2_SPI_DIR = 0x20,        // Direction is controlled by SPI when high.
  DRV8461_CTRL2_SPI_STEP = 0x10,       // Stepping is controlled by SPI when high.
  DRV8461_CTRL2_MICROSTEP_MODE = 0x0F, // Micro-stepping mode (default is 0110b = 1/16 step).
};

//Specific Values for Microstepping
enum class DRV8461_Micostep_Mode : uint8_t
{
  DRV8461_MICROSTEP_1_100 = 0b0000,    // Full step (2-phase excitation) with 100% current.
  DRV8461_MICROSTEP_1_71  = 0b0001,    // Full step (2-phase excitation) with 71% current.
  DRV8461_MICROSTEP_2_NC  = 0b0010,    // Non-circular 1/2 step.
  DRV8461_MICROSTEP_2     = 0b0011,    // 1/2 step.
  DRV8461_MICROSTEP_4     = 0b0100,    // 1/4 step.
  DRV8461_MICROSTEP_8     = 0b0101,    // 1/8 step.
  DRV8461_MICROSTEP_16    = 0b0110,    // 1/16 step.
  DRV8461_MICROSTEP_32    = 0b0111,    // 1/32 step.
  DRV8461_MICROSTEP_64    = 0b1000,    // 1/64 step.
  DRV8461_MICROSTEP_128   = 0b1001,    // 1/128 step.
  DRV8461_MICROSTEP_256   = 0b1010,    // 1/256 step.
};


// CONTROL 3 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_CTRL3_Reg_Val : uint8_t {
  DRV8461_CTRL3_CLR_FLT = 0x80,        // Clear all latched fault bits (self-resetting) by writing 1.
  DRV8461_CTRL3_LOCK = 0x70,           // Lock registers and ignore further register writes except for CLR_FLT.
  DRV8461_CTRL3_TOCP = 0x08,           // Set overcurrent protection deglitch time (0 = 1.2 us, 1 = 2.2us) default 1.
  DRV8461_CTRL3_OCP_MODE = 0x04,       // Overcurrent conditon fault latching (0 = latching, 1 = automatic retrying) default 0.
  DRV8461_CTRL3_OTSD_MODE = 0x02,      // Overtemperature conditon fault latching (0 = latching, 1 = automatic recovery) default 0.
  DRV8461_CTRL3_TW_REP = 0x01,         // Overtemperature warning reporting on nFAULT (0 = not reported, 1 = reported) default 0.
};


// CONTROL 4 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_CTRL4_Reg_Val : uint8_t {
  DRV8461_CTRL4_TBLANK_TIME = 0xC0,    // Controls current sense blanking time (default 1.5us).
  DRV8461_CTRL4_STL_LRN = 0x20,        // Automatic learning of stall detection threshold (set to 1, resets to 0).
  DRV8461_CTRL4_EN_STL = 0x10,         // Stall detection (1 = enabled) default 0.
  DRV8461_CTRL4_STL_REP = 0x08,        // Stall detection report on nFAULT (1 = enabled) default 1.
  DRV8461_CTRL4_FRQ_CHG = 0x40,        // STEP input is filtered as per STEP_FRQ_TOL bits (0 = enabled) default 0.
  DRV8461_CTRL4_STEP_FRQ_TOL = 0x03,   // Programs the filter setting for STEP input. Default 2%.
};

//Specific Values for Step Frequencies
enum class DRV8461_Step_Frequency : uint8_t {
  DRV8461_STEP_FRQ_FLTR_1 = 0b00,      // 1% filtering
  DRV8461_STEP_FRQ_FLTR_2 = 0b01,      // 2% filtering
  DRV8461_STEP_FRQ_FLTR_4 = 0b10,      // 4% filtering
  DRV8461_STEP_FRQ_FLTR_6 = 0b11,      // 6% filtering
};


// CONTROL 5 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_CTRL5_Reg_Val : uint8_t {
  DRV8461_CTRL5_STALL_TH = 0xFF,       // Lower 8-btis of stall threshold (default is 00000011b / 3).
};


// CONTROL 6 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_CTRL6_Reg_Val : uint8_t {
  DRV8461_CTRL6_RC_RIPPLE = 0xC0,      // Ripple setting (00b = 1%, 01b = 2%, 10b = 4%, 11b = 6%).
  DRV8461_CTRL6_EN_SSC    = 0x20,      // Enable spread-spectrum (on by default).
  DRV8461_CTRL6_TRQ_SCALE = 0x10,      // Apply 8x torque scaling (off by default).
  DRV8461_CTRL6_STL_REP   = 0x0F,      // Upper 4-bits of stall threshold.
};


//Specific Values for Ripple Percentage
enum class DRV8461_RC_Ripple : uint8_t {
  DRV8461_RIPPLE_1 = 0b00,    // 1% ripple (default).
  DRV8461_RIPPLE_2 = 0b01,    // 2% ripple (default).
  DRV8461_RIPPLE_4 = 0b10,    // 4% ripple (default).
  DRV8461_RIPPLE_6 = 0b11,    // 6% ripple (default).
};


// CONTROL 7 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_CTRL7_Reg_Val : uint8_t {
  DRV8461_CTRL7_TRQ_COUNT = 0xFF,      // Lower 8-bits of TRQ_COUNT.
};


// CONTROL 8 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_CTRL8_Reg_Val : uint8_t {
  DRV8461_CTRL8_TRQ_SCALE = 0x0F,      // Upper 4-bits of TRQ_COUNT.
};


// CONTROL 9 REGISTER SETINGS ************************************************************************************// 
enum class DRV8461_CTRL9_Reg_Val : uint8_t {
  DRV8461_CTRL9_EN_OL     = 0x80,      // Enable open load detection.
  DRV8461_CTRL9_OL_MODE   = 0x40,      // Open-load fault clearing mode.
  DRV8461_CTRL9_OL_T      = 0x30,      // Controls the open load fault detection time. (Default = 60ms, max)
  DRV8461_CTRL9_STEP_EDGE = 0x08,      // Active edge for STEP input (0 = only rising, 1 = both rising and falling) default 0.
  DRV8461_CTRL9_RES_AUTO  = 0x06,      // Controls microstepping resolution in auto mode. (Default = 1/256)
  DRV8461_CTRL9_EN_AUTO   = 0x01,      // Automatic microstepping (0 = disabled, 1 = enabled) default 0.
};

//Specific Values for Open Load Detection Time
enum class DRV8461_Open_Load_Detection_Time : uint8_t {
  DRV8461_OLT_30  = 0b00,              // 30ms
  DRV8461_OLT_60  = 0b01,              // 60ms
  DRV8461_OLT_120 = 0b10,              // 120ms
};

//Specific Values for Automatic Microstepping
enum class DRV8461_Auto_Microstep : uint8_t {
  DRV8461_MICRO_RES_256 = 0b00,        // 1/256
  DRV8461_MICRO_RES_128 = 0b01,        // 1/128
  DRV8461_MICRO_RES_64  = 0b10,        // 1/64
  DRV8461_MICRO_RES_32  = 0b11,        // 1/32
};


// CONTROL 10 REGISTER SETINGS ***********************************************************************************// 
enum class DRV8461_CTRL10_Reg_Val : uint8_t {
  DRV8461_CTRL10_ISTSL = 0xFF,         // Determines the holding current
};

//Specific Values for Holding Current
enum class DRV8461_Holding_Current : uint8_t {
  DRV8461_ISTSL_256  = 0b11111111,      // 256/256 * 100
  DRV8461_ISTSL_255  = 0b11111110,      // 255/256 * 100
  DRV8461_ISTSL_254  = 0b11111101,      // 254/256 * 100
  DRV8461_ISTSL_253  = 0b11111100,      // 253/256 * 100
  DRV8461_ISTSL_252  = 0b11111011,      // 252/256 * 100
  DRV8461_ISTSL_251  = 0b11111010,      // 251/256 * 100
  DRV8461_ISTSL_250  = 0b11111001,      // 250/256 * 100
  DRV8461_ISTSL_249  = 0b11111000,      // 249/256 * 100
  DRV8461_ISTSL_248  = 0b11110111,      // 248/256 * 100
  DRV8461_ISTSL_247  = 0b11110110,      // 247/256 * 100
  DRV8461_ISTSL_246  = 0b11110101,      // 246/256 * 100
  DRV8461_ISTSL_245  = 0b11110100,      // 245/256 * 100
  DRV8461_ISTSL_244  = 0b11110011,      // 244/256 * 100
  DRV8461_ISTSL_243  = 0b11110010,      // 243/256 * 100
  DRV8461_ISTSL_242  = 0b11110001,      // 242/256 * 100
  DRV8461_ISTSL_241  = 0b11110000,      // 241/256 * 100
  DRV8461_ISTSL_240  = 0b11101111,      // 240/256 * 100
  DRV8461_ISTSL_239  = 0b11101110,      // 239/256 * 100
  DRV8461_ISTSL_238  = 0b11101101,      // 238/256 * 100
  DRV8461_ISTSL_237  = 0b11101100,      // 237/256 * 100
  DRV8461_ISTSL_236  = 0b11101011,      // 236/256 * 100
  DRV8461_ISTSL_235  = 0b11101010,      // 235/256 * 100
  DRV8461_ISTSL_234  = 0b11101001,      // 234/256 * 100
  DRV8461_ISTSL_233  = 0b11101000,      // 233/256 * 100
  DRV8461_ISTSL_232  = 0b11100111,      // 232/256 * 100
  DRV8461_ISTSL_231  = 0b11100110,      // 231/256 * 100
  DRV8461_ISTSL_230  = 0b11100101,      // 230/256 * 100
  DRV8461_ISTSL_229  = 0b11100100,      // 229/256 * 100
  DRV8461_ISTSL_228  = 0b11100011,      // 228/256 * 100
  DRV8461_ISTSL_227  = 0b11100010,      // 227/256 * 100
  DRV8461_ISTSL_226  = 0b11100001,      // 226/256 * 100
  DRV8461_ISTSL_225  = 0b11100000,      // 225/256 * 100
  DRV8461_ISTSL_224  = 0b11011111,      // 224/256 * 100
  DRV8461_ISTSL_223  = 0b11011110,      // 223/256 * 100
  DRV8461_ISTSL_222  = 0b11011101,      // 222/256 * 100
  DRV8461_ISTSL_221  = 0b11011100,      // 221/256 * 100
  DRV8461_ISTSL_220  = 0b11011011,      // 220/256 * 100
  DRV8461_ISTSL_219  = 0b11011010,      // 219/256 * 100
  DRV8461_ISTSL_218  = 0b11011001,      // 218/256 * 100
  DRV8461_ISTSL_217  = 0b11011000,      // 217/256 * 100
  DRV8461_ISTSL_216  = 0b11010111,      // 216/256 * 100
  DRV8461_ISTSL_215  = 0b11010110,      // 215/256 * 100
  DRV8461_ISTSL_214  = 0b11010101,      // 214/256 * 100
  DRV8461_ISTSL_213  = 0b11010100,      // 213/256 * 100
  DRV8461_ISTSL_212  = 0b11010011,      // 212/256 * 100
  DRV8461_ISTSL_211  = 0b11010010,      // 211/256 * 100
  DRV8461_ISTSL_210  = 0b11010001,      // 210/256 * 100
  DRV8461_ISTSL_209  = 0b11010000,      // 209/256 * 100
  DRV8461_ISTSL_208  = 0b11001111,      // 208/256 * 100
  DRV8461_ISTSL_207  = 0b11001110,      // 207/256 * 100
  DRV8461_ISTSL_206  = 0b11001101,      // 206/256 * 100
  DRV8461_ISTSL_205  = 0b11001100,      // 205/256 * 100
  DRV8461_ISTSL_204  = 0b11001011,      // 204/256 * 100
  DRV8461_ISTSL_203  = 0b11001010,      // 203/256 * 100
  DRV8461_ISTSL_202  = 0b11001001,      // 202/256 * 100
  DRV8461_ISTSL_201  = 0b11001000,      // 201/256 * 100
  DRV8461_ISTSL_200  = 0b11000111,      // 200/256 * 100
  DRV8461_ISTSL_199  = 0b11000110,      // 199/256 * 100
  DRV8461_ISTSL_198  = 0b11000101,      // 198/256 * 100
  DRV8461_ISTSL_197  = 0b11000100,      // 197/256 * 100
  DRV8461_ISTSL_196  = 0b11000011,      // 196/256 * 100
  DRV8461_ISTSL_195  = 0b11000010,      // 195/256 * 100
  DRV8461_ISTSL_194  = 0b11000001,      // 194/256 * 100
  DRV8461_ISTSL_193  = 0b11000000,      // 193/256 * 100
  DRV8461_ISTSL_192  = 0b10111111,      // 192/256 * 100
  DRV8461_ISTSL_191  = 0b10111110,      // 191/256 * 100
  DRV8461_ISTSL_190  = 0b10111101,      // 190/256 * 100
  DRV8461_ISTSL_189  = 0b10111100,      // 189/256 * 100
  DRV8461_ISTSL_188  = 0b10111011,      // 188/256 * 100
  DRV8461_ISTSL_187  = 0b10111010,      // 187/256 * 100
  DRV8461_ISTSL_186  = 0b10111001,      // 186/256 * 100
  DRV8461_ISTSL_185  = 0b10111000,      // 185/256 * 100
  DRV8461_ISTSL_184  = 0b10110111,      // 184/256 * 100
  DRV8461_ISTSL_183  = 0b10110110,      // 183/256 * 100
  DRV8461_ISTSL_182  = 0b10110101,      // 182/256 * 100
  DRV8461_ISTSL_181  = 0b10110100,      // 181/256 * 100
  DRV8461_ISTSL_180  = 0b10110011,      // 180/256 * 100
  DRV8461_ISTSL_179  = 0b10110010,      // 179/256 * 100
  DRV8461_ISTSL_178  = 0b10110001,      // 178/256 * 100
  DRV8461_ISTSL_177  = 0b10110000,      // 177/256 * 100
  DRV8461_ISTSL_176  = 0b10101111,      // 176/256 * 100
  DRV8461_ISTSL_175  = 0b10101110,      // 175/256 * 100
  DRV8461_ISTSL_174  = 0b10101101,      // 174/256 * 100
  DRV8461_ISTSL_173  = 0b10101100,      // 173/256 * 100
  DRV8461_ISTSL_172  = 0b10101011,      // 172/256 * 100
  DRV8461_ISTSL_171  = 0b10101010,      // 171/256 * 100
  DRV8461_ISTSL_170  = 0b10101001,      // 170/256 * 100
  DRV8461_ISTSL_169  = 0b10101000,      // 169/256 * 100
  DRV8461_ISTSL_168  = 0b10100111,      // 168/256 * 100
  DRV8461_ISTSL_167  = 0b10100110,      // 167/256 * 100
  DRV8461_ISTSL_166  = 0b10100101,      // 166/256 * 100
  DRV8461_ISTSL_165  = 0b10100100,      // 165/256 * 100
  DRV8461_ISTSL_164  = 0b10100011,      // 164/256 * 100
  DRV8461_ISTSL_163  = 0b10100010,      // 163/256 * 100
  DRV8461_ISTSL_162  = 0b10100001,      // 162/256 * 100
  DRV8461_ISTSL_161  = 0b10100000,      // 161/256 * 100
  DRV8461_ISTSL_160  = 0b10011111,      // 160/256 * 100
  DRV8461_ISTSL_159  = 0b10011110,      // 159/256 * 100
  DRV8461_ISTSL_158  = 0b10011101,      // 158/256 * 100
  DRV8461_ISTSL_157  = 0b10011100,      // 157/256 * 100
  DRV8461_ISTSL_156  = 0b10011011,      // 156/256 * 100
  DRV8461_ISTSL_155  = 0b10011010,      // 155/256 * 100
  DRV8461_ISTSL_154  = 0b10011001,      // 154/256 * 100
  DRV8461_ISTSL_153  = 0b10011000,      // 153/256 * 100
  DRV8461_ISTSL_152  = 0b10010111,      // 152/256 * 100
  DRV8461_ISTSL_151  = 0b10010110,      // 151/256 * 100
  DRV8461_ISTSL_150  = 0b10010101,      // 150/256 * 100
  DRV8461_ISTSL_149  = 0b10010100,      // 149/256 * 100
  DRV8461_ISTSL_148  = 0b10010011,      // 148/256 * 100
  DRV8461_ISTSL_147  = 0b10010010,      // 147/256 * 100
  DRV8461_ISTSL_146  = 0b10010001,      // 146/256 * 100
  DRV8461_ISTSL_145  = 0b10010000,      // 145/256 * 100
  DRV8461_ISTSL_144  = 0b10001111,      // 144/256 * 100
  DRV8461_ISTSL_143  = 0b10001110,      // 143/256 * 100
  DRV8461_ISTSL_142  = 0b10001101,      // 142/256 * 100
  DRV8461_ISTSL_141  = 0b10001100,      // 141/256 * 100
  DRV8461_ISTSL_140  = 0b10001011,      // 140/256 * 100
  DRV8461_ISTSL_139  = 0b10001010,      // 139/256 * 100
  DRV8461_ISTSL_138  = 0b10001001,      // 138/256 * 100
  DRV8461_ISTSL_137  = 0b10001000,      // 137/256 * 100
  DRV8461_ISTSL_136  = 0b10000111,      // 136/256 * 100
  DRV8461_ISTSL_135  = 0b10000110,      // 135/256 * 100
  DRV8461_ISTSL_134  = 0b10000101,      // 134/256 * 100
  DRV8461_ISTSL_133  = 0b10000100,      // 133/256 * 100
  DRV8461_ISTSL_132  = 0b10000011,      // 132/256 * 100
  DRV8461_ISTSL_131  = 0b10000010,      // 131/256 * 100
  DRV8461_ISTSL_130  = 0b10000001,      // 130/256 * 100
  DRV8461_ISTSL_129  = 0b10000000,      // 129/256 * 100
  DRV8461_ISTSL_128  = 0b01111111,      // 128/256 * 100
  DRV8461_ISTSL_127  = 0b01111110,      // 127/256 * 100
  DRV8461_ISTSL_126  = 0b01111101,      // 126/256 * 100
  DRV8461_ISTSL_125  = 0b01111100,      // 125/256 * 100
  DRV8461_ISTSL_124  = 0b01111011,      // 124/256 * 100
  DRV8461_ISTSL_123  = 0b01111010,      // 123/256 * 100
  DRV8461_ISTSL_122  = 0b01111001,      // 122/256 * 100
  DRV8461_ISTSL_121  = 0b01111000,      // 121/256 * 100
  DRV8461_ISTSL_120  = 0b01110111,      // 120/256 * 100
  DRV8461_ISTSL_119  = 0b01110110,      // 119/256 * 100
  DRV8461_ISTSL_118  = 0b01110101,      // 118/256 * 100
  DRV8461_ISTSL_117  = 0b01110100,      // 117/256 * 100
  DRV8461_ISTSL_116  = 0b01110011,      // 116/256 * 100
  DRV8461_ISTSL_115  = 0b01110010,      // 115/256 * 100
  DRV8461_ISTSL_114  = 0b01110001,      // 114/256 * 100
  DRV8461_ISTSL_113  = 0b01110000,      // 113/256 * 100
  DRV8461_ISTSL_112  = 0b01101111,      // 112/256 * 100
  DRV8461_ISTSL_111  = 0b01101110,      // 111/256 * 100
  DRV8461_ISTSL_110  = 0b01101101,      // 110/256 * 100
  DRV8461_ISTSL_109  = 0b01101100,      // 109/256 * 100
  DRV8461_ISTSL_108  = 0b01101011,      // 108/256 * 100
  DRV8461_ISTSL_107  = 0b01101010,      // 107/256 * 100
  DRV8461_ISTSL_106  = 0b01101001,      // 106/256 * 100
  DRV8461_ISTSL_105  = 0b01101000,      // 105/256 * 100
  DRV8461_ISTSL_104  = 0b01100111,      // 104/256 * 100
  DRV8461_ISTSL_103  = 0b01100110,      // 103/256 * 100
  DRV8461_ISTSL_102  = 0b01100101,      // 102/256 * 100
  DRV8461_ISTSL_101  = 0b01100100,      // 101/256 * 100
  DRV8461_ISTSL_100  = 0b01100011,      // 100/256 * 100
  DRV8461_ISTSL_99   = 0b01100010,      // 99/256 * 100
  DRV8461_ISTSL_98   = 0b01100001,      // 98/256 * 100
  DRV8461_ISTSL_97   = 0b01100000,      // 97/256 * 100
  DRV8461_ISTSL_96   = 0b01011111,      // 96/256 * 100
  DRV8461_ISTSL_95   = 0b01011110,      // 95/256 * 100
  DRV8461_ISTSL_94   = 0b01011101,      // 94/256 * 100
  DRV8461_ISTSL_93   = 0b01011100,      // 93/256 * 100
  DRV8461_ISTSL_92   = 0b01011011,      // 92/256 * 100
  DRV8461_ISTSL_91   = 0b01011010,      // 91/256 * 100
  DRV8461_ISTSL_90   = 0b01011001,      // 90/256 * 100
  DRV8461_ISTSL_89   = 0b01011000,      // 89/256 * 100
  DRV8461_ISTSL_88   = 0b01010111,      // 88/256 * 100
  DRV8461_ISTSL_87   = 0b01010110,      // 87/256 * 100
  DRV8461_ISTSL_86   = 0b01010101,      // 86/256 * 100
  DRV8461_ISTSL_85   = 0b01010100,      // 85/256 * 100
  DRV8461_ISTSL_84   = 0b01010011,      // 84/256 * 100
  DRV8461_ISTSL_83   = 0b01010010,      // 83/256 * 100
  DRV8461_ISTSL_82   = 0b01010001,      // 82/256 * 100
  DRV8461_ISTSL_81   = 0b01010000,      // 81/256 * 100
  DRV8461_ISTSL_80   = 0b01001111,      // 80/256 * 100
  DRV8461_ISTSL_79   = 0b01001110,      // 79/256 * 100
  DRV8461_ISTSL_78   = 0b01001101,      // 78/256 * 100
  DRV8461_ISTSL_77   = 0b01001100,      // 77/256 * 100
  DRV8461_ISTSL_76   = 0b01001011,      // 76/256 * 100
  DRV8461_ISTSL_75   = 0b01001010,      // 75/256 * 100
  DRV8461_ISTSL_74   = 0b01001001,      // 74/256 * 100
  DRV8461_ISTSL_73   = 0b01001000,      // 73/256 * 100
  DRV8461_ISTSL_72   = 0b01000111,      // 72/256 * 100
  DRV8461_ISTSL_71   = 0b01000110,      // 71/256 * 100
  DRV8461_ISTSL_70   = 0b01000101,      // 70/256 * 100
  DRV8461_ISTSL_69   = 0b01000100,      // 69/256 * 100
  DRV8461_ISTSL_68   = 0b01000011,      // 68/256 * 100
  DRV8461_ISTSL_67   = 0b01000010,      // 67/256 * 100
  DRV8461_ISTSL_66   = 0b01000001,      // 66/256 * 100
  DRV8461_ISTSL_65   = 0b01000000,      // 65/256 * 100
  DRV8461_ISTSL_64   = 0b00111111,      // 64/256 * 100
  DRV8461_ISTSL_63   = 0b00111110,      // 63/256 * 100
  DRV8461_ISTSL_62   = 0b00111101,      // 62/256 * 100
  DRV8461_ISTSL_61   = 0b00111100,      // 61/256 * 100
  DRV8461_ISTSL_60   = 0b00111011,      // 60/256 * 100
  DRV8461_ISTSL_59   = 0b00111010,      // 59/256 * 100
  DRV8461_ISTSL_58   = 0b00111001,      // 58/256 * 100
  DRV8461_ISTSL_57   = 0b00111000,      // 57/256 * 100
  DRV8461_ISTSL_56   = 0b00110111,      // 56/256 * 100
  DRV8461_ISTSL_55   = 0b00110110,      // 55/256 * 100
  DRV8461_ISTSL_54   = 0b00110101,      // 54/256 * 100
  DRV8461_ISTSL_53   = 0b00110100,      // 53/256 * 100
  DRV8461_ISTSL_52   = 0b00110011,      // 52/256 * 100
  DRV8461_ISTSL_51   = 0b00110010,      // 51/256 * 100
  DRV8461_ISTSL_50   = 0b00110001,      // 50/256 * 100
  DRV8461_ISTSL_49   = 0b00110000,      // 49/256 * 100
  DRV8461_ISTSL_48   = 0b00101111,      // 48/256 * 100
  DRV8461_ISTSL_47   = 0b00101110,      // 47/256 * 100
  DRV8461_ISTSL_46   = 0b00101101,      // 46/256 * 100
  DRV8461_ISTSL_45   = 0b00101100,      // 45/256 * 100
  DRV8461_ISTSL_44   = 0b00101011,      // 44/256 * 100
  DRV8461_ISTSL_43   = 0b00101010,      // 43/256 * 100
  DRV8461_ISTSL_42   = 0b00101001,      // 42/256 * 100
  DRV8461_ISTSL_41   = 0b00101000,      // 41/256 * 100
  DRV8461_ISTSL_40   = 0b00100111,      // 40/256 * 100
  DRV8461_ISTSL_39   = 0b00100110,      // 39/256 * 100
  DRV8461_ISTSL_38   = 0b00100101,      // 38/256 * 100
  DRV8461_ISTSL_37   = 0b00100100,      // 37/256 * 100
  DRV8461_ISTSL_36   = 0b00100011,      // 36/256 * 100
  DRV8461_ISTSL_35   = 0b00100010,      // 35/256 * 100
  DRV8461_ISTSL_34   = 0b00100001,      // 34/256 * 100
  DRV8461_ISTSL_33   = 0b00100000,      // 33/256 * 100
  DRV8461_ISTSL_32   = 0b00011111,      // 32/256 * 100
  DRV8461_ISTSL_31   = 0b00011110,      // 31/256 * 100
  DRV8461_ISTSL_30   = 0b00011101,      // 30/256 * 100
  DRV8461_ISTSL_29   = 0b00011100,      // 29/256 * 100
  DRV8461_ISTSL_28   = 0b00011011,      // 28/256 * 100
  DRV8461_ISTSL_27   = 0b00011010,      // 27/256 * 100
  DRV8461_ISTSL_26   = 0b00011001,      // 26/256 * 100
  DRV8461_ISTSL_25   = 0b00011000,      // 25/256 * 100
  DRV8461_ISTSL_24   = 0b00010111,      // 24/256 * 100
  DRV8461_ISTSL_23   = 0b00010110,      // 23/256 * 100
  DRV8461_ISTSL_22   = 0b00010101,      // 22/256 * 100
  DRV8461_ISTSL_21   = 0b00010100,      // 21/256 * 100
  DRV8461_ISTSL_20   = 0b00010011,      // 20/256 * 100
  DRV8461_ISTSL_19   = 0b00010010,      // 19/256 * 100
  DRV8461_ISTSL_18   = 0b00010001,      // 18/256 * 100
  DRV8461_ISTSL_17   = 0b00010000,      // 17/256 * 100
  DRV8461_ISTSL_16   = 0b00001111,      // 16/256 * 100
  DRV8461_ISTSL_15   = 0b00001110,      // 15/256 * 100
  DRV8461_ISTSL_14   = 0b00001101,      // 14/256 * 100
  DRV8461_ISTSL_13   = 0b00001100,      // 13/256 * 100
  DRV8461_ISTSL_12   = 0b00001011,      // 12/256 * 100
  DRV8461_ISTSL_11   = 0b00001010,      // 11/256 * 100
  DRV8461_ISTSL_10   = 0b00001001,      // 10/256 * 100
  DRV8461_ISTSL_9    = 0b00001000,      // 9/256 * 100
  DRV8461_ISTSL_8    = 0b00000111,      // 8/256 * 100
  DRV8461_ISTSL_7    = 0b00000110,      // 7/256 * 100
  DRV8461_ISTSL_6    = 0b00000101,      // 6/256 * 100
  DRV8461_ISTSL_5    = 0b00000100,      // 5/256 * 100
  DRV8461_ISTSL_4    = 0b00000011,      // 4/256 * 100
  DRV8461_ISTSL_3    = 0b00000010,      // 3/256 * 100
  DRV8461_ISTSL_2    = 0b00000001,      // 2/256 * 100
  DRV8461_ISTSL_1    = 0b00000000,      // 1/256 * 100
};


// CONTROL 11 REGISTER SETINGS ***********************************************************************************// 
enum class DRV8461_CTRL11_Reg_Val : uint8_t {
  DRV8461_CTRL11_TRQ_DAC = 0xFF,     // Determines the run current.
};

//Specific Values for Run Current
enum class DRV_Run_Current : uint8_t {
  DRV8461_TRQ_DAC_256  = 0b11111111,      // 256/256 * 100
  DRV8461_TRQ_DAC_255  = 0b11111110,      // 255/256 * 100
  DRV8461_TRQ_DAC_254  = 0b11111101,      // 254/256 * 100
  DRV8461_TRQ_DAC_253  = 0b11111100,      // 253/256 * 100
  DRV8461_TRQ_DAC_252  = 0b11111011,      // 252/256 * 100
  DRV8461_TRQ_DAC_251  = 0b11111010,      // 251/256 * 100
  DRV8461_TRQ_DAC_250  = 0b11111001,      // 250/256 * 100
  DRV8461_TRQ_DAC_249  = 0b11111000,      // 249/256 * 100
  DRV8461_TRQ_DAC_248  = 0b11110111,      // 248/256 * 100
  DRV8461_TRQ_DAC_247  = 0b11110110,      // 247/256 * 100
  DRV8461_TRQ_DAC_246  = 0b11110101,      // 246/256 * 100
  DRV8461_TRQ_DAC_245  = 0b11110100,      // 245/256 * 100
  DRV8461_TRQ_DAC_244  = 0b11110011,      // 244/256 * 100
  DRV8461_TRQ_DAC_243  = 0b11110010,      // 243/256 * 100
  DRV8461_TRQ_DAC_242  = 0b11110001,      // 242/256 * 100
  DRV8461_TRQ_DAC_241  = 0b11110000,      // 241/256 * 100
  DRV8461_TRQ_DAC_240  = 0b11101111,      // 240/256 * 100
  DRV8461_TRQ_DAC_239  = 0b11101110,      // 239/256 * 100
  DRV8461_TRQ_DAC_238  = 0b11101101,      // 238/256 * 100
  DRV8461_TRQ_DAC_237  = 0b11101100,      // 237/256 * 100
  DRV8461_TRQ_DAC_236  = 0b11101011,      // 236/256 * 100
  DRV8461_TRQ_DAC_235  = 0b11101010,      // 235/256 * 100
  DRV8461_TRQ_DAC_234  = 0b11101001,      // 234/256 * 100
  DRV8461_TRQ_DAC_233  = 0b11101000,      // 233/256 * 100
  DRV8461_TRQ_DAC_232  = 0b11100111,      // 232/256 * 100
  DRV8461_TRQ_DAC_231  = 0b11100110,      // 231/256 * 100
  DRV8461_TRQ_DAC_230  = 0b11100101,      // 230/256 * 100
  DRV8461_TRQ_DAC_229  = 0b11100100,      // 229/256 * 100
  DRV8461_TRQ_DAC_228  = 0b11100011,      // 228/256 * 100
  DRV8461_TRQ_DAC_227  = 0b11100010,      // 227/256 * 100
  DRV8461_TRQ_DAC_226  = 0b11100001,      // 226/256 * 100
  DRV8461_TRQ_DAC_225  = 0b11100000,      // 225/256 * 100
  DRV8461_TRQ_DAC_224  = 0b11011111,      // 224/256 * 100
  DRV8461_TRQ_DAC_223  = 0b11011110,      // 223/256 * 100
  DRV8461_TRQ_DAC_222  = 0b11011101,      // 222/256 * 100
  DRV8461_TRQ_DAC_221  = 0b11011100,      // 221/256 * 100
  DRV8461_TRQ_DAC_220  = 0b11011011,      // 220/256 * 100
  DRV8461_TRQ_DAC_219  = 0b11011010,      // 219/256 * 100
  DRV8461_TRQ_DAC_218  = 0b11011001,      // 218/256 * 100
  DRV8461_TRQ_DAC_217  = 0b11011000,      // 217/256 * 100
  DRV8461_TRQ_DAC_216  = 0b11010111,      // 216/256 * 100
  DRV8461_TRQ_DAC_215  = 0b11010110,      // 215/256 * 100
  DRV8461_TRQ_DAC_214  = 0b11010101,      // 214/256 * 100
  DRV8461_TRQ_DAC_213  = 0b11010100,      // 213/256 * 100
  DRV8461_TRQ_DAC_212  = 0b11010011,      // 212/256 * 100
  DRV8461_TRQ_DAC_211  = 0b11010010,      // 211/256 * 100
  DRV8461_TRQ_DAC_210  = 0b11010001,      // 210/256 * 100
  DRV8461_TRQ_DAC_209  = 0b11010000,      // 209/256 * 100
  DRV8461_TRQ_DAC_208  = 0b11001111,      // 208/256 * 100
  DRV8461_TRQ_DAC_207  = 0b11001110,      // 207/256 * 100
  DRV8461_TRQ_DAC_206  = 0b11001101,      // 206/256 * 100
  DRV8461_TRQ_DAC_205  = 0b11001100,      // 205/256 * 100
  DRV8461_TRQ_DAC_204  = 0b11001011,      // 204/256 * 100
  DRV8461_TRQ_DAC_203  = 0b11001010,      // 203/256 * 100
  DRV8461_TRQ_DAC_202  = 0b11001001,      // 202/256 * 100
  DRV8461_TRQ_DAC_201  = 0b11001000,      // 201/256 * 100
  DRV8461_TRQ_DAC_200  = 0b11000111,      // 200/256 * 100
  DRV8461_TRQ_DAC_199  = 0b11000110,      // 199/256 * 100
  DRV8461_TRQ_DAC_198  = 0b11000101,      // 198/256 * 100
  DRV8461_TRQ_DAC_197  = 0b11000100,      // 197/256 * 100
  DRV8461_TRQ_DAC_196  = 0b11000011,      // 196/256 * 100
  DRV8461_TRQ_DAC_195  = 0b11000010,      // 195/256 * 100
  DRV8461_TRQ_DAC_194  = 0b11000001,      // 194/256 * 100
  DRV8461_TRQ_DAC_193  = 0b11000000,      // 193/256 * 100
  DRV8461_TRQ_DAC_192  = 0b10111111,      // 192/256 * 100
  DRV8461_TRQ_DAC_191  = 0b10111110,      // 191/256 * 100
  DRV8461_TRQ_DAC_190  = 0b10111101,      // 190/256 * 100
  DRV8461_TRQ_DAC_189  = 0b10111100,      // 189/256 * 100
  DRV8461_TRQ_DAC_188  = 0b10111011,      // 188/256 * 100
  DRV8461_TRQ_DAC_187  = 0b10111010,      // 187/256 * 100
  DRV8461_TRQ_DAC_186  = 0b10111001,      // 186/256 * 100
  DRV8461_TRQ_DAC_185  = 0b10111000,      // 185/256 * 100
  DRV8461_TRQ_DAC_184  = 0b10110111,      // 184/256 * 100
  DRV8461_TRQ_DAC_183  = 0b10110110,      // 183/256 * 100
  DRV8461_TRQ_DAC_182  = 0b10110101,      // 182/256 * 100
  DRV8461_TRQ_DAC_181  = 0b10110100,      // 181/256 * 100
  DRV8461_TRQ_DAC_180  = 0b10110011,      // 180/256 * 100
  DRV8461_TRQ_DAC_179  = 0b10110010,      // 179/256 * 100
  DRV8461_TRQ_DAC_178  = 0b10110001,      // 178/256 * 100
  DRV8461_TRQ_DAC_177  = 0b10110000,      // 177/256 * 100
  DRV8461_TRQ_DAC_176  = 0b10101111,      // 176/256 * 100
  DRV8461_TRQ_DAC_175  = 0b10101110,      // 175/256 * 100
  DRV8461_TRQ_DAC_174  = 0b10101101,      // 174/256 * 100
  DRV8461_TRQ_DAC_173  = 0b10101100,      // 173/256 * 100
  DRV8461_TRQ_DAC_172  = 0b10101011,      // 172/256 * 100
  DRV8461_TRQ_DAC_171  = 0b10101010,      // 171/256 * 100
  DRV8461_TRQ_DAC_170  = 0b10101001,      // 170/256 * 100
  DRV8461_TRQ_DAC_169  = 0b10101000,      // 169/256 * 100
  DRV8461_TRQ_DAC_168  = 0b10100111,      // 168/256 * 100
  DRV8461_TRQ_DAC_167  = 0b10100110,      // 167/256 * 100
  DRV8461_TRQ_DAC_166  = 0b10100101,      // 166/256 * 100
  DRV8461_TRQ_DAC_165  = 0b10100100,      // 165/256 * 100
  DRV8461_TRQ_DAC_164  = 0b10100011,      // 164/256 * 100
  DRV8461_TRQ_DAC_163  = 0b10100010,      // 163/256 * 100
  DRV8461_TRQ_DAC_162  = 0b10100001,      // 162/256 * 100
  DRV8461_TRQ_DAC_161  = 0b10100000,      // 161/256 * 100
  DRV8461_TRQ_DAC_160  = 0b10011111,      // 160/256 * 100
  DRV8461_TRQ_DAC_159  = 0b10011110,      // 159/256 * 100
  DRV8461_TRQ_DAC_158  = 0b10011101,      // 158/256 * 100
  DRV8461_TRQ_DAC_157  = 0b10011100,      // 157/256 * 100
  DRV8461_TRQ_DAC_156  = 0b10011011,      // 156/256 * 100
  DRV8461_TRQ_DAC_155  = 0b10011010,      // 155/256 * 100
  DRV8461_TRQ_DAC_154  = 0b10011001,      // 154/256 * 100
  DRV8461_TRQ_DAC_153  = 0b10011000,      // 153/256 * 100
  DRV8461_TRQ_DAC_152  = 0b10010111,      // 152/256 * 100
  DRV8461_TRQ_DAC_151  = 0b10010110,      // 151/256 * 100
  DRV8461_TRQ_DAC_150  = 0b10010101,      // 150/256 * 100
  DRV8461_TRQ_DAC_149  = 0b10010100,      // 149/256 * 100
  DRV8461_TRQ_DAC_148  = 0b10010011,      // 148/256 * 100
  DRV8461_TRQ_DAC_147  = 0b10010010,      // 147/256 * 100
  DRV8461_TRQ_DAC_146  = 0b10010001,      // 146/256 * 100
  DRV8461_TRQ_DAC_145  = 0b10010000,      // 145/256 * 100
  DRV8461_TRQ_DAC_144  = 0b10001111,      // 144/256 * 100
  DRV8461_TRQ_DAC_143  = 0b10001110,      // 143/256 * 100
  DRV8461_TRQ_DAC_142  = 0b10001101,      // 142/256 * 100
  DRV8461_TRQ_DAC_141  = 0b10001100,      // 141/256 * 100
  DRV8461_TRQ_DAC_140  = 0b10001011,      // 140/256 * 100
  DRV8461_TRQ_DAC_139  = 0b10001010,      // 139/256 * 100
  DRV8461_TRQ_DAC_138  = 0b10001001,      // 138/256 * 100
  DRV8461_TRQ_DAC_137  = 0b10001000,      // 137/256 * 100
  DRV8461_TRQ_DAC_136  = 0b10000111,      // 136/256 * 100
  DRV8461_TRQ_DAC_135  = 0b10000110,      // 135/256 * 100
  DRV8461_TRQ_DAC_134  = 0b10000101,      // 134/256 * 100
  DRV8461_TRQ_DAC_133  = 0b10000100,      // 133/256 * 100
  DRV8461_TRQ_DAC_132  = 0b10000011,      // 132/256 * 100
  DRV8461_TRQ_DAC_131  = 0b10000010,      // 131/256 * 100
  DRV8461_TRQ_DAC_130  = 0b10000001,      // 130/256 * 100
  DRV8461_TRQ_DAC_129  = 0b10000000,      // 129/256 * 100
  DRV8461_TRQ_DAC_128  = 0b01111111,      // 128/256 * 100
  DRV8461_TRQ_DAC_127  = 0b01111110,      // 127/256 * 100
  DRV8461_TRQ_DAC_126  = 0b01111101,      // 126/256 * 100
  DRV8461_TRQ_DAC_125  = 0b01111100,      // 125/256 * 100
  DRV8461_TRQ_DAC_124  = 0b01111011,      // 124/256 * 100
  DRV8461_TRQ_DAC_123  = 0b01111010,      // 123/256 * 100
  DRV8461_TRQ_DAC_122  = 0b01111001,      // 122/256 * 100
  DRV8461_TRQ_DAC_121  = 0b01111000,      // 121/256 * 100
  DRV8461_TRQ_DAC_120  = 0b01110111,      // 120/256 * 100
  DRV8461_TRQ_DAC_119  = 0b01110110,      // 119/256 * 100
  DRV8461_TRQ_DAC_118  = 0b01110101,      // 118/256 * 100
  DRV8461_TRQ_DAC_117  = 0b01110100,      // 117/256 * 100
  DRV8461_TRQ_DAC_116  = 0b01110011,      // 116/256 * 100
  DRV8461_TRQ_DAC_115  = 0b01110010,      // 115/256 * 100
  DRV8461_TRQ_DAC_114  = 0b01110001,      // 114/256 * 100
  DRV8461_TRQ_DAC_113  = 0b01110000,      // 113/256 * 100
  DRV8461_TRQ_DAC_112  = 0b01101111,      // 112/256 * 100
  DRV8461_TRQ_DAC_111  = 0b01101110,      // 111/256 * 100
  DRV8461_TRQ_DAC_110  = 0b01101101,      // 110/256 * 100
  DRV8461_TRQ_DAC_109  = 0b01101100,      // 109/256 * 100
  DRV8461_TRQ_DAC_108  = 0b01101011,      // 108/256 * 100
  DRV8461_TRQ_DAC_107  = 0b01101010,      // 107/256 * 100
  DRV8461_TRQ_DAC_106  = 0b01101001,      // 106/256 * 100
  DRV8461_TRQ_DAC_105  = 0b01101000,      // 105/256 * 100
  DRV8461_TRQ_DAC_104  = 0b01100111,      // 104/256 * 100
  DRV8461_TRQ_DAC_103  = 0b01100110,      // 103/256 * 100
  DRV8461_TRQ_DAC_102  = 0b01100101,      // 102/256 * 100
  DRV8461_TRQ_DAC_101  = 0b01100100,      // 101/256 * 100
  DRV8461_TRQ_DAC_100  = 0b01100011,      // 100/256 * 100
  DRV8461_TRQ_DAC_99   = 0b01100010,      // 99/256 * 100
  DRV8461_TRQ_DAC_98   = 0b01100001,      // 98/256 * 100
  DRV8461_TRQ_DAC_97   = 0b01100000,      // 97/256 * 100
  DRV8461_TRQ_DAC_96   = 0b01011111,      // 96/256 * 100
  DRV8461_TRQ_DAC_95   = 0b01011110,      // 95/256 * 100
  DRV8461_TRQ_DAC_94   = 0b01011101,      // 94/256 * 100
  DRV8461_TRQ_DAC_93   = 0b01011100,      // 93/256 * 100
  DRV8461_TRQ_DAC_92   = 0b01011011,      // 92/256 * 100
  DRV8461_TRQ_DAC_91   = 0b01011010,      // 91/256 * 100
  DRV8461_TRQ_DAC_90   = 0b01011001,      // 90/256 * 100
  DRV8461_TRQ_DAC_89   = 0b01011000,      // 89/256 * 100
  DRV8461_TRQ_DAC_88   = 0b01010111,      // 88/256 * 100
  DRV8461_TRQ_DAC_87   = 0b01010110,      // 87/256 * 100
  DRV8461_TRQ_DAC_86   = 0b01010101,      // 86/256 * 100
  DRV8461_TRQ_DAC_85   = 0b01010100,      // 85/256 * 100
  DRV8461_TRQ_DAC_84   = 0b01010011,      // 84/256 * 100
  DRV8461_TRQ_DAC_83   = 0b01010010,      // 83/256 * 100
  DRV8461_TRQ_DAC_82   = 0b01010001,      // 82/256 * 100
  DRV8461_TRQ_DAC_81   = 0b01010000,      // 81/256 * 100
  DRV8461_TRQ_DAC_80   = 0b01001111,      // 80/256 * 100
  DRV8461_TRQ_DAC_79   = 0b01001110,      // 79/256 * 100
  DRV8461_TRQ_DAC_78   = 0b01001101,      // 78/256 * 100
  DRV8461_TRQ_DAC_77   = 0b01001100,      // 77/256 * 100
  DRV8461_TRQ_DAC_76   = 0b01001011,      // 76/256 * 100
  DRV8461_TRQ_DAC_75   = 0b01001010,      // 75/256 * 100
  DRV8461_TRQ_DAC_74   = 0b01001001,      // 74/256 * 100
  DRV8461_TRQ_DAC_73   = 0b01001000,      // 73/256 * 100
  DRV8461_TRQ_DAC_72   = 0b01000111,      // 72/256 * 100
  DRV8461_TRQ_DAC_71   = 0b01000110,      // 71/256 * 100
  DRV8461_TRQ_DAC_70   = 0b01000101,      // 70/256 * 100
  DRV8461_TRQ_DAC_69   = 0b01000100,      // 69/256 * 100
  DRV8461_TRQ_DAC_68   = 0b01000011,      // 68/256 * 100
  DRV8461_TRQ_DAC_67   = 0b01000010,      // 67/256 * 100
  DRV8461_TRQ_DAC_66   = 0b01000001,      // 66/256 * 100
  DRV8461_TRQ_DAC_65   = 0b01000000,      // 65/256 * 100
  DRV8461_TRQ_DAC_64   = 0b00111111,      // 64/256 * 100
  DRV8461_TRQ_DAC_63   = 0b00111110,      // 63/256 * 100
  DRV8461_TRQ_DAC_62   = 0b00111101,      // 62/256 * 100
  DRV8461_TRQ_DAC_61   = 0b00111100,      // 61/256 * 100
  DRV8461_TRQ_DAC_60   = 0b00111011,      // 60/256 * 100
  DRV8461_TRQ_DAC_59   = 0b00111010,      // 59/256 * 100
  DRV8461_TRQ_DAC_58   = 0b00111001,      // 58/256 * 100
  DRV8461_TRQ_DAC_57   = 0b00111000,      // 57/256 * 100
  DRV8461_TRQ_DAC_56   = 0b00110111,      // 56/256 * 100
  DRV8461_TRQ_DAC_55   = 0b00110110,      // 55/256 * 100
  DRV8461_TRQ_DAC_54   = 0b00110101,      // 54/256 * 100
  DRV8461_TRQ_DAC_53   = 0b00110100,      // 53/256 * 100
  DRV8461_TRQ_DAC_52   = 0b00110011,      // 52/256 * 100
  DRV8461_TRQ_DAC_51   = 0b00110010,      // 51/256 * 100
  DRV8461_TRQ_DAC_50   = 0b00110001,      // 50/256 * 100
  DRV8461_TRQ_DAC_49   = 0b00110000,      // 49/256 * 100
  DRV8461_TRQ_DAC_48   = 0b00101111,      // 48/256 * 100
  DRV8461_TRQ_DAC_47   = 0b00101110,      // 47/256 * 100
  DRV8461_TRQ_DAC_46   = 0b00101101,      // 46/256 * 100
  DRV8461_TRQ_DAC_45   = 0b00101100,      // 45/256 * 100
  DRV8461_TRQ_DAC_44   = 0b00101011,      // 44/256 * 100
  DRV8461_TRQ_DAC_43   = 0b00101010,      // 43/256 * 100
  DRV8461_TRQ_DAC_42   = 0b00101001,      // 42/256 * 100
  DRV8461_TRQ_DAC_41   = 0b00101000,      // 41/256 * 100
  DRV8461_TRQ_DAC_40   = 0b00100111,      // 40/256 * 100
  DRV8461_TRQ_DAC_39   = 0b00100110,      // 39/256 * 100
  DRV8461_TRQ_DAC_38   = 0b00100101,      // 38/256 * 100
  DRV8461_TRQ_DAC_37   = 0b00100100,      // 37/256 * 100
  DRV8461_TRQ_DAC_36   = 0b00100011,      // 36/256 * 100
  DRV8461_TRQ_DAC_35   = 0b00100010,      // 35/256 * 100
  DRV8461_TRQ_DAC_34   = 0b00100001,      // 34/256 * 100
  DRV8461_TRQ_DAC_33   = 0b00100000,      // 33/256 * 100
  DRV8461_TRQ_DAC_32   = 0b00011111,      // 32/256 * 100
  DRV8461_TRQ_DAC_31   = 0b00011110,      // 31/256 * 100
  DRV8461_TRQ_DAC_30   = 0b00011101,      // 30/256 * 100
  DRV8461_TRQ_DAC_29   = 0b00011100,      // 29/256 * 100
  DRV8461_TRQ_DAC_28   = 0b00011011,      // 28/256 * 100
  DRV8461_TRQ_DAC_27   = 0b00011010,      // 27/256 * 100
  DRV8461_TRQ_DAC_26   = 0b00011001,      // 26/256 * 100
  DRV8461_TRQ_DAC_25   = 0b00011000,      // 25/256 * 100
  DRV8461_TRQ_DAC_24   = 0b00010111,      // 24/256 * 100
  DRV8461_TRQ_DAC_23   = 0b00010110,      // 23/256 * 100
  DRV8461_TRQ_DAC_22   = 0b00010101,      // 22/256 * 100
  DRV8461_TRQ_DAC_21   = 0b00010100,      // 21/256 * 100
  DRV8461_TRQ_DAC_20   = 0b00010011,      // 20/256 * 100
  DRV8461_TRQ_DAC_19   = 0b00010010,      // 19/256 * 100
  DRV8461_TRQ_DAC_18   = 0b00010001,      // 18/256 * 100
  DRV8461_TRQ_DAC_17   = 0b00010000,      // 17/256 * 100
  DRV8461_TRQ_DAC_16   = 0b00001111,      // 16/256 * 100
  DRV8461_TRQ_DAC_15   = 0b00001110,      // 15/256 * 100
  DRV8461_TRQ_DAC_14   = 0b00001101,      // 14/256 * 100
  DRV8461_TRQ_DAC_13   = 0b00001100,      // 13/256 * 100
  DRV8461_TRQ_DAC_12   = 0b00001011,      // 12/256 * 100
  DRV8461_TRQ_DAC_11   = 0b00001010,      // 11/256 * 100
  DRV8461_TRQ_DAC_10   = 0b00001001,      // 10/256 * 100
  DRV8461_TRQ_DAC_9    = 0b00001000,      // 9/256 * 100
  DRV8461_TRQ_DAC_8    = 0b00000111,      // 8/256 * 100
  DRV8461_TRQ_DAC_7    = 0b00000110,      // 7/256 * 100
  DRV8461_TRQ_DAC_6    = 0b00000101,      // 6/256 * 100
  DRV8461_TRQ_DAC_5    = 0b00000100,      // 5/256 * 100
  DRV8461_TRQ_DAC_4    = 0b00000011,      // 4/256 * 100
  DRV8461_TRQ_DAC_3    = 0b00000010,      // 3/256 * 100
  DRV8461_TRQ_DAC_2    = 0b00000001,      // 2/256 * 100
  DRV8461_TRQ_DAC_1    = 0b00000000,      // 1/256 * 100
};

// CONTROL 12 REGISTER SETINGS ***********************************************************************************// 
enum class DRV8461_CTRL12_Reg_Val : uint8_t {
  DRV8461_CTRL12_EN_STSL    = 0x80,     // Standstill power saving mode. (0 = disabled) default 0.
  DRV8461_CTRL12_TSTSL_FALL = 0x78,     // Controls the time for current to reduce to ISTSL from TRQ_DAC after TSTSL_DLY elapsed
};

//Specific Values for the time it takes for Current to Reduce
enum class DRV8461_Current_Reduction_Time : uint8_t {
  DRV8461_TSTSL_FALL_15   = 0b1111,      // 15ms
  DRV8461_TSTSL_FALL_14   = 0b1110,      // 14ms
  DRV8461_TSTSL_FALL_13   = 0b1101,      // 13ms
  DRV8461_TSTSL_FALL_12   = 0b1100,      // 12ms
  DRV8461_TSTSL_FALL_11   = 0b1011,      // 11ms
  DRV8461_TSTSL_FALL_10   = 0b1010,      // 10ms
  DRV8461_TSTSL_FALL_9    = 0b1001,      // 9ms
  DRV8461_TSTSL_FALL_8    = 0b1000,      // 8ms
  DRV8461_TSTSL_FALL_7    = 0b0111,      // 7ms
  DRV8461_TSTSL_FALL_6    = 0b0110,      // 6ms
  DRV8461_TSTSL_FALL_5    = 0b0101,      // 5ms
  DRV8461_TSTSL_FALL_4    = 0b0100,      // 4ms
  DRV8461_TSTSL_FALL_3    = 0b0011,      // 3ms
  DRV8461_TSTSL_FALL_2    = 0b0010,      // 2ms
  DRV8461_TSTSL_FALL_1    = 0b0001,      // 1ms
};


// CONTROL 13 REGISTER SETINGS ***********************************************************************************// 
enum class DRV8461_CTRL13_Reg_Val : uint8_t {
  DRV8461_CTRL13_TSTSL_DLY   = 0xFC,       // Controls the delay between last STEP and standstill activation.
  DRV8461_CTRL13_VREF_INT_EN = 0x02,       // When 1, uses 3.3V for current regulation, ignoring VREF. Default 0.
};

//Spefic Values for Delay Until Standstill
enum class DRV8461_Delay_Until_Standstill : uint8_t {
  DRV8461_TSTSL_FALL_1008  = 0b111111,      // 1008ms
  DRV8461_TSTSL_FALL_992   = 0b111110,      // 992ms
  DRV8461_TSTSL_FALL_976   = 0b111101,      // 976ms
  DRV8461_TSTSL_FALL_960   = 0b111100,      // 960ms
  DRV8461_TSTSL_FALL_944   = 0b111011,      // 944ms
  DRV8461_TSTSL_FALL_928   = 0b111010,      // 928ms
  DRV8461_TSTSL_FALL_912   = 0b111001,      // 912ms
  DRV8461_TSTSL_FALL_896   = 0b111000,      // 896ms
  DRV8461_TSTSL_FALL_880   = 0b110111,      // 880ms
  DRV8461_TSTSL_FALL_864   = 0b110110,      // 864ms
  DRV8461_TSTSL_FALL_848   = 0b110101,      // 848ms
  DRV8461_TSTSL_FALL_832   = 0b110100,      // 832ms
  DRV8461_TSTSL_FALL_816   = 0b110011,      // 816ms
  DRV8461_TSTSL_FALL_800   = 0b110010,      // 800ms
  DRV8461_TSTSL_FALL_784   = 0b110001,      // 784ms
  DRV8461_TSTSL_FALL_768   = 0b110000,      // 768ms
  DRV8461_TSTSL_FALL_752   = 0b101111,      // 752ms
  DRV8461_TSTSL_FALL_736   = 0b101110,      // 736ms
  DRV8461_TSTSL_FALL_720   = 0b101101,      // 720ms
  DRV8461_TSTSL_FALL_704   = 0b101100,      // 704ms
  DRV8461_TSTSL_FALL_688   = 0b101011,      // 688ms
  DRV8461_TSTSL_FALL_672   = 0b101010,      // 672ms
  DRV8461_TSTSL_FALL_656   = 0b101001,      // 656ms
  DRV8461_TSTSL_FALL_640   = 0b101000,      // 640ms
  DRV8461_TSTSL_FALL_624   = 0b100111,      // 624ms
  DRV8461_TSTSL_FALL_608   = 0b100110,      // 608ms
  DRV8461_TSTSL_FALL_592   = 0b100101,      // 592ms
  DRV8461_TSTSL_FALL_576   = 0b100100,      // 576ms
  DRV8461_TSTSL_FALL_560   = 0b100011,      // 560ms
  DRV8461_TSTSL_FALL_544   = 0b100010,      // 544ms
  DRV8461_TSTSL_FALL_528   = 0b100001,      // 528ms
  DRV8461_TSTSL_FALL_512   = 0b100000,      // 512ms
  DRV8461_TSTSL_FALL_496   = 0b011111,      // 496ms
  DRV8461_TSTSL_FALL_480   = 0b011110,      // 480ms
  DRV8461_TSTSL_FALL_464   = 0b011101,      // 464ms
  DRV8461_TSTSL_FALL_448   = 0b011100,      // 448ms
  DRV8461_TSTSL_FALL_432   = 0b011011,      // 432ms
  DRV8461_TSTSL_FALL_416   = 0b011010,      // 416ms
  DRV8461_TSTSL_FALL_400   = 0b011001,      // 400ms
  DRV8461_TSTSL_FALL_384   = 0b011000,      // 384ms
  DRV8461_TSTSL_FALL_368   = 0b010111,      // 368ms
  DRV8461_TSTSL_FALL_352   = 0b010110,      // 352ms
  DRV8461_TSTSL_FALL_336   = 0b010101,      // 336ms
  DRV8461_TSTSL_FALL_320   = 0b010100,      // 320ms
  DRV8461_TSTSL_FALL_304   = 0b010011,      // 304ms
  DRV8461_TSTSL_FALL_288   = 0b010010,      // 288ms
  DRV8461_TSTSL_FALL_272   = 0b010001,      // 272ms
  DRV8461_TSTSL_FALL_256   = 0b010000,      // 256ms
  DRV8461_TSTSL_FALL_240   = 0b001111,      // 240ms
  DRV8461_TSTSL_FALL_224   = 0b001110,      // 224ms
  DRV8461_TSTSL_FALL_208   = 0b001101,      // 208ms
  DRV8461_TSTSL_FALL_192   = 0b001100,      // 192ms
  DRV8461_TSTSL_FALL_176   = 0b001011,      // 176ms
  DRV8461_TSTSL_FALL_160   = 0b001010,      // 160ms
  DRV8461_TSTSL_FALL_144   = 0b001001,      // 144ms
  DRV8461_TSTSL_FALL_128   = 0b001000,      // 128ms
  DRV8461_TSTSL_FALL_112   = 0b000111,      // 112ms
  DRV8461_TSTSL_FALL_96    = 0b000110,      // 96ms
  DRV8461_TSTSL_FALL_80    = 0b000101,      // 80ms
  DRV8461_TSTSL_FALL_64    = 0b000100,      // 64ms
  DRV8461_TSTSL_FALL_48    = 0b000011,      // 48ms
  DRV8461_TSTSL_FALL_32    = 0b000010,      // 32ms
  DRV8461_TSTSL_FALL_16    = 0b000001,      // 16ms
};


// CONTROL 14 REGISTER SETINGS ***********************************************************************************// 
enum class DRV8461_CTRL14_Reg_Val : uint8_t {
  DRV8461_CTRL14_VM_ADC = 0xF8,       // Outputs the value of the supply voltage
};

//Specificc Values for Supply Voltage
enum class DRV8461_Supply_Voltage : uint8_t {
  DRV8461_VM_ADC_0  = 0b00000,         // Supply Voltage 0V
  DRV8461_VM_ADC_24 = 0b01011,         // Supply Voltage 24V
  DRV8461_VM_ADC_65 = 0b11111,         // Supply Voltage 65V
};


/// This class provides high-level functions for controlling a DRV8461, labelled 8434S stepper
/// motor driver.
class DRV8434S
{
public:
  /// The default constructor.
  DRV8434S()   //Purpose Unknown
  {
    // All settings set to power-on defaults
    ctrl1  = 0x0F;
    ctrl2  = 0x06;
    ctrl3  = 0x38;
    ctrl4  = 0x49;
    ctrl5  = 0x03;
    ctrl6  = 0x20;
    ctrl9  = 0x10;
    ctrl10 = 0x80;
    ctrl11 = 0xFF;
    ctrl12 = 0x20;
    ctrl13 = 0x10;
    ctrl14 = 0x58;
  }

  /// Configures this object to use the specified pin as a chip select pin.
  /// You must use a chip select pin; the DRV8711 requires it.
  void setChipSelectPin(uint8_t pin)
  {
    driver.setChipSelectPin(pin);
  }

  /// Changes all of the driver's settings back to their default values.
  ///
  /// It is good to call this near the beginning of your program to ensure that
  /// there are no settings left over from an earlier time that might affect the
  /// operation of the driver.
  void resetSettings()
  {
    ctrl1  = 0x0F;
    ctrl2  = 0x06;
    ctrl3  = 0x38;
    ctrl4  = 0x49;
    ctrl5  = 0x03;
    ctrl6  = 0x20;
    ctrl9  = 0x10;
    ctrl10 = 0x80;
    ctrl11 = 0xFF;
    ctrl12 = 0x20;
    ctrl13 = 0x10;
    ctrl14 = 0x58;


    applySettings();
  }

  /// Reads back the SPI configuration registers from the device and verifies
  /// that they are equal to the cached copies stored in this class.
  ///
  /// This can be used to verify that the driver is powered on and has not lost
  /// them due to a power failure.  The STATUS register is not verified because
  /// it does not contain any driver settings.
  ///
  /// @return 1 if the settings from the device match the cached copies, 0 if
  /// they do not.
  bool verifySettings()
  {
    return driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL1)  == ctrl1  &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL2)  == ctrl2  &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL3)  == ctrl3  &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL4)  == ctrl4  &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL5)  == ctrl5  &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL6)  == ctrl6  &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL9)  == ctrl9  &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL10) == ctrl10 &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL11) == ctrl11 &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL12) == ctrl12 &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL13) == ctrl13 &&
           driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL14) == ctrl14;
  }

  /// Re-writes the cached settings stored in this class to the device.
  ///
  /// You should not normally need to call this function because settings are
  /// written to the device whenever they are changed.  However, if
  /// verifySettings() returns false (due to a power interruption, for
  /// instance), then you could use applySettings() to get the device's settings
  /// back into the desired state.
  void applySettings()
  {
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL2);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL3);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL4);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL5);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL6);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL9);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL10);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL11);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL12);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL13);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL14);

    // CTRL1 is written last because it contains the EN_OUT bit, and we want to
    // try to have all the other settings correct first.
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL1);
  }

  /// Sets the driver's current scalar (TRQ_DAC), which scales the full current
  /// limit (as set by VREF) by the specified percentage. The available settings
  /// are multiples of 0.390625%.
  ///
  /// This function takes an integer, and if the desired current limit is not
  /// available, it generally tries to pick the closest current limit that is
  /// lower than the desired one (although the lowest possible setting is
  /// 6.25%). However, it will round up if the next setting is no more than
  /// 0.75% higher; this allows you to specify 43.75% by passing a value of 43,
  /// for example.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// // This sets TRQ_DAC to 37.5% (the closest setting lower than 42%):
  /// sd.setCurrentPercent(42);
  ///
  /// // This sets TRQ_DAC to 43.75% (rounding 43 up by 0.75% to 43.75%):
  /// sd.setCurrentPercent(43);
  ///
  /// // This also sets TRQ_DAC to 43.75%; even though the argument is truncated
  /// // to an integer (43), that is then rounded up by 0.75% to 43.75%:
  /// sd.setCurrentPercent(43.75);
  /// ~~~
  void setCurrentPercent(uint8_t percent)
  {
    if (percent > 100) { percent = 100; }
    if (percent < 1) { percent = 1; }

    uint8_t td = ((uint16_t)percent * 64 / 25) - 1; // convert 0-100 to 0-255
    if (td == 0) { td = 1; }                   // restrict to 1-16
    ctrl11 = td;
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL11);
  }

  /// Sets the driver's current scalar (TRQ_DAC) to produce the specified scaled
  /// current limit in milliamps. In order to calculate the correct value for
  /// TRQ_DAC, this function also needs to know the full current limit set by
  /// VREF (i.e. what the current limit is when the scaling is set to 100%).
  /// This is specified by the optional `fullCurrent` argument, which defaults
  /// to 2000 milliamps (2 A).
  ///
  /// If the desired current limit is not
  /// available, this function tries to pick the closest current limit that is
  /// lower than the desired one (although the lowest possible setting is 6.25%
  /// of the full current limit).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// // This specifies that we want a scaled current limit of 1200 mA and that
  /// // VREF is set to produce a full current limit of 1500 mA. TRQ_DAC will be
  /// // set to 75%, which will produce a 1125 mA scaled current limit.
  /// sd.setCurrentMilliamps(1200, 1500);
  /// ~~~
  void setCurrentMilliamps(uint16_t current, uint16_t fullCurrent = 2000)
  {
    if (fullCurrent > 4000) { fullCurrent = 4000; }
    if (current > fullCurrent) { current = fullCurrent; }

    uint8_t td = (current * 256 / fullCurrent); // convert 0-fullCurrent to 0-16
    if (td == 0) { td = 1; }                   // restrict to 1-16
    ctrl11 = td;
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL11);
  }

  /// Enables the driver (EN_OUT = 1).
  void enableDriver()
  {
    ctrl1 |= (1 << 7);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL1);
  }

  /// Disables the driver (EN_OUT = 0).
  void disableDriver()
  {
    ctrl1 &= ~(1 << 7);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL1);
  }

  /// Sets the driver's decay mode (DECAY).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setDecayMode(DRV8434SDecayMode::DRV8461_DECAY_SLOW_SLOW);
  /// ~~~
  void setDecayMode(DRV8461_Decay_Mode mode)
  {
    ctrl1 = (ctrl1 & 0b11111000) | ((uint8_t)mode & 0b111);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL1);
  }

  /// Sets the motor direction (DIR).
  ///
  /// Allowed values are 0 or 1.
  ///
  /// You must first call enableSPIDirection() to allow the direction to be
  /// controlled through SPI.  Once you have done so, you can use this command
  /// to control the direction of the stepper motor and leave the DIR pin
  /// disconnected.
  void setDirection(bool value)
  {
    if (value)
    {
      ctrl2 |= (1 << 7);
    }
    else
    {
      ctrl2 &= ~(1 << 7);
    }
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL2);
  }

  /// Returns the cached value of the motor direction (DIR).
  ///
  /// This does not perform any SPI communication with the driver.
  bool getDirection()
  {
    return (ctrl2 >> 7) & 1;
  }

  /// Advances the indexer by one step (STEP = 1).
  ///
  /// You must first call enableSPIStep() to allow stepping to be controlled
  /// through SPI.  Once you have done so, you can use this command to step the
  /// motor and leave the STEP pin disconnected.
  ///
  /// The driver automatically clears the STEP bit after it is written.
  void step()
  {
    driver.writeReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL2, ctrl2 | (1 << 6));
  }

  /// Enables direction control through SPI (SPI_DIR = 1), allowing
  /// setDirection() to override the DIR pin.
  void enableSPIDirection()
  {
    ctrl2 |= (1 << 5);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL2);
  }

  /// Disables direction control through SPI (SPI_DIR = 0), making the DIR pin
  /// control direction instead.
  void disableSPIDirection()
  {
    ctrl2 &= ~(1 << 5);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL2);
  }

  /// Enables stepping through SPI (SPI_STEP = 1), allowing step() to override
  /// the STEP pin.
  void enableSPIStep()
  {
    ctrl2 |= (1 << 4);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL2);
  }

  /// Disables stepping through SPI (SPI_STEP = 0), making the STEP pin control
  /// stepping instead.
  void disableSPIStep()
  {
    ctrl2 &= ~(1 << 4);
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL2);
  }

  /// Sets the driver's stepping mode (MICROSTEP_MODE).
  ///
  /// This affects many things about the performance of the motor, including how
  /// much the output moves for each step taken and how much current flows
  /// through the coils in each stepping position.
  ///
  /// If an invalid stepping mode is passed to this function, then it selects
  /// 1/16 micro-step, which is the driver's default.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setStepMode(DRV8434SStepMode::MicroStep32);
  /// ~~~
  void setStepMode(DRV8461_Micostep_Mode mode)
  {
    if (mode > DRV8461_Micostep_Mode::DRV8461_MICROSTEP_256)
    {
      // Invalid mode; pick 1/16 micro-step by default.
      mode = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_16;
    }

    ctrl2 = (ctrl2 & 0b11110000) | (uint8_t)mode;
    writeCachedReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL2);
  }

  /// Sets the driver's stepping mode (MICROSTEP_MODE).
  ///
  /// This version of the function allows you to express the requested
  /// microstepping ratio as a number directly.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setStepMode(32);
  /// ~~~
  int setStepMode(uint16_t mode)
  {
    DRV8461_Micostep_Mode sm;

    switch (mode)
    {
      case 1:   sm = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_1_71; break;
      case 2:   sm = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_2;    break;
      case 4:   sm = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_4;    break;
      case 8:   sm = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_8;    break;
      case 16:  sm = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_16;   break;
      case 32:  sm = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_32;   break;
      case 64:  sm = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_64;   break;
      case 128: sm = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_128;  break;
      case 256: sm = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_256;  break;

      // Invalid mode; pick 1/16 micro-step by default, returns 0 for error checking.
      default:  sm = DRV8461_Micostep_Mode::DRV8461_MICROSTEP_16; return(sm, 1);
    }

    setStepMode(sm, 0);
  }

  /// Reads the FAULT status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// FAULT condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8434SFaultBit enum to check individual bits.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// if (sd.readFault() & (1 << (uint8_t)DRV8434SFaultBit::UVLO))
  /// {
  ///   // Supply undervoltage lockout is active.
  /// }
  /// ~~~
  uint8_t readFault()
  {
    return driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_FAULT);
  }

  /// Reads the DIAG1 status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// DIAG1 condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8434SDiag1Bit enum to check individual bits.
  uint8_t readDiag1()
  {
    return driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_DIAG1);
  }

  /// Reads the DIAG2 status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// DIAG2 condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8434SDiag2Bit enum to check individual bits.
  uint8_t readDiag2()
  {
    return driver.readReg(DRV8461_REG_ADDR::DRV8461_REG_DIAG2);
  }

  /// Clears any fault conditions that are currently latched in the driver
  /// (CLR_FLT = 1).
  ///
  /// WARNING: Calling this function clears latched faults, which might allow
  /// the motor driver outputs to reactivate.  If you do this repeatedly without
  /// fixing an abnormal condition (like a short circuit), you might damage the
  /// driver.
  ///
  /// The driver automatically clears the CLR_FLT bit after it is written.
  void clearFaults()
  {
    driver.writeReg(DRV8461_REG_ADDR::DRV8461_REG_CTRL3, ctrl3 | (1 << 7));
  }

  /// Gets the cached value of a register. If the given register address is not
  /// valid, this function returns 0.
  uint8_t getCachedReg(DRV8461_REG_ADDR address)
  {
    uint8_t * cachedReg = cachedRegPtr(address);
    if (!cachedReg) { return 0; }
    return *cachedReg;
  }

  /// Writes the specified value to a register after updating the cached value
  /// to match.
  ///
  /// Using this function keeps this object's cached settings consistent with
  /// the settings being written to the driver, so if you are using
  /// verifySettings(), applySettings(), and/or any of the other functions for
  /// specific settings that this library provides, you should use this function
  /// for direct register accesses instead of calling DRV8434SSPI::writeReg()
  /// directly.
  void setReg(DRV8461_REG_ADDR address, uint8_t value)
  {
    uint8_t * cachedReg = cachedRegPtr(address);
    if (!cachedReg) { return; }
    *cachedReg = value;
    driver.writeReg(address, value);
  }

protected:

  uint8_t ctrl1, ctrl2, ctrl3, ctrl4, ctrl5, ctrl6, ctrl7, ctrl8, ctrl9, ctrl10, ctrl11, ctrl12, ctrl13, ctrl14, index1, index2, index3, index4, index5, custctrl1, custctrl2, custctrl3, custctrl4, custctrl5, custctrl6, custctrl7, custctrl8, custctrl9, atqctrl1, atqctrl2, atqctrl3, atqctrl4, atqctrl5, atqctrl6, atqctrl7, atqctrl8, atqctrl9, atqctrl10, atqctrl11, atqctrl12, atqctrl13, atqctrl14, atqctrl15, atqctrl16, atqctrl17, atqctrl18, ssctrl1, ssctrl2, ssctrl3, ssctrl4, ssctrl5;

  /// Returns a pointer to the variable containing the cached value for the
  /// given register.
  uint8_t * cachedRegPtr(DRV8461_REG_ADDR address)
  {
    switch (address)
    {
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL1: return &ctrl1;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL2: return &ctrl2;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL3: return &ctrl3;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL4: return &ctrl4;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL5: return &ctrl5;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL6: return &ctrl6;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL7: return &ctrl7;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL8: return &ctrl8;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL9: return &ctrl9;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL10: return &ctrl10;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL11: return &ctrl11;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL12: return &ctrl12;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL13: return &ctrl13;
      case DRV8461_REG_ADDR::DRV8461_REG_CTRL14: return &ctrl14;
      case DRV8461_REG_ADDR::DRV8461_REG_INDEX1: return &index1;
      case DRV8461_REG_ADDR::DRV8461_REG_INDEX2: return &index2;
      case DRV8461_REG_ADDR::DRV8461_REG_INDEX3: return &index3;
      case DRV8461_REG_ADDR::DRV8461_REG_INDEX4: return &index4;
      case DRV8461_REG_ADDR::DRV8461_REG_INDEX5: return &index5;
      case DRV8461_REG_ADDR::DRV8461_REG_CUSTOM_CTRL1: return &custctrl1;
      case DRV8461_REG_ADDR::DRV8461_REG_CUSTOM_CTRL2: return &custctrl2;
      case DRV8461_REG_ADDR::DRV8461_REG_CUSTOM_CTRL3: return &custctrl3;
      case DRV8461_REG_ADDR::DRV8461_REG_CUSTOM_CTRL4: return &custctrl4;
      case DRV8461_REG_ADDR::DRV8461_REG_CUSTOM_CTRL5: return &custctrl5;
      case DRV8461_REG_ADDR::DRV8461_REG_CUSTOM_CTRL6: return &custctrl6;
      case DRV8461_REG_ADDR::DRV8461_REG_CUSTOM_CTRL7: return &custctrl7;
      case DRV8461_REG_ADDR::DRV8461_REG_CUSTOM_CTRL8: return &custctrl8;
      case DRV8461_REG_ADDR::DRV8461_REG_CUSTOM_CTRL9: return &custctrl9;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL1: return &atqctrl1;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL2: return &atqctrl2;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL3: return &atqctrl3;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL4: return &atqctrl4;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL5: return &atqctrl5;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL6: return &atqctrl6;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL7: return &atqctrl7;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL8: return &atqctrl8;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL9: return &atqctrl9;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL11: return &atqctrl11;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL12: return &atqctrl12;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL13: return &atqctrl13;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL14: return &atqctrl14;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL15: return &atqctrl15;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL16: return &atqctrl16;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL17: return &atqctrl17;
      case DRV8461_REG_ADDR::DRV8461_REG_ATQ_CTRL18: return &atqctrl18;
      case DRV8461_REG_ADDR::DRV8461_REG_SS_CTRL1: return &ssctrl1;
      case DRV8461_REG_ADDR::DRV8461_REG_SS_CTRL2: return &ssctrl2;
      case DRV8461_REG_ADDR::DRV8461_REG_SS_CTRL3: return &ssctrl3;
      case DRV8461_REG_ADDR::DRV8461_REG_SS_CTRL4: return &ssctrl4;
      case DRV8461_REG_ADDR::DRV8461_REG_SS_CTRL5: return &ssctrl5;
  
      default: return nullptr;
    }
  }

  /// Writes the cached value of the given register to the device.
  void writeCachedReg(DRV8461_REG_ADDR address)
  {
    uint8_t * cachedReg = cachedRegPtr(address);
    if (!cachedReg) { return; }
    driver.writeReg(address, *cachedReg);
  }

public:
  /// This object handles all the communication with the DRV8711.  Generally,
  /// you should not need to use it in your code for basic usage of a
  /// High-Power Stepper Motor Driver, but you might want to use it to access
  /// more advanced settings that the HighPowerStepperDriver class does not
  /// provide functions for.
  DRV8434SSPI driver;
};


#endif                                    // #ifndef DRV8461_REGISTERS_H

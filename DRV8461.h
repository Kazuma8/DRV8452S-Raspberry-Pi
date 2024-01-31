// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/

/// \file DRV8461.h
///
/// This is the main header file for the DRV8461 library,
/// a library for controlling the DRV8461 stepper motor driver.
///
/// For more information about this library, see:
///
///   https://github.com/pololu/DRV8461-arduino
///
/// That is the main repository for this library.

#pragma once

#include <Arduino.h>
#include <SPI.h>

/// Addresses of all registers.
enum class DRV8461RegAddr : uint8_t
{
  FAULT        = 0x00             // Fault status register.
  DIAG1        = 0x01             // DIAG status 1.
  DIAG2        = 0x02             // DIAG status 2.
  DIAG3        = 0x03             // DIAG status 3.
  CTRL1        = 0x04             // Control register 1.
  CTRL2        = 0x05             // Control register 2.
  CTRL3        = 0x06             // Control register 3.
  CTRL4        = 0x07             // Control register 4.
  CTRL5        = 0x08             // Control register 5.
  CTRL6        = 0x09             // Control register 6.
  CTRL7        = 0x0A             // Control register 7.
  CTRL8        = 0x0B             // Control register 8.
  CTRL9        = 0x0C             // Control register 9.
  CTRL10       = 0x0D             // Control Register 10.
  CTRL11       = 0x0E             // Control Register 11.
  CTRL12       = 0x0F             // Control Register 12.
  CTRL13       = 0x10             // Control Register 13.
  INDEX1       = 0x11             // Index Register 1.
  INDEX2       = 0x12             // Index Register 2.
  INDEX3       = 0x13             // Index Register 3.
  INDEX4       = 0x14             // Index Register 4.
  INDEX5       = 0x15             // Index Register 5.
  CUSTOM_CTRL1 = 0x16             // Custom Microstep Register 1.
  CUSTOM_CTRL2 = 0x17             // Custom Microstep Register 2
  CUSTOM_CTRL3 = 0x18             // Custom Microstep Register 3
  CUSTOM_CTRL4 = 0x19             // Custom Microstep Register 4
  CUSTOM_CTRL5 = 0x1A             // Custom Microstep Register 5
  CUSTOM_CTRL6 = 0x1B             // Custom Microstep Register 6
  CUSTOM_CTRL7 = 0x1C             // Custom Microstep Register 7
  CUSTOM_CTRL8 = 0x1D             // Custom Microstep Register 8
  CUSTOM_CTRL9 = 0x1E             // Custom Microstep Register 9
  ATQ_CTRL1    = 0x1F             // Auto Torque Register 1.
  ATQ_CTRL2    = 0x20             // Auto Torque Register 2.
  ATQ_CTRL3    = 0x21             // Auto Torque Register 3.
  ATQ_CTRL4    = 0x22             // Auto Torque Register 4.
  ATQ_CTRL5    = 0x23             // Auto Torque Register 5.
  ATQ_CTRL6    = 0x24             // Auto Torque Register 6.
  ATQ_CTRL7    = 0x25             // Auto Torque Register 7.
  ATQ_CTRL8    = 0x26             // Auto Torque Register 8.
  ATQ_CTRL9    = 0x27             // Auto Torque Register 9.
  ATQ_CTRL10   = 0x28             // Auto Torque Register 10.
  ATQ_CTRL11   = 0x29             // Auto Torque Register 11.
  ATQ_CTRL12   = 0x2A             // Auto Torque Register 12.
  ATQ_CTRL13   = 0x2B             // Auto Torque Register 13.
  ATQ_CTRL14   = 0x2C             // Auto Torque Register 14.
  ATQ_CTRL15   = 0x2D             // Auto Torque Register 15.
  ATQ_CTRL16   = 0x2E             // Auto Torque Register 16.
  ATQ_CTRL17   = 0x2F             // Auto Torque Register 17.
  ATQ_CTRL18   = 0x30             // Auto Torque Register 18.
  SS_CTRL1     = 0x31             // Silent Step Register 1.
  SS_CTRL2     = 0x32             // Silent Step Register 2.
  SS_CTRL3     = 0x33             // Silent Step Register 3.
  SS_CTRL4     = 0x34             // Silent Step Register 4.
  SS_CTRL5     = 0x35             // Silent Step Register 5.
  CTRL14       = 0x3C             // Control Register 14.
};

/// Bits that are set in the return value of readFault() to indicate warning and
/// fault conditions.
///
/// See the DRV8461 datasheet for detailed descriptions of these conditions.
enum class DRV8461FaultBit : uint8_t
{
  /// Fault indication (0 when nFAULT pin is high, 1 when nFAULT pin is low)
  FAULT = 7,

  /// SPI protocol error (latched)
  SPI_ERROR = 6,

  /// Supply undervoltage lockout fault
  UVLO = 5,

  /// Charge pump undervoltage fault
  CPUV = 4,

  /// Overcurrent fault
  OCP = 3,

  /// Motor stall
  STL = 2,

  /// Overtemperature warning or shutdown
  TF = 1,

  /// Open load
  OL = 0,
};

/// Bits that are set in the return value of readDiag1() to indicate warning and
/// fault conditions.
///
/// See the DRV8461 datasheet for detailed descriptions of these conditions.
enum class DRV8461Diag1Bit : uint8_t
{
  /// Overcurrent fault on low-side FET of half bridge 2 in BOUT
  OCP_LS2_B = 7,

  /// Overcurrent fault on high-side FET of half bridge 2 in BOUT
  OCP_HS2_B = 6,

  /// Overcurrent fault on low-side FET of half bridge 1 in BOUT
  OCP_LS1_B = 5,

  /// Overcurrent fault on high-side FET of half bridge 1 in BOUT
  OCP_HS1_B = 4,

  /// Overcurrent fault on low-side FET of half bridge 2 in AOUT
  OCP_LS2_A = 3,

  /// Overcurrent fault on high-side FET of half bridge 2 in AOUT
  OCP_HS2_A = 2,

  /// Overcurrent fault on low-side FET of half bridge 1 in AOUT
  OCP_LS1_A = 1,

  /// Overcurrent fault on high-side FET of half bridge 1 in AOUT
  OCP_HS1_A = 0,
};

/// Bits that are set in the return value of readDiag2() to indicate warning and
/// fault conditions.
///
/// See the DRV8461 datasheet for detailed descriptions of these conditions.
enum class DRV8461Diag2Bit : uint8_t
{
  /// Standstill power saving mode
  STSL = 7,
  
  /// Overtemperature warning
  OTW = 6,

  /// Overtemperature shutdown
  OTS = 5,

  /// Stall detection learning successful
  STL_LRN_OK = 4,

  /// Motor stall condition
  STALL = 3,

  /// Auto Torque Learning successful
  LRN_DONE = 2,

  /// Open load on BOUT
  OL_B = 1,

  /// Open load on AOUT
  OL_A = 0,
};


enum class DRV8461Diag3Bit : uint8_t
{ 
  /// nHOME
  NHOME = 6,

  /// Current Overflow
  CNT_OFLW = 5,

  /// Current Underflow
  CNT_UFLW = 4,
};


/// Possible arguments to setDecayMode().
enum class DRV8461DecayMode : uint8_t
{
  Slow                   = 0b000,
  IncSlowDecMixed30      = 0b001,
  IncSlowDecMixed60      = 0b010,
  IncSlowDecFast         = 0b011,
  Mixed30                = 0b100,
  Mixed60                = 0b101,
  SmartTuneDynamicDecay  = 0b110,
  SmartTuneRippleControl = 0b111,
};

/// Possible arguments to setStepMode().
enum class DRV8461StepMode : uint8_t
{
  /// Full step with 100% current
  MicroStep1_100 = 0b0000,

  /// Full step with 71% current
  MicroStep1     = 0b0001,

  /// Non-circular 1/2 step
  MicroStep2_NC  = 0b0010,

  /// Circular 1/2 step
  MicroStep2     = 0b0011,

  MicroStep4     = 0b0100,
  MicroStep8     = 0b0101,
  MicroStep16    = 0b0110,
  MicroStep32    = 0b0111,
  MicroStep64    = 0b1000,
  MicroStep128   = 0b1001,
  MicroStep256   = 0b1010,
};


/// This class provides low-level functions for reading and writing from the SPI
/// interface of a DRV8461 stepper motor controller IC.
///
/// Most users should use the HighPowerStepperDriver class, which provides a
/// higher-level interface, instead of this class.
class DRV8461SPI
{
public:
  /// Configures this object to use the specified pin as a chip select pin.
  ///
  /// You must use a chip select pin; the DRV8461 requires it.
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
  uint16_t readReg(DRV8461RegAddr address)
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
  void writeReg(DRV8461RegAddr address, uint8_t value)
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
  /// register with DRV8461::readFault(), except the upper two bits are always
  /// 1.
  uint8_t lastStatus = 0;
};


/// This class provides high-level functions for controlling a DRV8461 stepper
/// motor driver.
class DRV8461
{
public:
  /// The default constructor.
  DRV8461()
  {
    // All settings set to power-on defaults
    ctrl1 = 0x00;
    ctrl2 = 0x0F;
    ctrl3 = 0x06;
    ctrl4 = 0x30;
    ctrl5 = 0x08;
    ctrl6 = 0x03;
    ctrl7 = 0x20;
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
    ctrl1 = 0x00;
    ctrl2 = 0x0F;
    ctrl3 = 0x06;
    ctrl4 = 0x30;
    ctrl5 = 0x08;
    ctrl6 = 0x03;
    ctrl7 = 0x20;
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
    return driver.readReg(DRV8461RegAddr::CTRL1) == ctrl1 &&
           driver.readReg(DRV8461RegAddr::CTRL2) == ctrl2 &&
           driver.readReg(DRV8461RegAddr::CTRL3) == ctrl3 &&
           driver.readReg(DRV8461RegAddr::CTRL4) == ctrl4 &&
           driver.readReg(DRV8461RegAddr::CTRL5) == ctrl5 &&
           driver.readReg(DRV8461RegAddr::CTRL6) == ctrl6 &&
           driver.readReg(DRV8461RegAddr::CTRL7) == ctrl7;
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
    writeCachedReg(DRV8461RegAddr::CTRL1);
    writeCachedReg(DRV8461RegAddr::CTRL2);
    writeCachedReg(DRV8461RegAddr::CTRL3);
    writeCachedReg(DRV8461RegAddr::CTRL4);
    writeCachedReg(DRV8461RegAddr::CTRL5);
    writeCachedReg(DRV8461RegAddr::CTRL6);
    writeCachedReg(DRV8461RegAddr::CTRL9);
    writeCachedReg(DRV8461RegAddr::CTRL10);
    writeCachedReg(DRV8461RegAddr::CTRL11);
    writeCachedReg(DRV8461RegAddr::CTRL12);
    writeCachedReg(DRV8461RegAddr::CTRL13);
    writeCachedReg(DRV8461RegAddr::CTRL14);
    writeCachedReg(DRV8461RegAddr::CSTMCTRL1);
    writeCachedReg(DRV8461RegAddr::CTRL1);

    // CTRL1 is written last because it contains the EN_OUT bit, and we want to
    // try to have all the other settings correct first.
    writeCachedReg(DRV8461RegAddr::CTRL1);
  }

  /// Sets the driver's current scalar (TRQ_DAC), which scales the full current
  /// limit (as set by VREF) by the specified percentage. The available settings
  /// are multiples of 6.25%.
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
    if (percent < (1/256)*100) { percent = 0.4; }

    uint8_t td = ((uint16_t)percent * 64) / 25; // convert 0.4-100% to 1-256,
    td = td - 1
    ctrl11 = 0b0000000 | (td);
    writeCachedReg(DRV8461RegAddr::CTRL11);
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

    uint8_t td = (current * 16 / fullCurrent); // convert 0-fullCurrent to 0-16
    if (td == 0) { td = 1; }                   // restrict to 1-16
    td = 16 - td;                              // convert 1-16 to 15-0 (15 = 6.25%, 0 = 100%)
    ctrl1 = (ctrl1 & 0b00001111) | (td << 4);
    writeCachedReg(DRV8461RegAddr::CTRL1);
  }

  /// Enables the driver (EN_OUT = 1).
  void enableDriver()
  {
    ctrl2 |= (1 << 7);
    writeCachedReg(DRV8461RegAddr::CTRL2);
  }

  /// Disables the driver (EN_OUT = 0).
  void disableDriver()
  {
    ctrl2 &= ~(1 << 7);
    writeCachedReg(DRV8461RegAddr::CTRL2);
  }

  /// Sets the driver's decay mode (DECAY).
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// sd.setDecayMode(DRV8461DecayMode::SmartTuneDynamicDecay);
  /// ~~~
  void setDecayMode(DRV8461DecayMode mode)
  {
    ctrl2 = (ctrl2 & 0b11111000) | ((uint8_t)mode & 0b111);
    writeCachedReg(DRV8461RegAddr::CTRL2);
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
      ctrl3 |= (1 << 7);
    }
    else
    {
      ctrl3 &= ~(1 << 7);
    }
    writeCachedReg(DRV8461RegAddr::CTRL3);
  }

  /// Returns the cached value of the motor direction (DIR).
  ///
  /// This does not perform any SPI communication with the driver.
  bool getDirection()
  {
    return (ctrl3 >> 7) & 1;
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
    driver.writeReg(DRV8461RegAddr::CTRL3, ctrl3 | (1 << 6));
  }

  /// Enables direction control through SPI (SPI_DIR = 1), allowing
  /// setDirection() to override the DIR pin.
  void enableSPIDirection()
  {
    ctrl3 |= (1 << 5);
    writeCachedReg(DRV8461RegAddr::CTRL3);
  }

  /// Disables direction control through SPI (SPI_DIR = 0), making the DIR pin
  /// control direction instead.
  void disableSPIDirection()
  {
    ctrl3 &= ~(1 << 5);
    writeCachedReg(DRV8461RegAddr::CTRL3);
  }

  /// Enables stepping through SPI (SPI_STEP = 1), allowing step() to override
  /// the STEP pin.
  void enableSPIStep()
  {
    ctrl3 |= (1 << 4);
    writeCachedReg(DRV8461RegAddr::CTRL3);
  }

  /// Disables stepping through SPI (SPI_STEP = 0), making the STEP pin control
  /// stepping instead.
  void disableSPIStep()
  {
    ctrl3 &= ~(1 << 4);
    writeCachedReg(DRV8461RegAddr::CTRL3);
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
  /// sd.setStepMode(DRV8461StepMode::MicroStep32);
  /// ~~~
  void setStepMode(DRV8461StepMode mode)
  {
    if (mode > DRV8461StepMode::MicroStep256)
    {
      // Invalid mode; pick 1/16 micro-step by default.
      mode = DRV8461StepMode::MicroStep16;
    }

    ctrl3 = (ctrl3 & 0b11110000) | (uint8_t)mode;
    writeCachedReg(DRV8461RegAddr::CTRL3);
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
  void setStepMode(uint16_t mode)
  {
    DRV8461StepMode sm;

    switch (mode)
    {
      case 1:   sm = DRV8461StepMode::MicroStep1;   break;
      case 2:   sm = DRV8461StepMode::MicroStep2;   break;
      case 4:   sm = DRV8461StepMode::MicroStep4;   break;
      case 8:   sm = DRV8461StepMode::MicroStep8;   break;
      case 16:  sm = DRV8461StepMode::MicroStep16;  break;
      case 32:  sm = DRV8461StepMode::MicroStep32;  break;
      case 64:  sm = DRV8461StepMode::MicroStep64;  break;
      case 128: sm = DRV8461StepMode::MicroStep128; break;
      case 256: sm = DRV8461StepMode::MicroStep256; break;

      // Invalid mode; pick 1/16 micro-step by default.
      default:  sm = DRV8461StepMode::MicroStep16;
    }

    setStepMode(sm);
  }

  /// Reads the FAULT status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// FAULT condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8461FaultBit enum to check individual bits.
  ///
  /// Example usage:
  /// ~~~{.cpp}
  /// if (sd.readFault() & (1 << (uint8_t)DRV8461FaultBit::UVLO))
  /// {
  ///   // Supply undervoltage lockout is active.
  /// }
  /// ~~~
  uint8_t readFault()
  {
    return driver.readReg(DRV8461RegAddr::FAULT);
  }

  /// Reads the DIAG1 status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// DIAG1 condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8461Diag1Bit enum to check individual bits.
  uint8_t readDiag1()
  {
    return driver.readReg(DRV8461RegAddr::DIAG1);
  }

  /// Reads the DIAG2 status register of the driver.
  ///
  /// The return value is an 8-bit unsigned integer that has one bit for each
  /// DIAG2 condition.  You can simply compare the return value to 0 to see if
  /// any of the bits are set, or you can use the logical AND operator (`&`) and
  /// the #DRV8461Diag2Bit enum to check individual bits.
  uint8_t readDiag2()
  {
    return driver.readReg(DRV8461RegAddr::DIAG2);
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
    driver.writeReg(DRV8461RegAddr::CTRL4, ctrl4 | (1 << 7));
  }

  /// Gets the cached value of a register. If the given register address is not
  /// valid, this function returns 0.
  uint8_t getCachedReg(DRV8461RegAddr address)
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
  /// for direct register accesses instead of calling DRV8461SPI::writeReg()
  /// directly.
  void setReg(DRV8461RegAddr address, uint8_t value)
  {
    uint8_t * cachedReg = cachedRegPtr(address);
    if (!cachedReg) { return; }
    *cachedReg = value;
    driver.writeReg(address, value);
  }

protected:

  uint8_t ctrl1, ctrl2, ctrl3, ctrl4, ctrl5, ctrl6, ctrl9, ctrl10, ctrl11, ctrl12, ctrl13, 
  custom_ctrl1, custom_ctrl2, custom_ctrl3, custom_ctrl4, custom_ctrl5, custom_ctrl6, custom_ctrl7, custom_ctrl8, custom_ctrl9, 
  atq_ctrl2, atq_ctrl3, atq_ctrl4, atq_ctrl5, atq_ctrl6, atq_ctrl7, atq_ctrl8, atq_ctrl9, atq_ctrl10, atq_ctrl11, atq_ctrl12, atq_ctrl13, atq_ctrl14, atq_ctrl15, atq_ctrl17, atq_ctrl18, 
  ss_ctrl1, ss_ctrl2, ss_ctrl3, ss_ctrl4, ss_ctrl5, ctrl14;

  /// Returns a pointer to the variable containing the cached value for the
  /// given register.
  uint8_t * cachedRegPtr(DRV8461RegAddr address)
  {
    switch (address)
    {
      case DRV8461RegAddr::CTRL1: return &ctrl1;
      case DRV8461RegAddr::CTRL2: return &ctrl2;
      case DRV8461RegAddr::CTRL3: return &ctrl3;
      case DRV8461RegAddr::CTRL4: return &ctrl4;
      case DRV8461RegAddr::CTRL5: return &ctrl5;
      case DRV8461RegAddr::CTRL6: return &ctrl6;
      case DRV8461RegAddr::CTRL9: return &ctrl9;
      case DRV8461RegAddr::CTRL10: return &ctrl10;
      case DRV8461RegAddr::CTRL11: return &ctrl11;
      case DRV8461RegAddr::CTRL12: return &ctrl12;
      case DRV8461RegAddr::CTRL13: return &ctrl13;
      case DRV8461RegAddr::CUSTOM_CTRL1: return &custom_ctrl1;
      case DRV8461RegAddr::CUSTOM_CTRL2: return &custom_ctrl2;
      case DRV8461RegAddr::CUSTOM_CTRL3: return &custom_ctrl3;
      case DRV8461RegAddr::CUSTOM_CTRL4: return &custom_ctrl4;
      case DRV8461RegAddr::CUSTOM_CTRL5: return &custom_ctrl5;
      case DRV8461RegAddr::CUSTOM_CTRL6: return &custom_ctrl6;
      case DRV8461RegAddr::CUSTOM_CTRL7: return &custom_ctrl7;
      case DRV8461RegAddr::CUSTOM_CTRL8: return &custom_ctrl8;
      case DRV8461RegAddr::CUSTOM_CTRL9: return &custom_ctrl9;
      case DRV8461RegAddr::ATQ_CTRL2: return &atq_ctrl2;
      case DRV8461RegAddr::ATQ_CTRL3: return &atq_ctrl3;
      case DRV8461RegAddr::ATQ_CTRL4: return &atq_ctrl4;
      case DRV8461RegAddr::ATQ_CTRL5: return &atq_ctrl5;
      case DRV8461RegAddr::ATQ_CTRL6: return &atq_ctrl6;
      case DRV8461RegAddr::ATQ_CTRL7: return &atq_ctrl7;
      case DRV8461RegAddr::ATQ_CTRL8: return &atq_ctrl8;
      case DRV8461RegAddr::ATQ_CTRL9: return &atq_ctrl9;
      case DRV8461RegAddr::ATQ_CTRL10: return &atq_ctrl10;
      case DRV8461RegAddr::ATQ_CTRL11: return &atq_ctrl11;
      case DRV8461RegAddr::ATQ_CTRL12: return &atq_ctrl12;
      case DRV8461RegAddr::ATQ_CTRL13: return &atq_ctrl13;
      case DRV8461RegAddr::ATQ_CTRL14: return &atq_ctrl14;
      case DRV8461RegAddr::ATQ_CTRL15: return &atq_ctrl15;
      case DRV8461RegAddr::ATQ_CTRL17: return &atq_ctrl17;
      case DRV8461RegAddr::ATQ_CTRL18: return &atq_ctrl18;
      case DRV8461RegAddr::SS_CTRL1: return &ss_ctrl1;
      case DRV8461RegAddr::SS_CTRL2: return &ss_ctrl2;
      case DRV8461RegAddr::SS_CTRL3: return &ss_ctrl3;
      case DRV8461RegAddr::SS_CTRL4: return &ss_ctrl4;
      case DRV8461RegAddr::SS_CTRL5: return &ss_ctrl5;
      case DRV8461RegAddr::CTRL14: return &ctrl14;

      default: return nullptr;
    }
  }

  /// Writes the cached value of the given register to the device.
  void writeCachedReg(DRV8461RegAddr address)
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
  DRV8461SPI driver;
};

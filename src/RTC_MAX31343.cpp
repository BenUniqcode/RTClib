#include "RTClib.h"

#define MAX31343_ADDRESS 0x68    ///< I2C address for MAX31343
#define MAX31343_STATUSREG 0x00  ///< Status register
#define MAX31343_INT_EN 0x01     ///< Interrupt Enable register
#define MAX31343_RTC_RESET 0x02  ///< Reset register
#define MAX31343_RTC_CONFIG1 0x03 ///< RTC Config1 register
#define MAX31343_RTC_CONFIG2 0x04 ///< RTC Config2 register
#define MAX31343_TIMER_CONFIG 0x05 ///< Timer Config register
#define MAX31343_TIME 0x06       ///< Time register
#define MAX31343_ALARM1 0x0D     ///< Alarm 1 register
#define MAX31343_ALARM2 0x13     ///< Alarm 2 register
#define MAX31343_TIMER_COUNT 0x16     ///< Timer Count register
#define MAX31343_TIMER_INIT 0x17     ///< Timer Init register
#define MAX31343_PWR_MGMT 0x17     ///< Power Management register
#define MAX31343_TRICKLE 0x17     ///< Trickle Charging register
#define MAX31343_TEMPERATUREREG 0x1A ///< Temperature register (high byte - low byte is at 0x1B), 10-bit 2's complement
#define MAX31343_TS_CONFIG 0x1C //< Temperature Sensor Config register
#define MAX31343_NVRAM 0x22 ///< Start of RAM registers - 64 bytes, 0x22 to 0x61
/**************************************************************************/
/*!
        @brief  Start I2C for the MAX31343 and test succesful connection
        @param  wireInstance pointer to the I2C bus
        @return True if Wire can find MAX31343 or false otherwise.
*/
/**************************************************************************/
boolean RTC_MAX31343::begin(TwoWire *wireInstance) {
  if (i2c_dev)
    delete i2c_dev;
  i2c_dev = new Adafruit_I2CDevice(MAX31343_ADDRESS, wireInstance);
  if (!i2c_dev->begin())
    return false;
  return true;
}

/**************************************************************************/
/*!
        @brief  Check the status register Oscillator Stop Flag to see if the
   MAX31343 stopped due to power loss
        @return True if the bit is set (oscillator stopped) or false if it is
   running
*/
/**************************************************************************/
bool RTC_MAX31343::lostPower() {
  return (status_ >> 6) & 1;
}

/**************************************************************************/
/*!
        @brief  Read the STATUS register. Reading this automatically clears any interrupts,
    hence the return value needs to be saved and passed into any functions that check for different
    interrupt causes.
        @return True if the bit is set (oscillator stopped) or false if it is
   running
*/
/**************************************************************************/
void RTC_MAX31343::readStatusAndClearInterrupts()
{
  status_ = read_register(MAX31343_STATUSREG);
}

/**************************************************************************/
/*!
        @brief  Set the date and clear the Oscillator Stop Flag
        @param dt DateTime object containing the date/time to set
*/
/**************************************************************************/
void RTC_MAX31343::adjust(const DateTime &dt) {
  // The top bit of the month register indicates the century: 20 or 21
  uint8_t month = bin2bcd(dt.month());
  if (dt.year() > 2099) {
    month |= 0x80;
  }
  uint8_t buffer[8] = {MAX31343_TIME,
                       bin2bcd(dt.second()),
                       bin2bcd(dt.minute()),
                       bin2bcd(dt.hour()),
                       dt.dayOfTheWeek(),
                       bin2bcd(dt.day()),
                       month,
                       bin2bcd(dt.year() - 2000U)};
  i2c_dev->write(buffer, 8);

  // Unlike other RTCs, we do not automatically clear interrupts here.
  // Any read of the STATUS reg does so, so leave that up to the caller to decide.
}

/**************************************************************************/
/*!
        @brief  Get the current date/time
        @return DateTime object with the current date/time
*/
/**************************************************************************/
DateTime RTC_MAX31343::now() {
  uint8_t buffer[7];
  buffer[0] = MAX31343_TIME;
  i2c_dev->write_then_read(buffer, 1, buffer, 7);

  return DateTime(bcd2bin(buffer[6]) + 2000U, bcd2bin(buffer[5] & 0x7F),
                  bcd2bin(buffer[4]), bcd2bin(buffer[2]), bcd2bin(buffer[1]),
                  bcd2bin(buffer[0] & 0x7F));
}

/**************************************************************************/
/*!
        @brief  Read the SQW pin mode
        @return Pin mode, see Max31343SqwPinMode enum
*/
/**************************************************************************/
Max31343SqwPinMode RTC_MAX31343::readSqwPinMode() {
  int mode;
  mode = read_register(MAX31343_RTC_CONFIG2) & 0x07;
  return static_cast<Max31343SqwPinMode>(mode);
}

/**************************************************************************/
/*!
        @brief  Set the SQW pin mode
        @param mode Desired mode, see Max31343SqwPinMode enum
*/
/**************************************************************************/
void RTC_MAX31343::writeSqwPinMode(Max31343SqwPinMode mode) {
  uint8_t config = read_register(MAX31343_RTC_CONFIG2);
  config &= ~0x7; // clear relevant bits
  write_register(MAX31343_RTC_CONFIG2, config | mode);
}

/**************************************************************************/
/*!
        @brief  Get the current temperature from the MAX31343's temperature sensor
        @return Current temperature (float)
*/
/**************************************************************************/
float RTC_MAX31343::getTemperature() {
  uint8_t buffer[2] = {MAX31343_TEMPERATUREREG, 0};
  i2c_dev->write_then_read(buffer, 1, buffer, 2);
  // Fix for negative temperatures https://github.com/adafruit/RTClib/pull/303
  int16_t temp = uint16_t(buffer[0]) << 8 | buffer[1];
  return temp * (1 / 256.0);
}

/**************************************************************************/
/*!
        @brief  Set alarm 1 for MAX31343
                @param 	dt DateTime object
                @param 	alarm_mode Desired mode, see Max31343Alarm1Mode enum
        @return False if control register is not set, otherwise true
*/
/**************************************************************************/
bool RTC_MAX31343::setAlarm1(const DateTime &dt, Max31343Alarm1Mode alarm_mode) {
  uint8_t A1M1 = (alarm_mode & 0x01) << 7; // Seconds bit 7.
  uint8_t A1M2 = (alarm_mode & 0x02) << 6; // Minutes bit 7.
  uint8_t A1M3 = (alarm_mode & 0x04) << 5; // Hour bit 7.
  uint8_t A1M4 = (alarm_mode & 0x08) << 4; // Day/Date bit 7.
  uint8_t A1M5 = (alarm_mode & 0x10) << 3; // Month bit 7.
  uint8_t A1M6 = (alarm_mode & 0x20) << 1; // Month bit 6.
  uint8_t DY_DT = (alarm_mode & 0x40); // Day/Date bit 6. Date when 0, day of week when 1.
  uint8_t day = (DY_DT) ? dt.dayOfTheWeek() : dt.day();

  uint8_t buffer[7] = {MAX31343_ALARM1, uint8_t(bin2bcd(dt.second()) | A1M1),
                       uint8_t(bin2bcd(dt.minute()) | A1M2),
                       uint8_t(bin2bcd(dt.hour()) | A1M3),
                       uint8_t(bin2bcd(day) | A1M4 | DY_DT),
                       uint8_t(bin2bcd(dt.month()) | A1M5 | A1M6),
                       uint8_t(bin2bcd(dt.year() - 2000U))};
  i2c_dev->write(buffer, 7);
  uint8_t int_en = read_register(MAX31343_INT_EN);
  write_register(MAX31343_INT_EN, int_en | 0x01); // A1IE

  return true;
}

/**************************************************************************/
/*!
        @brief  Set alarm 2 for MAX31343
                @param 	dt DateTime object
                @param 	alarm_mode Desired mode, see Max31343Alarm2Mode enum
        @return False if control register is not set, otherwise true
*/
/**************************************************************************/
bool RTC_MAX31343::setAlarm2(const DateTime &dt, Max31343Alarm2Mode alarm_mode) {
  uint8_t A2M2 = (alarm_mode & 0x01) << 7; // Minutes bit 7.
  uint8_t A2M3 = (alarm_mode & 0x02) << 6; // Hour bit 7.
  uint8_t A2M4 = (alarm_mode & 0x04) << 5; // Day/Date bit 7.
  uint8_t DY_DT = (alarm_mode & 0x08)
                  << 3; // Day/Date bit 6. Date when 0, day of week when 1.
  uint8_t day = (DY_DT) ? dt.dayOfTheWeek() : dt.day();

  uint8_t buffer[4] = {MAX31343_ALARM2, uint8_t(bin2bcd(dt.minute()) | A2M2),
                       uint8_t(bin2bcd(dt.hour()) | A2M3),
                       uint8_t(bin2bcd(day) | A2M4 | DY_DT)};
  i2c_dev->write(buffer, 4);

  uint8_t int_en = read_register(MAX31343_INT_EN);
  write_register(MAX31343_INT_EN, int_en | 0x02); // A2IE

  return true;
}

/**************************************************************************/
/*!
        @brief  Disable alarm
                @param 	alarm_num Alarm number to disable
*/
/**************************************************************************/
void RTC_MAX31343::disableAlarm(uint8_t alarm_num) {
  uint8_t int_en = read_register(MAX31343_INT_EN);
  int_en &= ~(1 << (alarm_num - 1));
  write_register(MAX31343_INT_EN, int_en);
}

/**************************************************************************/
/*!
        @brief  Clear status of alarm
                @param 	alarm_num Alarm number to clear
*/
/**************************************************************************/
void RTC_MAX31343::clearAlarm(uint8_t alarm_num) {
  status_ &= ~(1U << (alarm_num - 1));
}

/**************************************************************************/
/*!
        @brief  Get status of alarm
                @param 	alarm_num Alarm number to check status of
                @return True if alarm has been fired otherwise false
*/
/**************************************************************************/
bool RTC_MAX31343::alarmFired(uint8_t alarm_num) {
  return (status_ >> (alarm_num - 1)) & 0x1;
}

/**************************************************************************/
/*!
        @brief  Enable Clock Output on CLKO pin
        @details The MAX31343's clock output is configurable for different frequencies
*/
/**************************************************************************/
void RTC_MAX31343::enableClkOut(Max31343ClkOutFreq freq) {
  uint8_t config2 = read_register(MAX31343_RTC_CONFIG2);
  config2 &= 0x7; // Clear everything except the bottom 3 bits, which are the SQW freq
  config2 |= 0x80 | freq;
  write_register(MAX31343_RTC_CONFIG2, config2);
}

/**************************************************************************/
/*!
        @brief  Disable Clock Output on CLKO pin
*/
/**************************************************************************/
void RTC_MAX31343::disableClkOut(void) {
  uint8_t config2 = read_register(MAX31343_RTC_CONFIG2);
  config2 &= ~0x80; // Clear top bit
  write_register(MAX31343_RTC_CONFIG2, config2);
}

/**************************************************************************/
/*!
        @brief  Get status of Clock Output
        @return True if enabled otherwise false
*/
/**************************************************************************/
bool RTC_MAX31343::isEnabledClkOut(void) {
  return (read_register(MAX31343_RTC_CONFIG2) >> 7) & 0x01;
}

/**************************************************************************/
/*!
        @brief  Clear Oscillator Stop Flag (OSF). Bit 6 of STATUSREG (0Fh)
        @details A logic 1 in this bit indicates that the oscillator either is
         stopped or was stopped for some period and may be used to judge the
   validity of the timekeeping data. This bit is set to logic 1 any time that
   the oscillator stops. The following are examples of conditions that can cause
   the OSF bit to be set: 1) The first time power is applied. 2) The voltages
         present on both VCC and VBAT are insufficient to support oscillation.
   3) The EOSC bit is turned off in battery-backed mode. 4) External influences
   on the crystal (i.e., noise, leakage, etc.). This bit remains at logic 1
   until written to logic 0.
*/
/**************************************************************************/
void RTC_MAX31343::clearOSF(void) {
  status_ &= ~0x40; // clear OSF bit
}

/**************************************************************************/
/*!
        @brief  Read data from the MAX31343's NVRAM
        @param buf Pointer to a buffer to store the data - make sure it's large
   enough to hold size bytes
        @param size Number of bytes to read
        @param address Starting NVRAM address, from 0 to 236
*/
/**************************************************************************/
void RTC_MAX31343::readnvram(uint8_t *buf, uint8_t size, uint8_t address) {
  uint8_t addrByte = MAX31343_NVRAM + address;
  i2c_dev->write_then_read(&addrByte, 1, buf, size);
}
/**************************************************************************/
/*!
        @brief  Write data to the MAX31343 NVRAM
        @param address Starting NVRAM address, from 0 to 236
        @param buf Pointer to buffer containing the data to write
        @param size Number of bytes in buf to write to NVRAM
*/
/**************************************************************************/
void RTC_MAX31343::writenvram(uint8_t address, const uint8_t *buf, uint8_t size) {
  uint8_t addrByte = MAX31343_NVRAM + address;
  i2c_dev->write(buf, size, true, &addrByte, 1);
}

/**************************************************************************/
/*!
        @brief  Shortcut to read one byte from NVRAM
        @param address NVRAM address, 0 to 236
        @return The byte read from NVRAM
*/
/**************************************************************************/
uint8_t RTC_MAX31343::readnvram(uint8_t address) {
  uint8_t data;
  readnvram(&data, 1, address);
  return data;
}

/**************************************************************************/
/*!
        @brief  Shortcut to write one byte to NVRAM
        @param address NVRAM address, 0 to 236
        @param data One byte to write
*/
/**************************************************************************/
void RTC_MAX31343::writenvram(uint8_t address, uint8_t data) {
  writenvram(address, &data, 1);
}

#include "RTClib.h"

#define RV3032_ADDRESS 0x51   ///< I2C address for RV-3032
#define RV3032_TIME 0x01      ///< Start of Time registers, ignoring 1/100ths of a second
#define RV3032_ALARM 0x08     ///< Start of Alarm registers
#define RV3032_CONTROL1 0x10   ///< Control1 register
#define RV3032_CONTROL2 0x11   ///< Control2 register
#define RV3032_CONTROL3 0x12   ///< Control2 register
#define RV3032_STATUS 0x0D ///< Status register
#define RV3032_TEMPERATURE 0x0E ///< Temperature register (LSB and flags) (MSB is 0x0F)
#define RV3032_EECMD 0x3F // EEPROM Command register (write only)
#define RV3032_NVRAM 0x40 ///< Start of RAM - 16 bytes
#define RV3032_PMU 0xC0 // EEPROM Power Management Unit register
// Bits within CONTROL registers that we are interested in
#define RV3032_CONTROL1_BIT_EERD 2 // EEPROM Refresh Disable
#define RV3032_CONTROL2_BIT_AIE 3 // Alarm Interrupt Enable
// Bits within STATUS register that we are interested in
#define RV3032_STATUS_BIT_PORF 1 // Power On Reset Flag - set when starting after power loss
#define RV3032_STATUS_BIT_AF 3 // Alarm Flag - set when alarm interrupt fires
// Bits within the TEMPERATURE register that we are interested in
#define RV3032_TEMPERATURE_BIT_BSF 0
#define RV3032_TEMPERATURE_BIT_EEBUSY 2
#define RV3032_TEMPERATURE_BIT_EEF 3
// Bits within the PMU register that we are interested in
#define RV3032_PMU_BIT_BSM_HIGH 5
#define RV3032_PMU_BIT_BSM_LOW 4

/**************************************************************************/
/*!
        @brief  Start I2C for the RV3032 and test succesful connection
        @param  wireInstance pointer to the I2C bus
        @return True if Wire can find RV3032 or false otherwise.
*/
/**************************************************************************/
boolean RTC_RV3032::begin(TwoWire *wireInstance) {
  if (i2c_dev)
    delete i2c_dev;
  i2c_dev = new Adafruit_I2CDevice(RV3032_ADDRESS, wireInstance);
  if (!i2c_dev->begin())
    return false;
  return true;
}

/**************************************************************************/
/*!
        @brief  Check the status register PORF to see if the
   RV3032 stopped due to power loss
        @return True if the bit is set or false if not
*/
/**************************************************************************/
bool RTC_RV3032::lostPower(void) {
  return (read_register(RV3032_STATUS) >> RV3032_STATUS_BIT_PORF) & 0x01;
}

/**************************************************************************/
/*!
        @brief  Clear Power On Reset Flag bit if set
        @details A logic 1 in this bit (read by lostPower()) indicates that the power was lost. This function resets the bit to 0.
*/
/**************************************************************************/
void RTC_RV3032::clearLostPower(void) {
  uint8_t porfMask = 1 << RV3032_STATUS_BIT_PORF;
  uint8_t statreg = read_register(RV3032_STATUS);
  if (statreg & porfMask) {
    statreg &= ~porfMask; // clear PORF bit
    write_register(RV3032_STATUS, statreg);
  }
}

/**************************************************************************/
/*!
        @brief  Check the BSF flag to see if the RTC switched over to backup. Cleared on read.
        @return True if the bit is set or false if not
*/
/**************************************************************************/
bool RTC_RV3032::backupSwitchoverFlag()
{
  // BSF is in the first byte of the temperature register
  uint8_t reg = read_register(RV3032_TEMPERATURE);
  return reg & (1 << RV3032_TEMPERATURE_BIT_BSF);
} 

/**************************************************************************/
/*!
        @brief  Set the date and time
        @param dt DateTime object containing the date/time to set
*/
/**************************************************************************/
void RTC_RV3032::adjust(const DateTime &dt) {
  // Although the RV3032 has a 1/100th of a second register, it is not writeable
  // Writing to the seconds register ensures a safe time update. 
  uint8_t buffer[8] = {RV3032_TIME,
                       bin2bcd(dt.second()),
                       bin2bcd(dt.minute()),
                       bin2bcd(dt.hour()),
                       bin2bcd(dt.dayOfTheWeek()), // Unlike the DS3232, this is a number from 0 to 6, same as in DateTime, so does not need additional conversion.
                       bin2bcd(dt.day()),
                       bin2bcd(dt.month()),
                       bin2bcd(dt.year() - 2000U)};
  i2c_dev->write(buffer, 8);
  // Clear the PORF bit if set
  clearLostPower();
}

/**************************************************************************/
/*!
        @brief  Get the current date/time
        @return DateTime object with the current date/time
*/
/**************************************************************************/
DateTime RTC_RV3032::now() {
  uint8_t buffer[7];
  buffer[0] = RV3032_TIME;
  i2c_dev->write_then_read(buffer, 1, buffer, 7);

  return DateTime(bcd2bin(buffer[6]) + 2000U, bcd2bin(buffer[5]),
                  bcd2bin(buffer[4]), bcd2bin(buffer[2]), bcd2bin(buffer[1]),
                  bcd2bin(buffer[0]));
}


/**************************************************************************/
/*!
        @brief  Get the current temperature from the RV3032's temperature sensor
        @return Current temperature (float)
*/
/**************************************************************************/
float RTC_RV3032::getTemperature() {
  uint8_t buffer[2] = {RV3032_TEMPERATURE, 0};
  i2c_dev->write_then_read(buffer, 1, buffer, 2);
  // Whereas the DS3232 is MSB-first, the RV3032 is LSB-first. 
  // The fractional part is 4 bits instead of 2, but that doesn't matter for this method.
  // What does matter is the remaining 4 bits are used for flags, so unlike the DS* they
  // are not guaranteed to be zero, and must be explicitly ignored. 
  // Otherwise it's broadly the same, so we use a similar method to https://github.com/adafruit/RTClib/pull/303
  int16_t temp = uint16_t(buffer[1]) << 8 | (buffer[0] & 0xf0);
  return temp * (1 / 256.0);
}

/**************************************************************************/
/*!
        @brief  Set alarm for RV3032
                @param 	dt DateTime object
                @param 	alarm_mode Desired mode, see Rv3032AlarmMode enum
        @return False if control register is not set, otherwise true
*/
/**************************************************************************/
bool RTC_RV3032::setAlarm(const DateTime &dt, Rv3032AlarmMode alarm_mode) {
  // Somewhat bizarrely, the Alarm Enable bits need to be set to 0 to enable, 1 to disable
  bool AE_M = !(alarm_mode & RV3032_AlarmModeBit_Minute);
  bool AE_H = !(alarm_mode & RV3032_AlarmModeBit_Hour);
  bool AE_D = !(alarm_mode & RV3032_AlarmModeBit_Date);

  uint8_t buffer[4] = {RV3032_ALARM, 
                       uint8_t(bin2bcd(dt.minute()) | AE_M),
                       uint8_t(bin2bcd(dt.hour()) | AE_H),
                       uint8_t(bin2bcd(dt.day()) | AE_D)};
  i2c_dev->write(buffer, 4);

  // Enable Alarm Interrupt output pin, if not already enabled
  uint8_t aieMask = 1 << RV3032_CONTROL2_BIT_AIE;
  uint8_t ctrl2 = read_register(RV3032_CONTROL2);
  if ((ctrl2 & aieMask) == 0) {
    ctrl2 |= aieMask;
    write_register(RV3032_CONTROL2, ctrl2);
  }

  return true;
}

/**************************************************************************/
/*!
        @brief  Disable alarm
*/
/**************************************************************************/
void RTC_RV3032::disableAlarm() {
  // Disable Alarm Interrupt output pin, if enabled
  uint8_t aieMask = 1 << RV3032_CONTROL2_BIT_AIE;
  uint8_t ctrl2 = read_register(RV3032_CONTROL2);
  if (ctrl2 & aieMask) {
    ctrl2 &= ~aieMask;
    write_register(RV3032_CONTROL2, ctrl2);
  }
  // Disable the alarm by writing 0 to the Alarm Date (even if the AE_* bits are 0, hence
  // enabled, if the Date is zero then the alarm is disabled). We zero the whole lot to be safe.
  uint8_t buffer[4] = {RV3032_ALARM, 0, 0, 0};
  i2c_dev->write(buffer, 4);
}

/**************************************************************************/
/*!
        @brief  Clear alarm interrupt
                @param 	alarm_num Alarm number to clear
*/
/**************************************************************************/
void RTC_RV3032::clearAlarm() {
  uint8_t status = read_register(RV3032_STATUS);
  status &= ~(1 << RV3032_STATUS_BIT_AF);
  write_register(RV3032_STATUS, status);
}

/**************************************************************************/
/*!
        @brief  Get status of alarm
                @return True if alarm has been fired otherwise false
*/
/**************************************************************************/
bool RTC_RV3032::alarmFired() {
  return (read_register(RV3032_STATUS) & (1 << RV3032_STATUS_BIT_AF));
}

/**************************************************************************/
/*!
        @brief  Get current Backup Switchover Mode
                @return See enum Rv3032BackupSwitchoverMode
*/
/**************************************************************************/
Rv3032BackupSwitchoverMode RTC_RV3032::backupSwitchoverMode()
{
  uint8_t pmu = read_register(RV3032_PMU);
  uint8_t bsmMask = (1 << RV3032_PMU_BIT_BSM_HIGH) | (1 << RV3032_PMU_BIT_BSM_LOW);
  printf("pmu is %u\n", pmu);
  uint8_t bsm = (pmu & bsmMask) >> RV3032_PMU_BIT_BSM_LOW;
  printf("bsm is %u\n", bsm);
  return static_cast<Rv3032BackupSwitchoverMode>(bsm);
}

/**************************************************************************/
/*!
        @brief  Set EERD flag to disable EEPROM refresh prior to updating it
*/
/**************************************************************************/
void RTC_RV3032::disableEEPROMRefresh()
{
  printf("Disable EEPROM refresh\n");
  const uint8_t eerdMask = 1 << RV3032_CONTROL1_BIT_EERD;
  uint8_t control1 = read_register(RV3032_CONTROL1);
  control1 |= eerdMask;
  write_register(RV3032_CONTROL1, control1);
}

/**************************************************************************/
/*!
        @brief  Clear EERD flag to re-enable EEPROM refresh
*/
/**************************************************************************/
void RTC_RV3032::enableEEPROMRefresh()
{
  printf("Enable EEPROM refresh\n");
  const uint8_t eerdMask = 1 << RV3032_CONTROL1_BIT_EERD;
  uint8_t control1 = read_register(RV3032_CONTROL1);
  control1 &= ~eerdMask;
  write_register(RV3032_CONTROL1, control1);
}

/**************************************************************************/
/*!
        @brief  Wait upto 80ms for the EEBUSY flag to clear
                @return True if clear, false if timed out
*/
/**************************************************************************/
bool RTC_RV3032::waitForEEPROM()
{
  printf("Wait for EEPROM\n");
  const unsigned long timeout = millis() + 80;
  const uint8_t eebusyMask = 1 << RV3032_TEMPERATURE_BIT_EEBUSY;
  uint8_t templsb = read_register(RV3032_TEMPERATURE);
  while ((templsb & eebusyMask) && millis() < timeout) {
    delay(5);
    templsb = read_register(RV3032_TEMPERATURE);
  }
  if (templsb & eebusyMask) {
    // Timeout
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
        @brief  Set Backup Switchover Mode to the supplied mode
                @return True if success, false otherwise
*/
/**************************************************************************/
bool RTC_RV3032::setBackupSwitchoverMode(Rv3032BackupSwitchoverMode bsm)
{
  // Although it is possible to write single bytes to the EEPROM (as long as you also
  // update the RAM mirror, because that's what is used for the current config), the 
  // datasheet recommends updating the RAM mirror and then issuing an Update command 
  // to copy the whole lot back to the EEPROM.

  // 1. Set EERD to prevent refresh during EEPROM access
  disableEEPROMRefresh();

  // 3. If EEBUSY is set, wait for it to clear. If that failed, re-enable refresh and return false.
  if (!waitForEEPROM()) {
    enableEEPROMRefresh();
    return false;
  }

  // 2. Update the value of BSM in the RAM Mirror
  printf("Setting BSM to %u in RAM Mirror\n", bsm);
  uint8_t pmu = read_register(RV3032_PMU);
  uint8_t bsmMask = (1 << RV3032_PMU_BIT_BSM_HIGH) | (1 << RV3032_PMU_BIT_BSM_LOW);
  pmu &= ~bsmMask;
  pmu |= bsm;
  write_register(RV3032_PMU, pmu);

  // 4. Update EEPROM
  printf("Writing EEPROM Update command\n");
  write_register(RV3032_EECMD, 0x11); // "Update" command

  // 5. Wait for update to finish (should take ~46ms)
  if (!waitForEEPROM()) {
    enableEEPROMRefresh();
    return false;
  }

  // 5. Clear EERD
  enableEEPROMRefresh();

  // 6. Check EEF
  uint8_t templsb = read_register(RV3032_TEMPERATURE);
  printf("TempLSB is %u\n", templsb);
  printf("EEF is %u\n", templsb & (1 << RV3032_TEMPERATURE_BIT_EEF));
  if (templsb & (1 << RV3032_TEMPERATURE_BIT_EEF)) {
    printf("Fail");
    return false;
  }
  printf("Success");
  return true;
}


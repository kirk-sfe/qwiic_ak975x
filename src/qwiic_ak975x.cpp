/*
  This is a library written for the AK975X Human Presence Sensor.

  Written by Nathan Seidle @ SparkFun Electronics, March 10th, 2017

  The sensor uses I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.

  https://github.com/sparkfun/SparkFun_AK975X_Arduino_Library

  Do you like this library? Help support SparkFun. Buy a board!

  Development environment specifics:
  Arduino IDE 1.8.1

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <string.h>
#include "qwiic_i2c.h"
#include "qwiic_ak975x.h"

Qwiic_I2C qwiic;

//Sets up the sensor for constant read
//Returns false if sensor does not respond
bool AK975X::begin(uint8_t i2caddr)
{

    _i2caddr = i2caddr;

    // init I2C.

    if(!qwiic.init()){
        printf("Error: I2C subsystem failed to initialize.");
        return false;
    }
    if( !isConnected())
        return false;


    setMode(AK975X_MODE_0); //Set to continuous read

    setCutoffFrequency(AK975X_FREQ_8_8HZ); //Set output to fastest, with least filtering

    refresh(); //Read dummy register after new data is read

    return true; //Success!
}
bool AK975X::isConnected(void){

    uint8_t deviceID = qwiic.readRegister(_i2caddr, AK975X_WIA2);
    return deviceID == 0x13; //Device ID should be 0x13

}
bool AK975X::reboot()
{

    setMode(AK975X_MODE_0); //Set to continuous read

    setCutoffFrequency(AK975X_FREQ_8_8HZ); //Set output to fastest, with least filtering

    refresh(); //Read dummy register after new data is read

    return true; //Success!
}

//Returns the decimal value of sensor channel
int16_t AK975X::getIR1()
{
    int16_t value;

    int rc = qwiic.readRegisterInt16(_i2caddr, AK975X_IR1, (uint16_t*)&value);

    return (rc == QWIIC_ERROR_GENERIC ? 0 : value);
}

int16_t AK975X::getIR2()
{
    int16_t value;

    int rc = qwiic.readRegisterInt16(_i2caddr, AK975X_IR2, (uint16_t*)&value);

    return (rc == QWIIC_ERROR_GENERIC ? 0 : value);

}

int16_t AK975X::getIR3()
{
    int16_t value;

    int rc = qwiic.readRegisterInt16(_i2caddr, AK975X_IR3, (uint16_t*)&value);

    return (rc == QWIIC_ERROR_GENERIC ? 0 : value);

}
int16_t AK975X::getIR4()
{
    int16_t value;

    int rc = qwiic.readRegisterInt16(_i2caddr, AK975X_IR4, (uint16_t*)&value);

    return (rc == QWIIC_ERROR_GENERIC ? 0 : value);

}

//This reads the dummy register ST2. Required after
//reading out IR data.
void AK975X::refresh()
{
    (void)qwiic.readRegister(_i2caddr, AK975X_ST2); //Do nothing but read the register
}

//Returns the temperature in C
float AK975X::getTemperature()
{
    uint16_t value;

    int rc = qwiic.readRegisterInt16(_i2caddr, AK975X_TMP, &value);

    if(rc == QWIIC_ERROR_GENERIC)
        return -1.0;

    //value >>= 6; //Temp is 10-bit. TMPL0:5 fixed at 0

    return 26.75 + ((value >> 6) * 0.125);
}

//Returns the temperature in F
float AK975X::getTemperatureF()
{
    //  float temperature = getTemperature();
    //  temperature = temperature * 1.8 + 32.0;
    return getTemperature() * 1.8 + 32.0;
}

//Set the mode. Continuous mode 0 is favored
void AK975X::setMode(uint8_t mode)
{
    if (mode > AK975X_MODE_3 || mode == 0b011) 
        mode = AK975X_MODE_0; //Default to mode 0
    //if (mode == 0b011) mode = AK975X_MODE_0; //0x03 is prohibited

    //Read, mask set, write
    uint8_t currentSettings = qwiic.readRegister(_i2caddr, AK975X_ECNTL1);

    currentSettings = (currentSettings & 0b11111000) | mode; //Clear Mode bits, set our mode
    //  currentSettings |= mode;

    qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, currentSettings);
}

//Set the cutoff frequency. See datasheet for allowable settings.
void AK975X::setCutoffFrequency(uint8_t frequency)
{
    if (frequency > 0b101) 
        frequency = 0; //Default to 0.3Hz

    //Read, mask set, write
    uint8_t currentSettings = qwiic.readRegister(_i2caddr, AK975X_ECNTL1);

    currentSettings = (currentSettings & 0b11000111) | (frequency << 3); //Clear EFC bits

    qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, currentSettings); //Write
}

void AK975X::setThresholdIr2Ir4(bool weight, int v) {

    int k=v >>5; //high bits [11;5]
    int b=v <<3; // low bits

    b &= 0b11111000; // low bits
    k &= 0b01111111; // high bits
    uint8_t currentSettings;

    if(weight) {

        qwiic.writeRegister(_i2caddr, AK975X_ETH24H_LOW, b);
        qwiic.writeRegister(_i2caddr, AK975X_ETH24H_HIGH, k);

        currentSettings = qwiic.readRegister(_i2caddr, AK975X_ETH24H_LOW);
        //Serial.print("AK975X_ETH24H_LOW : ");
        //Serial.println(currentSettings,BIN);

        currentSettings = qwiic.readRegister(_i2caddr, AK975X_ETH24H_HIGH);
        //Serial.print("AK975X_ETH24H_HIGH : ");
        //Serial.println(currentSettings,BIN);

    } else {

        qwiic.writeRegister(_i2caddr, AK975X_ETH24L_LOW, b);
        qwiic.writeRegister(_i2caddr, AK975X_ETH24L_HIGH, k);

        currentSettings = qwiic.readRegister(_i2caddr, AK975X_ETH24L_LOW);
        //Serial.print("AK975X_ETH24L_LOW : ");
        //Serial.println(currentSettings,BIN);

        currentSettings = qwiic.readRegister(_i2caddr, AK975X_ETH24L_HIGH);
        //Serial.print("AK975X_ETH24L_HIGH : ");
        //Serial.println(currentSettings,BIN);
    }
}

void AK975X::setThresholdEepromIr2Ir4(bool weight, int v) {

    int h=v >>6; //high bits [11;6]
    v &= 0b00111111; // low bits
    h &= 0b00111111; // high bits
    v |= 0b11000000; // mask
    h |= 0b11000000; // mask

    uint8_t currentSettings;
    uint8_t currentMode = qwiic.readRegister(_i2caddr, AK975X_ECNTL1);
    uint8_t temp = EEPROM_MODE | (currentMode & 00111000);

    qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, temp); // open EEPROM mode  

    if (weight) {

        qwiic.writeRegister(_i2caddr, AK975X_EKEY, EKEY_ON); // allow EEPROM writing

        // low uint8_t is at adresse 0x55 OR 0X57 11+[6:0] for ETH24H OR ETH24L
        qwiic.writeRegister(_i2caddr, AK975X_EETH24H_LOW, v); // ETH24H low
        sleep_ms(15); //  EEPROM write time (tWR) should be longer than 10ms

        qwiic.writeRegister(_i2caddr, AK975X_EKEY, EKEY_ON); // allow EEPROM writing

        // high uint8_t is at adresse 0x56 OR 0X58 11+[6:0] for ETH24H OR ETH24L
        qwiic.writeRegister(_i2caddr, AK975X_EETH24H_HIGH, h); // ETH24H High
        sleep_ms(15); //  EEPROM write time (tWR) should be longer than 10ms

        currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH24H_LOW);
        //Serial.print("AK975X_EETH24H_LOW : ");
        //Serial.println(currentSettings,BIN);

        currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH24H_HIGH);
        //Serial.print("AK975X_EETH24H_HIGH : ");
        //Serial.println(currentSettings,BIN);

    } else {

        qwiic.writeRegister(_i2caddr, AK975X_EKEY, EKEY_ON); // allow EEPROM writing

        // low uint8_t is at adresse 0x55 OR 0X57 11+[6:0] for ETH24H OR ETH24L
        qwiic.writeRegister(_i2caddr, AK975X_EETH24L_LOW, v); // ETH24L low
        sleep_ms(15); //  EEPROM write time (tWR) should be longer than 10ms

        qwiic.writeRegister(_i2caddr, AK975X_EKEY, EKEY_ON); // allow EEPROM writing

        // high uint8_t is at adresse 0x56 OR 0X58 11+[6:0] for ETH24H OR ETH24L
        qwiic.writeRegister(_i2caddr, AK975X_EETH24L_HIGH, h); // ETH24L High
        sleep_ms(15); //  EEPROM write time (tWR) should be longer than 10ms

        currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH24L_LOW);
        //Serial.print("AK975X_EETH24L_LOW : ");
        //Serial.println(currentSettings,BIN);

        currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH24L_HIGH);
        //Serial.print("AK975X_EETH24L_HIGH : ");
        //Serial.println(currentSettings,BIN);

    }

    qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, currentMode); // close EEPROM mode  
}

void AK975X::setThresholdIr1Ir3(bool weight, int v) {

  int k=v >>5; //high bits [11;5]
  int b=v <<3; // low bits
  b &= 0b11111000; // low bits
  k &= 0b01111111; // high bits
  uint8_t currentSettings;

  if (weight) {

    qwiic.writeRegister(_i2caddr, AK975X_ETH13H_LOW, b);
    qwiic.writeRegister(_i2caddr, AK975X_ETH13H_HIGH, k);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH13H_LOW);
    //Serial.print("AK975X_ETH13H_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH13H_HIGH);
    //Serial.print("AK975X_ETH13H_HIGH : ");
    //Serial.println(currentSettings,BIN);

  } else {

    qwiic.writeRegister(_i2caddr, AK975X_ETH13L_LOW, b);
    qwiic.writeRegister(_i2caddr, AK975X_ETH13L_HIGH, k);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH13L_LOW);
    //Serial.print("AK975X_ETH13L_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH13L_HIGH);
    //Serial.print("AK975X_ETH13L_HIGH : ");
    //Serial.println(currentSettings,BIN);
  } 
}

void AK975X::setThresholdEepromIr1Ir3(bool weight, int v) {

  int h=v >>6; //high bits [11;6]
  v &= 0b00111111; // low bits
  h &= 0b00111111; // high bits
  v |= 0b11000000; // mask
  h |= 0b11000000; // mask

  uint8_t currentSettings;
  uint8_t currentMode = qwiic.readRegister(_i2caddr, AK975X_ECNTL1);
  uint8_t temp = EEPROM_MODE | (currentMode & 00111000);

  qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, temp); // open EEPROM mode  

  if (weight) {

    qwiic.writeRegister(_i2caddr, AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // low uint8_t is at adresse 0x52 OR 0X54 11+[6:0] for ETH13H OR ETH13L
    qwiic.writeRegister(_i2caddr, AK975X_EETH13H_LOW, v); // ETH13H low
    sleep_ms(15); //  EEPROM write time (tWR) should be longer than 10ms
    qwiic.writeRegister(_i2caddr, AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // high uint8_t is at adresse 0x51 OR 0X53 11+[6:0] for ETH13H OR ETH13L
    qwiic.writeRegister(_i2caddr, AK975X_EETH13H_HIGH, h); // ETH13H High
    sleep_ms(15); //  EEPROM write time (tWR) should be longer than 10ms

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH13H_LOW);
    //Serial.print("AK975X_EETH13H_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH13H_HIGH);
    //Serial.print("AK975X_EETH13H_HIGH : ");
    //Serial.println(currentSettings,BIN);

  } else {

    qwiic.writeRegister(_i2caddr, AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // low uint8_t is at adresse 0x52 OR 0X54 11+[6:0] for ETH13H OR ETH13L
    qwiic.writeRegister(_i2caddr, AK975X_EETH13L_LOW, v); // ETH13L low
    sleep_ms(15); //  EEPROM write time (tWR) should be longer than 10ms

    qwiic.writeRegister(_i2caddr, AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // high uint8_t is at adresse 0x51 OR 0X53 11+[6:0] for ETH13H OR ETH13L
    qwiic.writeRegister(_i2caddr, AK975X_EETH13L_HIGH, h); // ETH13L High
    sleep_ms(15); //  EEPROM write time (tWR) should be longer than 10ms

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH13L_LOW);
    //Serial.print("AK975X_EETH13L_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH13L_HIGH);
    //Serial.print("AK975X_EETH13L_HIGH : ");
    //Serial.println(currentSettings,BIN);

  }

  qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, currentMode); // close EEPROM mode  
}

void AK975X::readThreshold() {
  uint8_t currentSettings;

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH24H_LOW);
  //Serial.print("AK975X_ETH24H_LOW : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH24H_HIGH);
  //Serial.print("AK975X_ETH24H_HIGH : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH24L_LOW);
  //Serial.print("AK975X_ETH24L_LOW : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH24L_HIGH);
  //Serial.print("AK975X_ETH24L_HIGH : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH13H_LOW);
  //Serial.print("AK975X_ETH13H_LOW : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH13H_HIGH);
  //Serial.print("AK975X_ETH13H_HIGH : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH13L_LOW);
  //Serial.print("AK975X_ETH13L_LOW : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_ETH13L_HIGH);
  //Serial.print("AK975X_ETH13L_HIGH : ");
  //Serial.println(currentSettings,BIN);
}

void AK975X::readThresholdEeprom() {
  uint8_t currentSettings;
  uint8_t currentMode = qwiic.readRegister(_i2caddr, AK975X_ECNTL1);
  uint8_t temp = EEPROM_MODE | (currentMode & 00111000);

  qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, temp); // open EEPROM mode  

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH24H_LOW);
    //Serial.print("AK975X_EETH24H_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH24H_HIGH);
    //Serial.print("AK975X_EETH24H_HIGH : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH24L_LOW);
    //Serial.print("AK975X_EETH24L_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH24L_HIGH);
    //Serial.print("AK975X_EETH24L_HIGH : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH13H_LOW);
    //Serial.print("AK975X_EETH13H_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH13H_HIGH);
    //Serial.print("AK975X_EETH13H_HIGH : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH13L_LOW);
    //Serial.print("AK975X_EETH13L_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=qwiic.readRegister(_i2caddr, AK975X_EETH13L_HIGH);
    //Serial.print("AK975X_EETH13L_HIGH : ");
    //Serial.println(currentSettings,BIN);    

  qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, currentMode); // close EEPROM mode  
}

void AK975X::setHysteresisIr2Ir4(int v) {

  v &= 0b00011111; // we keep the 5 last bits
  v |= 0b11100000; // mask

  uint8_t currentSettings;

  qwiic.writeRegister(_i2caddr, AK975X_EHYS24, v); // EHYS24 hysteresis level
  currentSettings=qwiic.readRegister(_i2caddr, AK975X_EHYS24);
  //Serial.print("AK975X_EHYS24 : ");
  //Serial.println(currentSettings,BIN);
}

void AK975X::setHysteresisEepromIr2Ir4(int v) {

  v &= 0b00011111; // we keep the 5 last bits
  v |= 0b11100000; // mask

  uint8_t currentSettings;
  uint8_t currentMode = qwiic.readRegister(_i2caddr, AK975X_ECNTL1);
  uint8_t temp = EEPROM_MODE | (currentMode & 00111000);

  qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, temp); // open EEPROM mode  
  qwiic.writeRegister(_i2caddr, AK975X_EKEY, EKEY_ON); // allow EEPROM writing

  qwiic.writeRegister(_i2caddr, AK975X_EEHYS24, v); // EHY24 hysteresis level
  sleep_ms(15);

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_EEHYS24);
  //Serial.print("AK975X_EEHYS24 : ");
  //Serial.println(currentSettings,BIN);

  qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, currentMode); // close EEPROM mode  

}

void AK975X::setHysteresisIr1Ir3(int v) {

  v &= 0b00011111; // we keep the 5 last bits
  v |= 0b11100000; // mask

  uint8_t currentSettings;

  qwiic.writeRegister(_i2caddr, AK975X_EHYS13, v); // EHYS13 hysteresis level
  currentSettings=qwiic.readRegister(_i2caddr, AK975X_EHYS13);
  //Serial.print("AK975X_EHYS13 : ");
  //Serial.println(currentSettings,BIN);
}

void AK975X::setHysteresisEepromIr1Ir3(int v) {

  v &= 0b00011111; // we keep the 5 last bits
  v |= 0b11100000; // mask

  uint8_t currentSettings;
  uint8_t currentMode = qwiic.readRegister(_i2caddr, AK975X_ECNTL1);
  uint8_t temp = EEPROM_MODE | (currentMode & 00111000);

  qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, temp); // open EEPROM mode  
  qwiic.writeRegister(_i2caddr, AK975X_EKEY, EKEY_ON); // allow EEPROM writing

  qwiic.writeRegister(_i2caddr, AK975X_EEHYS13, v); // EHY13 hysteresis level
  sleep_ms(15); 

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_EEHYS13);
  //Serial.print("AK975X_EEHYS13 : ");
  //Serial.println(currentSettings,BIN);

  qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, currentMode); // close EEPROM mode  
}

void AK975X::readHysteresis() {

  uint8_t currentSettings;

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_EHYS24);
  //Serial.print("AK975X_EHYS24 : ");
  //Serial.println(currentSettings,BIN);
  currentSettings=qwiic.readRegister(_i2caddr, AK975X_EHYS13);
  //Serial.print("AK975X_EHYS13 : ");
  //Serial.println(currentSettings,BIN);
}

void AK975X::readHysteresisEeprom() {

  uint8_t currentSettings;
  uint8_t currentMode = qwiic.readRegister(_i2caddr, AK975X_ECNTL1);
  uint8_t temp = EEPROM_MODE | (currentMode & 00111000);

  qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, temp); // open EEPROM mode  

  currentSettings=qwiic.readRegister(_i2caddr, AK975X_EEHYS24);
  //Serial.print("AK975X_EEHYS24 : ");
  //Serial.println(currentSettings,BIN);
  currentSettings=qwiic.readRegister(_i2caddr, AK975X_EEHYS13);
  //Serial.print("AK975X_EEHYS13 : ");
  //Serial.println(currentSettings,BIN);


  qwiic.writeRegister(_i2caddr, AK975X_ECNTL1, currentMode); // close EEPROM mode  
}

void AK975X::setInterrupts(bool ir13h,bool ir13l,bool ir24h,bool ir24l,bool dr) {
  // mask , assembly of AK975X_EINTEN bits
  int v = 0b11000000 | ( ir13h<<4 | ir13l<<3 | ir24h<<2 | ir24l<<1 | dr);
  qwiic.writeRegister(_i2caddr, AK975X_EINTEN, v); 

  uint8_t currentSettings=qwiic.readRegister(_i2caddr, AK975X_EINTEN);
  //Serial.print("AK975X_EINTEN : ");
  //Serial.println(currentSettings,BIN);  
}

int AK975X::readInterruptStatus() {

  uint8_t currentSettings=qwiic.readRegister(_i2caddr, AK975X_INTST);

  // no settings, no dice
  if(currentSettings & 0b00011111 == 0)
      return 0;

  // look at bits 0-4: if set, return that value.
  for(int i=0; i < 5; i++ ){
     if(currentSettings & (0x1 << i))
        return i + (i == 0) * 5;   // bit 0 set, return 5, otherwise return bit number
  }
  // should never get here ...
  return 0;

}

//Checks to see if DRDY flag is set in the status register
bool AK975X::available()
{
  uint8_t value = qwiic.readRegister(_i2caddr, AK975X_ST1);
  return (value & (1 << 0)); //Bit 0 is DRDY
}

//Checks to see if Data overrun flag is set in the status register
bool AK975X::overrun()
{
  uint8_t value = qwiic.readRegister(_i2caddr, AK975X_ST1);

  return (value & 1 << 1); //Bit 1 is DOR
}

//Does a soft reset
void AK975X::softReset()
{
  qwiic.writeRegister(_i2caddr, AK975X_CNTL2, 0xFF);
}

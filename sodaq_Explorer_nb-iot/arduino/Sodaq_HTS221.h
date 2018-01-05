/***************************************************************************
    This is a library for the HTS221 Humidity Temperature Sensor
        Originally written by speirano for SmartEverything
        Adjusted by Gregory Knauff of SODAQ for the NB-IoT shield

***************************************************************************/


#ifndef Sodaq_HTS221_H_
#define Sodaq_HTS221_H_

#include <Arduino.h>


class Sodaq_HTS221
{
public:
    Sodaq_HTS221(void);
    bool begin(void);
    bool activate(void);
    bool deactivate(void);

    bool bduActivate(void);
    bool bduDeactivate(void);

    const double readHumidity(void);
    const double readTemperature(void);
private:
    bool storeCalibration(void);
    unsigned char _h0_rH, _h1_rH;
    unsigned int  _T0_degC, _T1_degC;
    unsigned int  _H0_T0, _H1_T0;
    unsigned int  _T0_OUT, _T1_OUT;
    double _temperature;
    double _humidity;
    uint8_t _address;

    byte readRegister(byte slaveAddress, byte regToRead);
    bool writeRegister(byte slaveAddress, byte regToWrite, byte dataToWrite);
};


extern Sodaq_HTS221 hts221;


#endif /* Sodaq_HTS221_H_ */

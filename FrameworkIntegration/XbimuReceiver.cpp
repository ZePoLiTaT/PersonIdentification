/*
    XbimuReceiver.cpp
    Author: Seb Madgwick

    C++ library for receiving data from the x-BIMU.  Only supports binary
    packet mode, 'binary packet mode' must be enabled in x-BIMU settings
    (BP = 1).

    See x_BIMU_Arduino_Example.ino for example usage.
*/

//------------------------------------------------------------------------------
// Includes

#include "XbimuReceiver.h"
#include<iostream>
//------------------------------------------------------------------------------
// Definitions
bool GetList (const std::string& src, std::vector<std::string>& res)
  {
    using boost::lexical_cast;
    using boost::bad_lexical_cast;
    bool success = true;
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sepa(",");
    tokenizer tokens(src, sepa);
    for (tokenizer::iterator tok_iter = tokens.begin(); 
         tok_iter != tokens.end(); ++tok_iter) 
	{
      try 
	  {
        res.push_back((*tok_iter));
      }
      catch (bad_lexical_cast &) 
	  {
        success = false;
      }
    }
    return success;
  }
enum {
    QUAT_LENGTH = 11,
    SENS_LENGTH = 21,
    BATT_LENGTH = 5,
    MAX_LENGTH = 21 /* maximum packet length of all packet types */
};

//------------------------------------------------------------------------------
// Variables

// Serial stream decoding
unsigned char buf[256];
unsigned char bufIndex = 0;
unsigned char bufCount = 0;
bool inSync = false;

// Decoded data
QuaternionStruct quaternionStruct = { 0, 0, 0, 0, 0 };
SensorStruct sensorStruct = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
BatteryStruct batteryStruct = { 0, 0 };

// Data ready flags
bool quaternionGetReady = false;
bool sensorGetReady = false;
bool batteryGetReady = false;

//------------------------------------------------------------------------------
// Methods

void XbimuReceiver::processNewCharASCII(unsigned char c)
{
	int j=(int)c;

	if (j == 13)
	{
		{
                
                    // Split string to comma separated variables
                    std::vector<std::string> vars=std::vector<std::string>(); 

                    // Validate checksum (http://en.wikipedia.org/wiki/Longitudinal_redundancy_check)
                    unsigned char checksum = 0;
					if(GetList(asciiBuf,vars))
					{
						if(vars.size()>=5 && vars[0]=="Q")
						{
							try
							{
								quaternionStruct.w =  boost::lexical_cast<int>(vars[1]);
								quaternionStruct.x = -boost::lexical_cast<int>(vars[2]);
								quaternionStruct.y = -boost::lexical_cast<int>(vars[4]);
								quaternionStruct.z = -boost::lexical_cast<int>(vars[3]);
								quaternionGetReady = true;
							}
							catch(...)
							{
							}
						}
					}

                    //for (int i = 0; i <= asciiBuf.LastIndexOf(','); i++)    // checksum does not include checksum characters
                    //{
                    //    checksum ^= (byte)asciiBuf[i];
                    //}
                    //if (checksum != byte.Parse(vars[vars.Length - 1]))
                    //{
                    //    throw new Exception();  // checksum failed
                    //}

                    //// Decode according to packet header
                    //switch (vars[0])
                    //{
                    //    case ("Q"):
                    //        OnQuaternionReceived(new int[] { Int32.Parse(vars[1]), Int32.Parse(vars[2]), Int32.Parse(vars[3]), Int32.Parse(vars[4]),    /* quaternion elements 0, 1, 2, 3   */
                    //                                         Int32.Parse(vars[5]) });                                                                   /* counter                          */
                    //        break;
                    //    case ("S"):
                    //        OnSensorsReceived(new int[] { Int32.Parse(vars[1]), Int32.Parse(vars[2]), Int32.Parse(vars[3]), /* gyroscope, X, Y, Z axes      */
                    //                                      Int32.Parse(vars[4]), Int32.Parse(vars[5]), Int32.Parse(vars[6]), /* acceleroemter, X, Y, Z axes  */
                    //                                      Int32.Parse(vars[7]), Int32.Parse(vars[8]), Int32.Parse(vars[9]), /* magnetometer, X, Y, Z axes   */
                    //                                      Int32.Parse(vars[10]) });                                         /* counter                      */
                    //        break;
                    //    case ("B"):
                    //        OnBatteryReceived(new int[] { Int32.Parse(vars[1]),     /* battery voltage */
                    //                                      Int32.Parse(vars[2]) });  /* counter          */
                    //        break;
                    //    default:
                    //        throw new Exception();
                    //}
                }
               
                asciiBuf = "";
            }
	
            
    else
    {
        asciiBuf.push_back(c);
    }
}
XbimuReceiver::XbimuReceiver() 
{
	asciiBuf=std::string();
	//asciiBuf+=asciiBuf;
}


void XbimuReceiver::processNewCharBinary(unsigned char c) 
{

    // Add new byte to buffer
    buf[bufIndex++] = c;

    // Check if out of sync
    bufCount++;
    if (bufCount > MAX_LENGTH) 
	{
        bufCount = MAX_LENGTH; // prevent overflow
        inSync = false;
    }
	
    // Decode quaternion packet
    if (bufIndex >= QUAT_LENGTH) 
	{
        if ((inSync ? (char)buf[0] : (char)buf[bufIndex - QUAT_LENGTH]) == 'Q') 
		{
            if (calcChecksum(QUAT_LENGTH) != 0) 
			{
                quaternionStruct.w = (int)(((int)buf[bufIndex - 10] << 8) | (int)buf[bufIndex - 9]);
                quaternionStruct.x = (int)(((int)buf[bufIndex - 8] << 8) | (int)buf[bufIndex - 7]);
                quaternionStruct.y = (int)(((int)buf[bufIndex - 6] << 8) | (int)buf[bufIndex - 5]);
                quaternionStruct.z = (int)(((int)buf[bufIndex - 4] << 8) | (int)buf[bufIndex - 3]);
                quaternionStruct.counter = buf[bufIndex - 2];
                quaternionGetReady = true;
                bufIndex = 0;
                bufCount = 0;
                inSync = true;
            }
        }
    }

    // Decode sensor packet
    if (bufIndex >= SENS_LENGTH) 
	{
        if ((inSync ? (char)buf[0] : (char)buf[bufIndex - SENS_LENGTH]) == 'S') 
		{
            if (calcChecksum(SENS_LENGTH) == 0) {
                sensorStruct.gyrX = (int)(((int)buf[bufIndex - 20] << 8) | (int)buf[bufIndex - 19]);
                sensorStruct.gyrY = (int)(((int)buf[bufIndex - 18] << 8) | (int)buf[bufIndex - 17]);
                sensorStruct.gyrZ = (int)(((int)buf[bufIndex - 16] << 8) | (int)buf[bufIndex - 15]);
                sensorStruct.accX = (int)(((int)buf[bufIndex - 14] << 8) | (int)buf[bufIndex - 13]);
                sensorStruct.accY = (int)(((int)buf[bufIndex - 12] << 8) | (int)buf[bufIndex - 11]);
                sensorStruct.accZ = (int)(((int)buf[bufIndex - 10] << 8) | (int)buf[bufIndex - 9]);
                sensorStruct.magX = (int)(((int)buf[bufIndex - 8] << 8) | (int)buf[bufIndex - 7]);
                sensorStruct.magY = (int)(((int)buf[bufIndex - 6] << 8) | (int)buf[bufIndex - 5]);
                sensorStruct.magZ = (int)(((int)buf[bufIndex - 4] << 8) | (int)buf[bufIndex - 3]);
                sensorStruct.counter = buf[bufIndex - 2];
                sensorGetReady = true;
                bufIndex = 0;
                bufCount = 0;
                inSync = true;
            }
        }
    }

    // Decode battery packet
    if (bufIndex >= BATT_LENGTH) 
	{
        if ((inSync ? (char)buf[0] : (char)buf[bufIndex - BATT_LENGTH]) == 'B') {
            if (calcChecksum(BATT_LENGTH) == 0) {
                batteryStruct.voltage = (int)(((int)buf[bufIndex - 4] << 8) | (int)buf[bufIndex - 3]);
                batteryStruct.counter = buf[bufIndex - 2];
                batteryGetReady = true;
                bufIndex = 0;
                bufCount = 0;
                inSync = true;
            }
        }
    }
}

unsigned char XbimuReceiver::calcChecksum(unsigned char packetLength) const {
    unsigned char tempRxBufIndex = (unsigned char)(bufIndex - packetLength);
    unsigned char checksum = 0;
    while (tempRxBufIndex != bufIndex) {
        checksum ^= buf[tempRxBufIndex++];
    }
    return checksum;
}

bool XbimuReceiver::isQuaternionGetReady(void) const 
{
    return quaternionGetReady;
}

bool XbimuReceiver::isSensorGetReady(void) const 
{
    return sensorGetReady;
}

bool XbimuReceiver::isBatteryGetReady(void) const 
{
    return batteryGetReady;
}

QuaternionStruct XbimuReceiver::getQuaternion(void) 
{
    quaternionGetReady = false;
    return quaternionStruct;
}

SensorStruct XbimuReceiver::getSensor(void) 
{
    sensorGetReady = false;
    return sensorStruct;
}

BatteryStruct XbimuReceiver::getBattery(void) 
{
    batteryGetReady = false;
    return batteryStruct;
}
void XbimuReceiver::getQuatValues(float &w, float &x,float &y,float &z)
{
	w=(float)quaternionStruct.w;
	x=(float)quaternionStruct.x;
	y=(float)quaternionStruct.y;
	z=(float)quaternionStruct.z;
}
//------------------------------------------------------------------------------
// End of file
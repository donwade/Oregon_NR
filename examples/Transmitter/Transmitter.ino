#include <Oregon_TM.h>

//Oregon Scientific Temperature and Humidity Sensor Simulator

//In this example, packets of the following types of sensors are simulated:
//THGN132N (type 1D20h, v2 protocol, 3 channels)
//RTGN318 (type ХСС3h, protocol v2, 5 channels)
//THGR810 (F824h type, v3 protocol, 10 channels)
//BTHR968 (type 5D60h, v2 protocol, 1 channel)
//BTHGN129 (type 5D53h, v2 protocol, 5 channels)
---------------------------------------------------------------------------------------------------------------------------------------------------
//The transmitter output is one. Specified only when creating the first object. In this example, the transmitter is connected to D4

Oregon_TM transmitter(4,19), transmitter2(4,19), transmitter3(4,19), transmitter4(4, 24), transmitter5(4, 24); //it is enough to indicate the output of the transmitter once,
//Oregon_TM transmitter (4), transmitter2, transmitter3, transmitter4, transmitter5; // transmitter output is enough to specify once,

---------------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);     
  
  //THGN132
  transmitter.setType(THGN132);
  transmitter.setChannel(3);        //Channel number for THGN132 - 1 ... 3
  transmitter.setBatteryFlag(1);    //Low battery flag
  transmitter.setTemperature(24.2); // -49.9C...+69.9C
  transmitter.setHumidity(30);      // 2...98%
  transmitter.setComfort(24.2, 30); //Calculation of the transmitted comfort index
  transmitter.buffer_size = 19;

  //THGR810
  transmitter2.setType(THGR810);    
  transmitter2.setChannel(1);       //The channel number for THGR810 is 1 ... 10 (It is possible that only the first 5 channels will be received by the base. I have nothing to check ...)
  transmitter2.setBatteryFlag(1); 
  transmitter2.setTemperature(24.3); // -49.9C...+69.9C
  transmitter2.setHumidity(50);      // 2...98%
  transmitter2.setComfort(24.3, 50); //Calculation of the transmitted comfort index
  transmitter2.buffer_size = 19;

  //RTGN318
  transmitter3.setType(RTGN318);
  transmitter3.setChannel(5);       //The channel number for RTGN318 is 1 ... 5.
  transmitter3.setBatteryFlag(0); 
  transmitter3.setTemperature(-24.2); // -49.9C...+69.9C
  transmitter3.setHumidity(98);      // 2...98%
  transmitter3.setComfort(-24.2, 98); //Calculation of the transmitted comfort index
  transmitter3.buffer_size = 19;

  //Third transmitter
  transmitter4.setType(BTHGN129);
  transmitter4.setChannel(1);       //The channel number for BTHGN129 is 1 ... 5.
  transmitter4.setBatteryFlag(0); 
  transmitter4.setTemperature(-49.9); // -49.9C...+69.9C
  transmitter4.setHumidity(3);      // 2...98%
  transmitter4.setComfort(-49.9, 3); //Calculation of the transmitted comfort index
  transmitter4.setPressure(760.0); 
  transmitter4.buffer_size = 23;

  //Third transmitter
  transmitter5.setType(BTHR968);
  transmitter5.setChannel(0);       //BTHR968 has 1 channel, but you need to call the procedure
  transmitter5.setBatteryFlag(0); 
  transmitter5.setTemperature(49.9); // -49.9C...+69.9C
  transmitter5.setHumidity(97);      // 2...98%
  transmitter5.setComfort(49.9, 97); //Calculation of the transmitted comfort index
  transmitter5.setPressure(785.0); 
  transmitter5.buffer_size = 23;
}

---------------------------------------------------------------------------------------------------------------------------------------------------
void loop()
{
    //transmission is carried out by a timer, which is determined by the type of sensor and channel number
    
    if (transmitter.transmit()) PrintSentData(transmitter.SendBuffer, transmitter.buffer_size);
    if (transmitter2.transmit()) PrintSentData(transmitter2.SendBuffer, transmitter2.buffer_size);
    if (transmitter3.transmit()) PrintSentData(transmitter3.SendBuffer, transmitter3.buffer_size);
    if (transmitter4.transmit()) PrintSentData(transmitter4.SendBuffer, transmitter4.buffer_size);
    if (transmitter5.transmit()) PrintSentData(transmitter5.SendBuffer, transmitter5.buffer_size);
}

---------------------------------------------------------------------------------------------------------------------------------------------------
void PrintSentData(byte* buf, int buf_size)
{
      Serial.print(millis() / 1000);
      Serial.print("s \t\t");
      for (byte i = 0; i < buf_size; i++)   
      {
        byte trmbuf = *buf;
        Serial.print(trmbuf >> 4, HEX);
        i++;
        if (i >= buf_size) break;
        Serial.print(trmbuf & 0x0F, HEX);
        buf++;
      }  
      Serial.println();
      delay(1000);
}

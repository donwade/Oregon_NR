#include <Wire.h>
#include <Oregon_TM.h>
#include <BME280I2C.h>

------------------------------------------------------------------------------------------------------------
//Oregon Scientific THGN132N BME280 Transmitter Sketch
//The schematic diagram is attached.
//To work, you need a library https://github.com/finitespace/BME280-
//The device is powered by 3 AA batteries, to save electricity, you need to fill in the sketch via ISP
------------------------------------------------------------------------------------------------------------
//It is also possible to transfer data in the format - THP (temperature, humidity, pressure, battery voltage)
//Receiver example supports THP decoding
------------------------------------------------------------------------------------------------------------

# define THGN_SEND        0  //Whether to transmit data in THGN132 format
# define THP_SEND         1  //Whether to transmit data in THP format
# define DEVICE_LOG       0   //Whether to write a log to Serial

# define DONE_PIN         15      //output of the signal about the end of work on the timer

# define BME_WAIT         10      //How many ms to wait for BME sensor
# define BATTERY_THR      3.5     //Voltage threshold for setting the low battery flag (THGN)

-------------------------------------------------------------------------------------------------
//THP sensor nibles
//Minor nibls go forward in all fields !!!
//1-2 - type (55)
//3 - channel (0-7)
// 4-6    - (температура от -100С) * 10. Т.е. +25.1С = 1251 = 4E3h 
// 7-9    - Влажность *10 Т.е. 25.1% = 251 = 0FBh
// 10-12  - (давление от 500ммртст) * 10. Т.е. 765мм = 2650 = A5Ah
//13-15 - data from ADC (A0)
// 16-17  - CheckSUM 
// 18-19  - CRC8 (poly 0x07 start 0x00)
-------------------------------------------------------------------------------------------------

Oregon_TM transmitter(4);
BME280I2C bme;  

bool  bme_present = false;
float bme_temp(NAN), bme_hum(NAN), bme_pres(NAN);
-------------------------------------------------------------------------------------------------

void setup()
  {
    digitalWrite(DONE_PIN, LOW);   
    pinMode(DONE_PIN, OUTPUT);
    
#ifdef DEVICE_LOG
    Serial.begin(115200);     
    Serial.println("Waiting for BMEsensor...");
#endif

    //Communication with BME -------------------------------- / -
    Wire.begin();
    while(!bme.begin())
    {
      if (millis() > BME_WAIT) break;
    }
    if (!bme.begin())
    {
#ifdef DEVICE_LOG                             
      Serial.println("No BME sensor found");
#endif      
      bme_present = false;
    }
    else
    {
      switch(bme.chipModel())
      {
        case BME280::ChipModel_BME280:
           bme_present = true;
           bme.read(bme_pres, bme_temp, bme_hum);
#ifdef DEVICE_LOG                       
          Serial.println("Found BME280 sensor! Success.");
          Serial.print("Temperature = ");
          Serial.print(bme_temp, 1);
          Serial.println("C");
          Serial.print("Humidity = ");
          Serial.print(bme_hum, 1);
          Serial.println("%");
          Serial.print("Pressure = ");
          Serial.print(bme_pres * 0.75, 1);
          Serial.println("mmHg");
#endif                     
          break;
        default:
#ifdef DEVICE_LOG                       
            Serial.println("Found UNKNOWN sensor! Error!");
#endif
            bme_present = false;
      }
    }

    //Battery Voltages -------------------------------------------
    word battvotage = (word)(((float)(1.1 * 16368) / Vbg()) * 100);
#ifdef DEVICE_LOG
    Serial.print("Battery voltage = ");
    Serial.println(battvotage,HEX);
#endif
    
    //Preparing and Sending THGN Data ------------------------------------ / -
    transmitter.protocol == 2;
    if (THGN_SEND)
    {
      transmitter.setType(THGN132);
      transmitter.setChannel(3);   
      transmitter.setBatteryFlag(battvotage < BATTERY_THR); 
      if (bme_present)
      {
        if (bme_hum > 98) bme_hum = 98;
        if (bme_hum < 2) bme_hum = 2;
        if (bme_temp > 70) bme_temp = 70;
        if (bme_temp < -50) bme_temp = -50;
        transmitter.setTemperature(bme_temp);
        transmitter.setHumidity(bme_hum);  
        transmitter.setComfort(bme_temp, bme_hum);
      }
      else
      {
        transmitter.setTemperature(-49.9);
        transmitter.setHumidity(2);  
        transmitter.setComfort(-49.9, 2);
      }
      transmitter.SendPacket();
    }
    
    //If both packet formats are sent, a pause must be paused between them.
    if (THP_SEND && THGN_SEND) delay(100);
    
    //Preparing and sending THP data ------------------------------------ / -
    if (THP_SEND)
    {
      transmitter.setType(THP);
      transmitter.setChannelTHP(1);         
      transmitter.setBatteryTHP( battvotage);
      if (bme_present)
      {
        transmitter.setTemperatureTHP(bme_temp);
        transmitter.setHumidityTHP(bme_hum);  
        transmitter.setPressureTHP(bme_pres * 0.75);  //converting Pa to mmHg
      }
      else
      {
        transmitter.setErrorTHP();  
      }
      transmitter.SendPacket();
    }
    
#ifdef DEVICE_LOG    
    Serial.println();
    Serial.print(millis());
    Serial.println("ms");
    Serial.println();
#endif
    //Power off command
    digitalWrite(DONE_PIN, HIGH);
  }
-------------------------------------------------------------------------------------------------
void loop(){}
-------------------------------------------------------------------------------------------------
int Vbg() { 
  ADMUX = (1<<REFS0)|(0<<REFS1)|(1<<MUX3)|(1<<MUX2)|(1<<MUX1)|(0<<MUX0);
  long buffersamp=0;
  for (int n=0x0; n<=0xff; n++ ) {
  ADCSRA = 0xc7;
  while (bit_is_set(ADCSRA,ADSC));
  buffersamp += ADC; }
  buffersamp >>=4; //16368 full scale 14bit
  ADCSRA &= ~(1 << ADEN);  //turn off the ADC
  return buffersamp;
}

#include <Oregon_NR.h>
#include <time.h>
#include <Udp.h>

int log2SD ( const char * format, ... );

void setupWiFi();
void setupFS();

#ifndef ST1005
#define ST1005 0x12345
#define ST1004 0x54321
#endif

// need to know which of N temperature sensors is outside (for windchill)
#define OUTSIDE_TMPID 0xF5
#define OUTSIDE_WGRID 0xF0  // I have 2. This is the one I want for wind speed


//---------------------------------------
typedef struct tempStat_s {
    tempStat_s *next;
    uint8_t channel;
    float outsideTempNow;
    float outsideTempLo;
    float outsideTempHi;
    time_t outsideTempLoTime;
    time_t outsideTempHiTime;
    float outsideWindNow;
    float outsideWindHi;
    time_t outsideWindTime;

} tempStat;

tempStat *first = NULL;

//---------------------------------------
tempStat * locateStats(uint8_t sens_id)
{
   tempStat *ret = first;
   tempStat *lastRet = NULL;

    if (!first)
        {
           ret = (tempStat *) malloc (sizeof (tempStat));
           first = ret;
           memset(ret, 0, sizeof(tempStat));
init:
          time_t tick;
          time(&tick);

           ret->channel = sens_id;
           ret->outsideTempHi = -1000.;
           ret->outsideTempLo= +1000.;
           ret->outsideTempHiTime = tick;
           ret->outsideTempLoTime = tick;
           ret->outsideWindTime = tick;

           return ret;
        }
    else
        {
            lastRet = ret;
            while( ret && ret->channel != sens_id)
            {
              lastRet = ret;
              ret = ret->next;
            }

            if (!ret)
            {
                ret = (tempStat *) malloc (sizeof (tempStat));
                memset(ret, 0, sizeof(tempStat));
                lastRet->next = ret;
                goto init;
            }
        }
    return ret;
}

//---------------------------------------
float windchill (float speed, float temperature)
{
    //Serial.printf("%s : speed = %f temperature = %f\n", __FUNCTION__, speed, temperature);
    float chill = 13.12 + 0.6215 * temperature -  11.37 * pow(speed,0.16) + 0.3965 * temperature * pow(speed,0.16);
    return chill;
}

//---------------------------------------
#define DUMB433RxPin 34  // was 13 but ttgo SD card takes 12,13,14

#if defined ( ESP8266 ) || ( ESP32 )//For Wemos
  Oregon_NR oregon(DUMB433RxPin, DUMB433RxPin,          //receiver on pin
                        2, true,    //The LED on D2 is pulled up to + pit (true). If the LED is not needed, then the pin number is 255
                        50, true);  //Buffer for receiving a parcel from 50 nibls, package assembly for v2 is included

#else                               //For AVR
  Oregon_NR oregon(2, 0,            //Receiver at pin D2, Interrupt 0,
                     13, false);    //Receive LED on pin 13
                                    //By default, the Buffer for receiving a parcel is standard - for 24 nibbles, package assembly for v2 is enabled

//Oregon_NR oregon (2, 0); // Default constructor, with standard buffer and no LED
#endif

//---------------------------------------
#define MAX_WIDTH 30

void setup() {
   Serial.begin(115200);
   Serial.println();
   if (oregon.no_memory)
   {
    Serial.println("NO MEMORY");   //Out of memory
    do yield();
    while(true);
   }

  delay(1000);
  Serial.println("OK");

  setupWiFi();  // setup the time first.
  setupFS();

  //turn on listening to the radio channel
  oregon.start();
  oregon.receiver_dump = 0;       //true - Turns on "oscilloscope" - display of data received from the receiver
}

//---------------------------------------

void loop() {
  char lead[80];

  time_t now;
  time(&now);
  struct tm timeinfo;

  //Capturing a packet
  oregon.capture(false); // output service information to Serial

  //The captured data is valid until the next capture call
  //Processing the received package

  if (oregon.captured && oregon.sens_type)  {

    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
      return;
    }

    struct tm  qtime;
    char  cNowTime[80];

    // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
    qtime = *localtime(&now);

    //strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
    strftime(cNowTime, sizeof(cNowTime), "%b %d %H:%M:%S", &qtime);

    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    sprintf( lead, "%d %s", now, cNowTime);

    //Outputting information to Serial
    //log2SD ("% 6.1fs", (float) millis () / 1000, 1); //Time

    //dump Output of the received packet.
    char cDump[100];
    cDump[0] = '\0';

    int16_t tlen = 0;
    for (int q = 0;q < min(oregon.packet_length, MAX_WIDTH); q++)
    {
      if (oregon.valid_p[q] == 0x0F)
      {
        sprintf(&cDump[strlen(cDump)], "%02X", oregon.packet[q]);
        tlen++;
      }
    }
    tlen = 25 - tlen; // spaces to right
    while (tlen-- > 0 ) sprintf(&cDump[strlen(cDump)], " ");


    char cVer[6];
    //Protocol version
    if (oregon.ver == 2) strcpy (cVer, "T2 ");
    if (oregon.ver == 3) strcpy (cVer, "T3 ");
    if (oregon.ver == 11) strcpy(cVer, "TE ");
    if (oregon.ver == 12) strcpy(cVer, "E2 ");

    //Package recovery information
    char cError[5];
    cError[0] = '\0';
    if (oregon.restore_sign & 0x01) strcat(cError,"s"); //restored single ticks
    else  strcat(cError,".");
    if (oregon.restore_sign & 0x02) strcat(cError,"d"); //restored double bars
    else  strcat(cError,".");
    if (oregon.restore_sign & 0x04) strcat(cError,"p "); //fixed bug when recognizing package version
    else  strcat(cError,".");
    if (oregon.restore_sign & 0x08) strcat(cError,"r "); //built from two packages (for build mode in v.2)
    else  strcat(cError,".");


    //Package processing time

    //Serial.print(" ");
    //Serial.print(oregon.work_time);
    //Serial.print("ms ");

    char cDevName[15];
    char cChannel[10];
    char cBatID[30];
    char cSenorType[8];

    Serial.printf("Sensor = 0x%X\n", oregon.sens_type);

    strcpy(cDevName, "UNKNOWN ");
    if ((oregon.sens_type == THGN132 ||
    (oregon.sens_type & 0x0FFF) == RTGN318 ||
    (oregon.sens_type & 0x0FFF) == RTHN318 ||
    oregon.sens_type == THGR810 ||
    oregon.sens_type == THN132 ||
    oregon.sens_type == THN800 ||
    oregon.sens_type == BTHGN129 ||
    oregon.sens_type == BTHR968 ||
    oregon.sens_type == ST1004 ||
    oregon.sens_type == ST1005 ||
    oregon.sens_type == THGN500) && oregon.crc_c)
    {
      if (oregon.sens_type == ST1004) strcpy(cDevName , "ST1004   ");
      if (oregon.sens_type == ST1005) strcpy(cDevName,  "ST1005   ");
      if (oregon.sens_type == THGN132) strcpy(cDevName, "THGN132N ");
      if (oregon.sens_type == THGN500) strcpy(cDevName, "THGN500  ");
      if (oregon.sens_type == THGR810) strcpy(cDevName, "THGR810  ");
      if ((oregon.sens_type & 0x0FFF) == RTGN318) strcpy(cDevName, "RTGN318  ");
      if ((oregon.sens_type & 0x0FFF) == RTHN318) strcpy(cDevName, "RTHN318  ");
      if (oregon.sens_type == THN132 ) strcpy(cDevName,  "THN132N  ");
      if (oregon.sens_type == THN800 ) strcpy(cDevName,  "THN800   ");
      if (oregon.sens_type == BTHGN129 ) strcpy(cDevName,"BTHGN129 ");
      if (oregon.sens_type == BTHR968 ) strcpy(cDevName, "BTHR968  ");

      if (oregon.sens_type != BTHR968 && oregon.sens_type != THGN500)
      {
        sprintf(cChannel,"C=%02d ", oregon.sens_chnl);
      }
      else strcpy(cChannel,("    "));

      sprintf(cBatID, "%s BAT: %s  ID: %x02", cError, (oregon.sens_battery) ? "F " : "e ", oregon.sens_id);

      tempStat *who = locateStats(oregon.sens_id);

      who->outsideTempNow = oregon.sens_tmp;

      time_t tick;
      time(&tick);
     char record = ':';

      if (who->outsideTempNow < who->outsideTempLo)
      {
          who->outsideTempLo = who->outsideTempNow;
          who->outsideTempLoTime = tick;
          record = '*';
      }

      if (who->outsideTempNow > who->outsideTempHi)
      {
        who->outsideTempHi = who->outsideTempNow;
        who->outsideTempHiTime = tick;
        record = '*';
      }

      struct tm  ts_hi;
      struct tm  ts_lo;
      char  hibuf[80];
      char  lobuf[80];

      // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
      ts_lo = *localtime(&who->outsideTempLoTime);
      ts_hi = *localtime(&who->outsideTempHiTime);

      //strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
      strftime(lobuf, sizeof(lobuf), "%m-%d %H:%M", &ts_lo);
      strftime(hibuf, sizeof(hibuf), "%m-%d %H:%M", &ts_hi);

#if 0
      if (oregon.sens_tmp >= 0 && oregon.sens_tmp < 10) Serial.print(" TMP:  ");
      if (oregon.sens_tmp < 0 && oregon.sens_tmp >-10)  Serial.print(" TMP: ");
      if (oregon.sens_tmp <= -10)                       Serial.print(" TMP:");
      if (oregon.sens_tmp >= 10)                        Serial.print(" TMP: ");
      Serial.print(oregon.sens_tmp, 1);
#endif

      char cHumidity[20] = {0};
      if (oregon.sens_type == THGN132 ||
          oregon.sens_type == THGR810 ||
          oregon.sens_type == BTHGN129 ||
          oregon.sens_type == BTHR968 ||
          (oregon.sens_type & 0x0FFF) == RTGN318 ||
          oregon.sens_type == THGN500 ||
          oregon.sens_type == ST1005 ||
          oregon.sens_type == ST1004)
          {
            sprintf(cHumidity, "HUM: %.1f%%", oregon.sens_hmdty);
          }
          else strcpy(cHumidity,"      ");

      char cPressure[25] = {0};
      if (oregon.sens_type == BTHGN129 ||  oregon.sens_type == BTHR968)
      {
        sprintf(cPressure, " PRESS: %d Hgmm", oregon.get_pressure());
      }

      //Serial.printf("xxxxx %s yyyyy\n", lead);
      log2SD("%s | %s %s %s %c TEMP : now %+05.1f C     [lo %s %+05.1f C]  [hi %s %+05.1f C ] %s %s %s\n",
          lead,
          cDevName,
          cChannel,
          cBatID,
          record,
          who->outsideTempNow,
          lobuf,
          who->outsideTempLo,
          hibuf,
          who->outsideTempHi,
          cHumidity,
          cPressure,
          "ok" //cDump
          );

      if (oregon.sens_type == ST1005 && (oregon.packet[2] & 0x04)) Serial.print(" TX MODE ");
    }

  if (oregon.sens_type == WGR800 && oregon.crc_c){

      tempStat *who =  locateStats(oregon.sens_id);
      who->outsideWindNow = oregon.sens_avg_ws * 3.6;
     strcpy(cDevName, "WGR800   ");

     char record = ':';
      if (who->outsideWindHi  <  oregon.sens_max_ws * 3.6)
      {
         time_t tick;
         time(&tick);
         who->outsideWindHi = oregon.sens_max_ws * 3.6;
         who->outsideWindTime = tick;
         record = '*';
      }

     char windDir[5];
     switch (oregon.sens_wdir)
     {
     case  0: strcpy(windDir,"N   "); break;
     case  1: strcpy(windDir,"NNE "); break;
     case  2: strcpy(windDir,"NE  "); break;
     case  3: strcpy(windDir,"NEE "); break;
     case  4: strcpy(windDir,"E   "); break;
     case  5: strcpy(windDir,"SEEE"); break;
     case  6: strcpy(windDir,"SE  "); break;
     case  7: strcpy(windDir,"SSE "); break;
     case  8: strcpy(windDir,"S   "); break;
     case  9: strcpy(windDir,"SSW "); break;
     case 10: strcpy(windDir,"SW  "); break;
     case 11: strcpy(windDir,"SWW "); break;
     case 12: strcpy(windDir,"W   "); break;
     case 13: strcpy(windDir,"NWW "); break;
     case 14: strcpy(windDir,"NW  "); break;
     case 15: strcpy(windDir,"NNW "); break;
     }


     struct tm  ts_wind;
     char  cHiWind[80];

     // Format time, "ddd yyyy-mm-dd hh:mm:ss zzz"
     ts_wind = *localtime(&who->outsideWindTime);

     //strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &ts);
     strftime(cHiWind, sizeof(cHiWind), "%m-%d %H:%M", &ts_wind);

     log2SD("%s | %s %s %s %c WIND : now %5.1f kph   [hi %s %5.1f kph] DIR=%s %s\n",
         lead,
         cDevName,
         cChannel,
         cBatID,
         record,
         who->outsideWindNow,
         cHiWind,
         who->outsideWindHi,
         windDir,
         "ok" //cDump
         ); //kph

    }

    if (oregon.sens_type == UVN800 && oregon.crc_c){
      Serial.print(" | UVN800  ");
      Serial.print("      ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print("ID: ");
      Serial.print(oregon.sens_id, HEX);

      Serial.print(" UV IDX: ");
      Serial.print(oregon.UV_index);

    }


    if (oregon.sens_type == RFCLOCK && oregon.crc_c){
      Serial.print(" | RF CLOCK");
      Serial.print(" C=");
      Serial.print(oregon.sens_chnl);
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F "); else Serial.print("e ");
      Serial.print("ID: ");
      Serial.print(oregon.sens_id, HEX);
      Serial.print(" TIME: ");
      Serial.print(oregon.packet[6] & 0x0F, HEX);
      Serial.print(oregon.packet[6] & 0xF0 >> 4, HEX);
      Serial.print(':');
      Serial.print(oregon.packet[5] & 0x0F, HEX);
      Serial.print(oregon.packet[5] & 0xF0 >> 4, HEX);
      Serial.print(':');
      Serial.print(':');
      Serial.print(oregon.packet[4] & 0x0F, HEX);
      Serial.print(oregon.packet[4] & 0xF0 >> 4, HEX);
      Serial.print(" DATE: ");
      Serial.print(oregon.packet[7] & 0x0F, HEX);
      Serial.print(oregon.packet[7] & 0xF0 >> 4, HEX);
      Serial.print('.');
      if (oregon.packet[8] & 0x0F ==1 || oregon.packet[8] & 0x0F ==3)   Serial.print('1');
      else Serial.print('0');
      Serial.print(oregon.packet[8] & 0xF0 >> 4, HEX);
      Serial.print('.');
      Serial.print(oregon.packet[9] & 0x0F, HEX);
      Serial.print(oregon.packet[9] & 0xF0 >> 4, HEX);

    }

    if (oregon.sens_type == PCR800 && oregon.crc_c){
      Serial.print(" | PCR800  ");
      Serial.print("      ");
      Serial.print(" BAT: ");
      if (oregon.sens_battery) Serial.print("F"); else Serial.print("e");
      Serial.print(" ID: ");
      Serial.print(oregon.sens_id, HEX);
      Serial.print(" TOTAL: ");
      Serial.print(oregon.get_total_rain(), 1);
      Serial.print(" mm RATE: ");
      Serial.print(oregon.get_rain_rate(), 1);
      Serial.print(" mm/h");

    }

#if ADD_SENS_SUPPORT == 1
      if ((oregon.sens_type & 0xFF00) == THP && oregon.crc_c) {
      Serial.print(" | THP     ");
      Serial.print(" C=");
      Serial.print(oregon.sens_chnl);
      Serial.print(" BAT: ");
      Serial.print(oregon.sens_voltage, 2);
      Serial.print("V");
      if (oregon.sens_tmp > 0 && oregon.sens_tmp < 10) Serial.print(" TMP:  ");
      if (oregon.sens_tmp < 0 && oregon.sens_tmp > -10) Serial.print(" TMP: ");
      if (oregon.sens_tmp <= -10) Serial.print(" TMP:");
      if (oregon.sens_tmp >= 10) Serial.print(" TMP: ");
      Serial.print(oregon.sens_tmp, 1);
      Serial.print("C ");
      Serial.print("HUM: ");
      Serial.print(oregon.sens_hmdty, 1);
      Serial.print("% ");
      Serial.print("PRESS: ");
      Serial.print(oregon.sens_pressure, 1);
      Serial.print("Hgmm");

    }
#endif

  }
  yield();
}


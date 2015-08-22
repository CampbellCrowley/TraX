/*************************************************************************
* Arduino Data Logger Class
* Distributed under GPL v2.0
* Copyright (c) 2013-2014 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
* Visit http://freematics.com for more information
*************************************************************************/

#define FORMAT_BIN 0
#define FORMAT_CSV 1

typedef struct {
    uint32_t time;
    uint16_t pid;
    uint8_t flags;
    uint8_t checksum;
    float value[3];
} LOG_DATA_COMM;

#define HEADER_LEN 128 /* bytes */

#define PID_GPS_LATITUDE 0xA
#define PID_GPS_LONGITUDE 0xB
#define PID_GPS_ALTITUDE 0xC
#define PID_GPS_SPEED 0xD
#define PID_GPS_HEADING 0xE
#define PID_GPS_SAT_COUNT 0xF
#define PID_GPS_TIME 0x10
#define PID_GPS_DATE 0x11

#define PID_ACC 0x20
#define PID_GYRO 0x21
#define PID_COMPASS 0x22
#define PID_MEMS_TEMP 0x23
#define PID_BATTERY_VOLTAGE 0x24

#define FILE_NAME_FORMAT "/DAT%05d.CSV"

#if ENABLE_DATA_OUT || ENABLE_DATA_FILE

  #if USE_SOFTSERIAL

    #if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
        SoftwareSerial SerialBLE(A8, A9); /* for BLE Shield on MEGA*/
    #else
        SoftwareSerial SerialBLE(A2, A3); /* for BLE Shield on UNO/leonardo*/
    #endif

  #else

  #define SerialBLE Serial3

  #endif
  static File sdfile;
#endif

<<<<<<< HEAD
static bool fileOpen = false;
static bool SDChecked = false;

static const char* idstr = "FREEMATICS\r";
=======
// static const char* idstr = "FREEMATICS\r";
static const char* idstr = "__TRAX__ML\r";
>>>>>>> Campbell

class CDataLogger {
public:
    void initSender()
    {
#if ENABLE_DATA_OUT
        SerialBLE.begin(STREAM_BAUDRATE);
        SerialBLE.println(idstr);
#endif
#if ENABLE_DATA_LOG
        m_lastDataTime = 0;
#endif
    }
    void logTimeElapsed()
    {
#if ENABLE_DATA_LOG
        dataSize += sdfile.print(dataTime - m_lastDataTime);
        sdfile.write(',');
        dataSize++;
        m_lastDataTime = dataTime;
#endif
    }
    void logData(char c)
    {
#if ENABLE_DATA_OUT && STREAM_FORMAT == FORMAT_CSV
        SerialBLE.write(c);
#endif
#if ENABLE_DATA_LOG
        if (c >= ' ') {
            sdfile.write(c);
            dataSize++;
        }
#endif
    }
    void logData(uint16_t pid, int value)
    {
        char buf[16];
        byte n = sprintf(buf, "%X,%d\r", pid, value);
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
        SerialBLE.write((uint8_t*)&ld, 12);
#else
        SerialBLE.write((uint8_t*)buf, n);
#endif
#endif
#if ENABLE_DATA_LOG
        logTimeElapsed();
        dataSize += sdfile.write((uint8_t*)buf, n);
#endif
    }
    void logData(uint16_t pid, int32_t value)
    {
        char buf[20];
        byte n = sprintf(buf, "%X,%ld\r", pid, value);
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 1, 0, value};
        ld.checksum = getChecksum((char*)&ld, 12);
        SerialBLE.write((uint8_t*)&ld, 12);
#else
        SerialBLE.write((uint8_t*)buf, n);
#endif
#endif
#if ENABLE_DATA_LOG
        logTimeElapsed();
        dataSize += sdfile.write((uint8_t*)buf, n);
#endif
    }
    void logData(uint16_t pid, int value1, int value2, int value3)
    {
        char buf[24];
        byte n = sprintf(buf, "%X,%d,%d,%d\r", pid, value1, value2, value3);
#if ENABLE_DATA_OUT
#if STREAM_FORMAT == FORMAT_BIN
        LOG_DATA_COMM ld = {dataTime, pid, 3, 0, {value1, value2, value3}};
        ld.checksum = getChecksum((char*)&ld, 20);
        SerialBLE.write((uint8_t*)&ld, 20);
#else
        SerialBLE.write((uint8_t*)buf, n);
#endif
#endif
#if ENABLE_DATA_LOG
        logTimeElapsed();
        dataSize += sdfile.write((uint8_t*)buf, n);
#endif
    }
    void logAll(int lap, unsigned long time, uint32_t dist, byte numSat, float lat, float lon, long speed, double alt, unsigned long bearing, unsigned long accLon, unsigned long accLat, uint32_t rpm, byte throttle)
    {
      String data = "";
      if(!(lap >= 0)) {// lap number NYI
        data += "0,";
      } else {
        data += lap + ",";
      }
      data += ((double)time/1000.0);data+=","; // timestamp (s)
      data += (double)dist*0.621371;data+=","; //  distance driven based on speed and time NOT GPS (miles)
      data += dist;data+=","; //  distance driven based on speed and time NOT GPS (kilometeres)
      data += numSat;data+=",";// number of connected satellites
      
      
      data += (int)(lat/100000);// latitude
      data += '.';
      uint32_t poslat;
      if(lat<0) {
        poslat=lat*-1;
      } else {
        poslat=lat;
      }
      for(uint32_t i = 10000; i > 10; i/=10) {
        if(poslat%(i*10) < i) {
          data += '0';
        } else {
          break;
        }
      }
      if(poslat%100000 < 10000) {
        data += poslat%100000;
      } else {
        data += poslat%100000;
      }
      data+=',';
      
      data += (int)(lon/100000);// longitude
      data += '.';
      uint32_t poslon;
      if(lon<0) {
        poslon=lon*-1;
      } else {
        poslon=lon;
      }
      for(uint32_t i = 10000; i > 10; i/=10) {
        if(poslon%(i*10) < i) {
          data += '0';
        } else {
          break;
        }
      }
      if(poslon%100000 < 10000) {
        data += poslon%100000;
      } else {
        data += poslon%100000;
      }
      data+=',';
      
      data += (double)speed*0.27777777777;data+=",";// speed in meters/second
      data += speed;data+=",";// speed in kilometres/hour
      data += (double)speed*0.621371;data+=",";// speed in miles/hour
      if(alt != -9999) {
        data += alt;data+=",";// altitude
      } else {
        data += ",";
      }
      if(bearing >= 360) {
        data += '0';data+=",";// compass direction/rotation
      } else if(bearing >= 0) {
        data += bearing;data+=",";// compass direction/rotation
      } else {
        data += ",";
      }
      data += accLon;data+=",";// longitudinal acceleration
      data += accLat;data+=",";// lateral acceleration
      data += "0";data+=",";//xpos
      data += "0";data+=",";//ypos
      data += rpm;data+=",";// rpm not simplified
      data += throttle;data+=",";// throttle percent
      data += "";data+=",";//trap name
      
      data += ",\n";
      
      //Lap #,Timestamp (s),Distance (m),Distance (km),Locked satellites,Latitude (deg),Longitude (deg),Speed (m/s),Speed (kph),Speed (mph),Altitude (m),Bearing (deg),Longitudinal Acceleration (G),Lateral Acceleration (G),X-position (m),Y-position (m),RPM (rpm),Throttle Position (%),Trap name
      
      char dataChar[data.length()];
      data.toCharArray(dataChar, data.length());
      
      char buf[data.length()];
      byte n = sprintf(buf, dataChar);
      dataSize += sdfile.write((uint8_t*)buf, n);
      dataSize += sdfile.println();
      /* #if USE_SERIAL_LOGGING
        Serial.print("\nWritten to SD Card: \n");
        Serial.write((uint8_t*)buf, n);
        Serial.print("\n");
        Serial.print(data);
      #endif */
    }
#if ENABLE_DATA_LOG || ENABLE_DATA_FILE
    uint16_t openFile() {
      return openFile(2000, 00, 00, 00, 00, 00);
    }
    uint16_t openFile(uint32_t year, uint32_t month, uint32_t day, uint32_t hour, uint32_t minute, uint32_t second, uint16_t logFlags = 0, uint32_t dateTime = 0)
    {
        uint16_t fileIndex;
        //char filename[24] = "/FRMATICS";
        char filename[24] = "/DATA____";
        
        dataSize = 0;
        if (SD.exists(filename)) {
            for (fileIndex = 1; fileIndex; fileIndex++) {
                sprintf(filename + 9, FILE_NAME_FORMAT, fileIndex);
                if (!SD.exists(filename)) {
                    break;
                }
            }
            if (fileIndex == 0)
                return 0;
        } else {
            SD.mkdir(filename);
            fileIndex = 1;
            sprintf(filename + 9, FILE_NAME_FORMAT, 1);
        }

        sdfile = SD.open(filename, FILE_WRITE);
        if (!sdfile) {
            return 0;
        }
        
        #if USE_SERIAL_LOGGING
          Serial.print("\nDATE: ");
          Serial.print(month);
          Serial.print("/");
          Serial.print(day);
          Serial.print("/");
          Serial.print(year);
          Serial.print(" ");
          Serial.print(hour);
          Serial.print(":");
          Serial.print(minute);
          Serial.print(":");
          Serial.print(second);
        #endif
        
        
        sdfile.println("This file is created using TraX.,,,,,,,,,,,,,,,,,,");
        sdfile.println(",,,,,,,,,,,,,,,,,,");
        sdfile.println("Session title,TraX Home Testing,,,,,,,,,,,,,,,,,");
        sdfile.println("Session type,Lap timing,,,,,,,,,,,,,,,,,");
        sdfile.println("Track name,N/A,,,,,,,,,,,,,,,,,");
        sdfile.println("Driver name,,,,,,,,,,,,,,,,,,");
        sdfile.println("Export scope,Whole session,,,,,,,,,,,,,,,,,");
        
        sdfile.print  ("Created,");
        sdfile.print  (month);
        sdfile.print  ("/");
        sdfile.print  (day);
        sdfile.print  ("/");
        sdfile.print  (year);
        sdfile.print  (",");
        sdfile.print  (hour);
        sdfile.print  (":");
        sdfile.print  (minute);
        sdfile.println(",,,,,,,,,,,,,,,,");
        
        sdfile.println("Note,PROTOTYPE,,,,,,,,,,,,,,,,,");
        sdfile.println(",,,,,,,,,,,,,,,,,,");
        // sdfile.print("\nLap #,Timestamp (ms),Distance (km),Locked satellites,Latitude (deg),Longitude (deg),Speed (m/s),Speed (kph),Altitude (m),Bearing (deg),Longitudinal Acceleration (G),Lateral Acceleration (G),RPM (rpm),Throttle Percentage\r");
        sdfile.println("Lap #,Timestamp (s),Distance (mi),Distance (km),Locked satellites,Latitude (deg),Longitude (deg),Speed (m/s),Speed (kph),Speed (mph),Altitude (m),Bearing (deg),Longitudinal Acceleration (G),Lateral Acceleration (G),X-position (m),Y-position (m),RPM (rpm),Throttle Position (%),Trap name");
        return fileIndex;
    }
    void closeFile()
    {
        sdfile.close();
    }
    void flushFile()
    {
        sdfile.flush();
    }
#endif
    uint32_t dataTime;
    uint32_t dataSize;
private:
    byte getChecksum(char* buffer, byte len)
    {
        uint8_t checksum = 0;
        for (byte i = 0; i < len; i++) {
          checksum ^= buffer[i];
        }
        return checksum;
    }
#if ENABLE_DATA_LOG || ENABLE_DATA_FILE
    uint32_t m_lastDataTime;
#endif
};

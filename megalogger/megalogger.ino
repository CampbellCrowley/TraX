/*************************************************************************
* Arduino GPS/OBD-II/9-Axis Data Logger
* Distributed under GPL v2.0
* Copyright (c) 2013-2015 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
* Visit http://freematics.com for more information
*************************************************************************/
//
#include <Arduino.h>
#include <Wire.h>
#include <OBD.h>
#include <SPI.h>
#include <SD.h>
#include <MultiLCD.h>
#include <TinyGPS.h>
#include <I2Cdev.h>
#include <MPU9150.h>
#include <avr/wdt.h>// for Reboot()
#include "Narcoleptic.h"
#include "config.h"
#if USE_TOUCH
#include "Touch.h"
#endif
#include "images.h"
#if ENABLE_DATA_OUT && USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#include "datalogger.h"


// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_CONNECTED 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_GUI_ON 0x20

// adapter type
#define OBD_ADAPTER_I2C 0
#define OBD_ADAPTER_UART 1

#if USE_GPS
// GPS logging can only be enabled when there is additional hardware serial UART
#define GPSUART Serial2
TinyGPS gps;
#endif

#if USE_MPU6050 || USE_MPU9150
MPU6050 accelgyro;
static uint32_t lastMemsDataTime = 0;
#endif

static uint8_t lastFileSize = 0;
static uint32_t lastGPSDataTime = 0;
static uint32_t lastRefreshTime = 0;
static uint32_t lastTime = 0;
static uint32_t distance = 0;
static uint32_t startTime = 0;
static uint32_t logStartTime = 0;
static uint16_t lastSpeed = 0;
static uint32_t lastSpeedTime = 0;
static int gpsSpeed = -1;
static uint16_t gpsDate = 0;
String inputString = "";
boolean stringComplete = false;
static unsigned int tX, tY;
static bool pressed = false;
static bool lastpressed = false;
static unsigned int numreconnect = 0;
static uint16_t accXoffset = 0;
static uint16_t accYoffset = 0;
static uint16_t accZoffset = 0;

static uint16_t BackgroundColor = RGB16_RED;

/* #define RGB16_RED 0xF800
#define RGB16_GREEN 0x7E0
#define RGB16_BLUE 0x1F
#define RGB16_YELLOW 0xFFE0
#define RGB16_CYAN 0x7FF
#define RGB16_YELLOW 0xF81F
#define RGB16_WHITE 0xFFFF */

static const byte PROGMEM pidTier1[] = {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static const byte PROGMEM pidTier2[] = {PID_INTAKE_MAP, PID_MAF_FLOW, PID_TIMING_ADVANCE};
static const byte PROGMEM pidTier3[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_ENGINE_FUEL_RATE};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)
#define TIER_NUM3 sizeof(pidTier3)

byte state = 0;

void processAccelerometer();
void processGPS();

CDataLogger logger;

class CMyOBD : public COBD
{
  public:
    void dataIdleLoop()
    {
      if (!(state & STATE_GUI_ON)) return;

      if (state & STATE_MEMS_READY) {
        processAccelerometer();
      }
#if USE_GPS
      uint32_t t = millis();
      while (GPSUART.available() && millis() - t < MAX_GPS_PROCESS_TIME) {
        processGPS();
      }
#endif
    }
};

class CMyOBDI2C : public COBDI2C
{
  public:
    void dataIdleLoop()
    {
      if (!(state & STATE_GUI_ON)) return;

      if (state & STATE_MEMS_READY) {
        processAccelerometer();
      }
#if USE_GPS
      uint32_t t = millis();
      while (GPSUART.available() && millis() - t < MAX_GPS_PROCESS_TIME) {
        processGPS();
      }
#endif
    }
};

#if OBD_ADAPTER_MODEL == OBD_MODEL_I2C
CMyOBDI2C obd;
#else
CMyOBD obd;
#endif

void setColorByValue(int value, int threshold1, int threshold2, int threshold3)
{
  if (value < 0) value = -value;
  if (value < threshold1) {
    lcd.setColor(RGB16_WHITE);
  } else if (value < threshold2) {
    byte n = (uint32_t)(threshold2 - value) * 255 / (threshold2 - threshold1);
    lcd.setColor(255, 255, n);
  } else if (value < threshold3) {
    byte n = (uint32_t)(threshold3 - value) * 255 / (threshold3 - threshold2);
    lcd.setColor(255, n, 0);
  } else {
    lcd.setColor(255, 0, 0);
  }
}

void showPIDData(byte pid, int value)
{
  char buf[8];
  switch (pid) {
    case PID_RPM:
      lcd.setFontSize(FONT_SIZE_XLARGE);
      lcd.setCursor(32, 6);
      if (value >= 10000) break;
      setColorByValue(value, 2500, 5000, 7000);
      lcd.printInt(value, 4);
      break;
    case PID_SPEED:
      if (value < 1000) {
        lcd.setFontSize(FONT_SIZE_XLARGE);
        lcd.setCursor(50, 2);
        setColorByValue(value, 60, 100, 160);
        lcd.printInt(value, 3);

        if (gpsSpeed != -1) {
          lcd.setFontSize(FONT_SIZE_SMALL);
          lcd.setCursor(110, 2);
          lcd.setColor(RGB16_YELLOW);
          int diff = gpsSpeed - value;
          if (diff >= 0) {
            lcd.write('+');
            lcd.printInt(diff);
          } else {
            lcd.write('-');
            lcd.printInt(-diff);
          }
          lcd.write(' ');
        }
      }
      break;
    case PID_ENGINE_LOAD:
      lcd.setFontSize(FONT_SIZE_XLARGE);
      lcd.setCursor(50, 10);
      if (value >= 100) value = 99;
      setColorByValue(value, 75, 80, 100);
      lcd.printInt(value, 3);
      break;
    case PID_THROTTLE:
      lcd.setFontSize(FONT_SIZE_LARGE);
      lcd.setCursor(80, 21);
      if (value >= 100) value = 99;
      setColorByValue(value, 50, 75, 100);
      lcd.printInt(value, 2);
      break;
    case PID_ENGINE_FUEL_RATE:
      if (value < 100) {
        lcd.setFontSize(FONT_SIZE_LARGE);
        lcd.setCursor(80, 24);
        lcd.printInt(value, 2);
      }
      break;
    case PID_INTAKE_TEMP:
      if (value >= 0 && value < 100) {
        lcd.setFontSize(FONT_SIZE_LARGE);
        lcd.setCursor(80, 27);
        lcd.printInt(value, 2);
      }
      break;
  }
  lcd.setColor(RGB16_WHITE);
}

void ShowVoltage(float v)
{
  lcd.setFontSize(FONT_SIZE_LARGE);
  lcd.setCursor(80, 18);
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.print(v);
}

void fadeOutScreen()
{
  // fade out backlight
  for (int n = 254; n >= 0; n--) {
    lcd.setBackLight(n);
    delay(5);// 5
  }
}

void fadeInScreen()
{
  for (int n = 1; n <= 255; n++) {
    lcd.setBackLight(n);
    delay(10);// 10
  }
}

void initScreen()
{
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setColor(RGB16_CYAN);
  lcd.setCursor(0, 2);
  lcd.print("ELAPSED:");

  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.setCursor(0, 28);
  lcd.print("LOG SIZE:");

  //lcd.setColor(0xFFFF);
  /*
  lcd.setCursor(32, 4);
  lcd.print("%");
  lcd.setCursor(68, 5);
  lcd.print("Intake Air");
  lcd.setCursor(112, 4);
  lcd.print("C");
  */

  state |= STATE_GUI_ON;

  fadeInScreen();
}

#if ENABLE_DATA_LOG || ENABLE_DATA_FILE
bool checkSD()
{
  Sd2Card card;
  SdVolume volume;
  state &= ~STATE_SD_READY;
  pinMode(SS, OUTPUT);

  lcd.setFontSize(FONT_SIZE_MEDIUM);
  if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
    const char* type;
    switch (card.type()) {
      case SD_CARD_TYPE_SD1:
        type = "SD1";
        break;
      case SD_CARD_TYPE_SD2:
        type = "SD2";
        break;
      case SD_CARD_TYPE_SDHC:
        type = "SDHC";
        break;
      default:
        type = "SDx";
    }

    lcd.print(type);
    lcd.write(' ');
    if (!volume.init(card)) {
      lcd.print("No FAT!");
      return false;
    }

    uint32_t volumesize = volume.blocksPerCluster();
    volumesize >>= 1; // 512 bytes per block
    volumesize *= volume.clusterCount();
    volumesize >>= 10;

    lcd.print((int)volumesize);
    lcd.print("MB");
  } else {
    lcd.println("No SD Card");
    return false;
  }

  if (!SD.begin(SD_CS_PIN)) {
    lcd.println("Bad SD");
    return false;
  }

  state |= STATE_SD_READY;
  return true;
}
#endif

#if USE_GPS
void processGPS()
{
  getTouch();
  if(!lastpressed && pressed) {
    if(fileOpen) {
      stopLogging();
      if(fileOpen) {
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(0,10);
        lcd.print("SD ERROR");
      }
    } else {
      uint16_t index = startLogging();
      if(!fileOpen) {
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(0,10);
        lcd.print("SD ERROR");
      } else {
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(0,10);
        lcd.print("SD: ");
        lcd.print(index);
      }
    }
  }
  // process GPS data
  char c = GPSUART.read();
  if (!gps.encode(c))
    return;
  
  // parsed GPS data is ready
  uint32_t time;
  uint32_t date;

  logger.dataTime = millis();

  gps.get_datetime(&date, &time, 0);
  if (date != gpsDate) {
    // log date only if it's changed
    logger.logData(PID_GPS_DATE, (int32_t)time);
    gpsDate = date;
  }
  logger.logData(PID_GPS_TIME, (int32_t)time);

  int32_t lat, lon;
  gps.get_position(&lat, &lon, 0);

  byte sat = gps.satellites();


  // keep current data time as last GPS time
  lastGPSDataTime = logger.dataTime;

<<<<<<< HEAD
=======
  // display UTC date/time
  lcd.write(' ');
  lcd.setCursor(214, 18);
  lcd.setFlags(FLAG_PAD_ZERO);
  lcd.printLong(date, 6);
  lcd.write(' ');
  lcd.printLong(time, 8);

  /* // display latitude
  lcd.setCursor(214, 19);
  lcd.print(lat / 100000);
  lcd.write('.');
  lcd.printLong(abs(lat) % 100000, 5);
  // display longitude
  lcd.setCursor(214, 20);
  lcd.print(lon / 100000);
  lcd.write('.');
  lcd.printLong(abs(lon) % 100000, 5); */
  
  //display latitude
  lcd.setCursor(214, 19);
  lcd.print((int)(lon/100000));
  lcd.print('.');
  uint32_t poslon;
  if(lon<0) {
    poslon=lon*-1;
  } else {
    poslon=lon;
  }
  for(uint32_t i = 10000; i > 10; i/=10) {
    if(poslon%(i*10) < i) {
      lcd.print('0');
    } else {
      break;
    }
  }
  if(poslon%100000 < 10000) {
    lcd.print(poslon%100000);
  } else {
    lcd.print(poslon%100000);
  }
  //display longitude
  lcd.setCursor(214, 20);
  lcd.print((int)(lat/100000));
  lcd.print('.');
  uint32_t poslat;
  if(lat<0) {
    poslat=lat*-1;
  } else {
    poslat=lat;
  }
  for(uint32_t i = 10000; i > 10; i/=10) {
    if(poslat%(i*10) < i) {
      lcd.print('0');
    } else {
      break;
    }
  }
  if(poslat%100000 < 10000) {
    lcd.print(poslat%100000);
  } else {
    lcd.print(poslat%100000);
  }
  
>>>>>>> Campbell
  // log latitude/longitude
  logger.logData(PID_GPS_LATITUDE, lat);
  logger.logData(PID_GPS_LONGITUDE, lon);

  // display number of satellites
  if (sat < 100) {
    /* lcd.setCursor(280, 20);
    lcd.printInt(sat);
    lcd.write(' '); */
  }
  // display altitude
  int32_t alt = gps.altitude();
  if (alt > -1000000 && alt < 1000000) {
    // log altitude
    logger.logData(PID_GPS_ALTITUDE, (int)(alt / 100));
  }

  // only log these data when satellite status is good
  if (sat >= 3) {
    gpsSpeed = gps.speed() * 1852 / 100000;
    logger.logData(PID_GPS_SPEED, gpsSpeed);
  }
}
#endif

void processAccelerometer()
{
  #if USE_MPU6050 || USE_MPU9150
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    #if USE_MPU9150
      int16_t mx, my, mz;
    #endif
    int temp;

    if (logger.dataTime - lastMemsDataTime < ACC_DATA_INTERVAL) {
      return;
    }

    #if USE_MPU9150
      accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    #else
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    #endif

    logger.dataTime = millis();

<<<<<<< HEAD
  temp = accelgyro.getTemperature();

  ax /= ACC_DATA_RATIO;
  ay /= ACC_DATA_RATIO;
  az /= ACC_DATA_RATIO;
  gx /= GYRO_DATA_RATIO;
  gy /= GYRO_DATA_RATIO;
  gz /= GYRO_DATA_RATIO;
#if USE_MPU9150
  mx /= COMPASS_DATA_RATIO;
  my /= COMPASS_DATA_RATIO;
  mz /= COMPASS_DATA_RATIO;
#endif
=======
    temp = accelgyro.getTemperature();
>>>>>>> Campbell

    ax /= ACC_DATA_RATIO;
    ay /= ACC_DATA_RATIO;
    az /= ACC_DATA_RATIO;
    gx /= GYRO_DATA_RATIO;
    gy /= GYRO_DATA_RATIO;
    gz /= GYRO_DATA_RATIO;
    #if USE_MPU9150
      mx /= COMPASS_DATA_RATIO;
      my /= COMPASS_DATA_RATIO;
      mz /= COMPASS_DATA_RATIO;
    #endif
    // display MEMS data
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(214, 22);
    setColorByValue(ax, 50, 100, 200);
    lcd.print(ax);
    setColorByValue(ay, 50, 100, 200);
    lcd.write('/');
    lcd.print(ay);
    setColorByValue(az, 50, 100, 200);
    lcd.write('/');
    lcd.print(az);
    lcd.printSpace(8);

    // display gyro data
    lcd.setCursor(214, 23);
    lcd.setColor(RGB16_WHITE);
    lcd.print(gx);
    lcd.write('/');
    lcd.print(gy);
    lcd.write('/');
    lcd.print(gz);
    lcd.printSpace(8);

    #if USE_MPU9150
      // display compass data
      lcd.setCursor(214, 24);
      lcd.setColor(RGB16_WHITE);
      lcd.print(mx);
      lcd.write('/');
      lcd.print(my);
      lcd.write('/');
      lcd.print(mz);
      lcd.printSpace(8);
    #endif

    // log x/y/z of accelerometer
    logger.logData(PID_ACC, ax, ay, az);
    // log x/y/z of gyro meter
    logger.logData(PID_GYRO, gx, gy, gz);
    #if USE_MPU9150
      // log x/y/z of compass
      logger.logData(PID_COMPASS, mx, my, mz);
    #endif
    logger.logData(PID_MEMS_TEMP, temp);

    lastMemsDataTime = logger.dataTime;
  #endif
}

void logOBDData(byte pid)
{
  getTouch();
  if(!lastpressed && pressed) {
    if(fileOpen) {
      stopLogging();
      if(fileOpen) {
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(0,10);
        lcd.print("SD ERROR");
      }
    } else {
      uint16_t index = startLogging();
      if(!fileOpen) {
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(0,10);
        lcd.print("SD ERROR");
      } else {
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(0,10);
        lcd.print("SD: ");
        lcd.print(index);
      }
    }
  }
  char buffer[OBD_RECV_BUF_SIZE];
  uint32_t start = millis();
  int value;

  // send query for OBD-II PID
  obd.sendQuery(pid);
  // let PID parsed from response
  pid = 0;
  
  
  /* // read responded PID and data
  if (!obd.getResult(pid, value)) {
    return;
  } */
  if(obd.getResult(pid, value)) {

    logger.dataTime = millis();
    // display data
    showPIDData(pid, value);

    // log data to SD card
    logger.logData(0x100 | pid, value);

    if (pid == PID_SPEED) {
      // estimate distance travelled since last speed update
      distance += (uint32_t)(value + lastSpeed) * (logger.dataTime - lastSpeedTime) / 6000;
      // display speed
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      lcd.setCursor(250, 5);
      lcd.printInt(distance / 1000);
      lcd.write('.');
      lcd.printInt(((uint16_t)distance % 1000) / 100);
      // calculate and display average speed
      int avgSpeed = (unsigned long)distance * 3600 / (millis() - startTime);
      lcd.setCursor(250, 8);
      lcd.printInt(avgSpeed);

      lastSpeed = value;
      lastSpeedTime = logger.dataTime;
    
    }
  }
<<<<<<< HEAD

  logger.dataTime = millis();
  // display data
  //showPIDData(pid, value);

  // log data to SD card
  logger.logData(0x100 | pid, value);

  if (pid == PID_SPEED) {
    // estimate distance travelled since last speed update
    distance += (uint32_t)(value + lastSpeed) * (logger.dataTime - lastSpeedTime) / 6000;

    lastSpeed = value;
    lastSpeedTime = logger.dataTime;
  }
#if ENABLE_DATA_LOG
  // flush SD data every 1KB
  if ((logger.dataSize >> 10) != lastFileSize) {
    logger.flushFile();
    // display logged data sizes
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.setCursor(0, 30);
    lcd.setColor(RGB16_WHITE);
    lcd.print((unsigned int)(logger.dataSize >> 10));
    lcd.print("KB");
    lastFileSize = logger.dataSize >> 10;
  }
#endif

=======
>>>>>>> Campbell
  // if OBD response is very fast, go on processing other data for a while
  #ifdef OBD_MIN_INTERVAL
    /* do {
      obd.dataIdleLoop();
    } while (millis() - start < OBD_MIN_INTERVAL); */
    uint32_t last = millis();
    writeSD();
    while (millis() - start < OBD_MIN_INTERVAL) {
      obd.dataIdleLoop();
      if((millis() - last) > 5 == 0) { 
        writeSD();
        last = millis();
      }
    }
  #endif
}

void showECUCap()
{
  static const byte PROGMEM pidlist[] = {PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_FUEL_PRESSURE, PID_INTAKE_MAP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP, PID_MAF_FLOW, PID_THROTTLE, PID_AUX_INPUT,
                                         PID_EGR_ERROR, PID_COMMANDED_EVAPORATIVE_PURGE, PID_FUEL_LEVEL, PID_CONTROL_MODULE_VOLTAGE, PID_ABSOLUTE_ENGINE_LOAD, PID_AMBIENT_TEMP, PID_COMMANDED_THROTTLE_ACTUATOR, PID_ETHANOL_FUEL,
                                         PID_FUEL_RAIL_PRESSURE, PID_HYBRID_BATTERY_PERCENTAGE, PID_ENGINE_OIL_TEMP, PID_FUEL_INJECTION_TIMING, PID_ENGINE_FUEL_RATE, PID_ENGINE_TORQUE_DEMANDED, PID_ENGINE_TORQUE_PERCENTAGE
                                        };

  lcd.setColor(RGB16_WHITE);
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i += 2) {
    for (byte j = 0; j < 2; j++) {
      byte pid = pgm_read_byte(pidlist + i + j);
      lcd.setCursor(216 + j * 56 , i + 4);
      lcd.print((int)pid | 0x100, HEX);
      bool valid = obd.isValidPID(pid);
      if (valid) {
        lcd.setColor(RGB16_GREEN);
        lcd.draw(tick, 16, 16);
        lcd.setColor(RGB16_WHITE);
      }
    }
  }
}

bool reconnect()
{
  int value;
  bool reconnected = false;
  numreconnect++;
#if OBD_BREAKOUT == 0
  fadeOutScreen();
  lcd.clear();
#endif
/* #if ENABLE_DATA_LOG
  logger.closeFile();
#endif */
  if(numreconnect >= 3 && !(obd.read(PID_RPM, value) && value <= 0)) {
    lcd.setFontSize(FONT_SIZE_XLARGE);
    lcd.setColor(RGB16_YELLOW);
    lcd.setCursor(0, 0);
    lcd.print("OBD RESTARTING  ");
    obd.end();
    obd.begin();
    numreconnect = 0;
  } else {
    lcd.setFontSize(FONT_SIZE_XLARGE);
    lcd.setColor(RGB16_YELLOW);
    lcd.setCursor(0, 0);
    lcd.print("OBD RECONNECTING");
  }
  obd.init(OBD_PROTOCOL);
  
//#if OBD_BREAKOUT == 0
  state &= ~(STATE_OBD_READY/* | STATE_GUI_ON*/);
//#endif
  //digitalWrite(SD_CS_PIN, LOW);
  
  if (obd.read(PID_RPM, value) && value > 0) {
      // re-initialize
      state |= STATE_OBD_READY;
      // startTime = millis();
      lastSpeedTime = millis();
      lastSpeed = 0;
      distance = 0;
      reconnected = true;
    }

  //Narcoleptic.delay(1000);
/* #if ENABLE_DATA_LOG
  logger.openFile();
#endif */
#if OBD_BREAKOUT == 0
  initScreen();
#endif
  lcd.setFontSize(FONT_SIZE_XLARGE);
  lcd.setColor(RGB16_YELLOW);
  lcd.setCursor(0, 0);
  lcd.print("OBD             ");
  lcd.setColor(RGB16_WHITE);
  return reconnected;
}

int startLogging()
{
  #if ENABLE_DATA_LOG
    lcd.setColor(RGB16_WHITE);
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(0,10);
    if (!SDChecked && checkSD()) {
      SDChecked = true;
      uint16_t index = logger.openFile();
      if (index > 0) {
        fileOpen = true;
        logStartTime = millis();
        #if USE_SERIAL_LOGGING
          Serial.print("SD: ");
          Serial.println(index);
        #endif
        fillBackground(RGB16_GREEN);
        initScreen();
        
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(0,10);
        lcd.print("SD: ");
        lcd.print(index);
        return index;
      } else {
        lcd.print("No File");
        #if USE_SERIAL_LOGGING
          Serial.print("SD: No File");
        #endif
        fileOpen = false;
        return -1;
      }
    } else if(SDChecked) {
      uint16_t index = logger.openFile();
      if (index > 0) {
        fileOpen = true;
        logStartTime = millis();
        #if USE_SERIAL_LOGGING
          Serial.print("SD: ");
          Serial.println(index);
        #endif
        fillBackground(RGB16_GREEN);
        initScreen();
        
        lcd.setColor(RGB16_WHITE);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(0,10);
        lcd.print("SD: ");
        lcd.print(index);
        return index;
      } else {
        lcd.print("No File");
        #if USE_SERIAL_LOGGING
          Serial.print("SD: No File");
        #endif
        fileOpen = false;
        return -1;
      }
    }
  #else
    lcd.print("File Logging Disabled");
    fileOpen = false;
    return -1;
  #endif
}
void stopLogging()
{
  #if ENABLE_DATA_LOG
    logger.closeFile();
    #if USE_SERIAL_LOGGING
      Serial.print("SD: No File");
    #endif
    fileOpen = false;
    fillBackground(RGB16_RED);
    initScreen();
    
    lcd.setColor(RGB16_WHITE);
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(0,10);
    
    lcd.setColor(RGB16_WHITE);
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(0,10);
    lcd.print("No File");
  #endif
}

void getTouch() {
  lastpressed = pressed;
  pressed=false;
  #if USE_TOUCH
    if(TouchDataAvailable() == 1)
    {
      TouchRead();
      tX = TouchGetX();
      tY = TouchGetY();
      #if USE_SERIAL_LOGGING
        Serial.print("\nTouch: (");
        Serial.print(tX);
        Serial.print(",");
        Serial.print(tY);
        Serial.print(")");
      #endif
      pressed=true;
    }
  #endif
}

void fillBackground(uint16_t color) {
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(0,0);
  lcd.setBackColor(color);
  lcd.setColor(color);
  for(int r=0; r<30; r++) {
    for(int c=0; c<53; c++) {
      lcd.print(' ');
    }
    lcd.println();
  }
  lcd.setBackColor(RGB16_BLACK);
}
// screen layout related stuff
void showStates()
{
  #if USE_SERIAL_LOGGING
  Serial.print("\nShowing States...");
  #endif
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setColor(RGB16_WHITE);
  lcd.setCursor(0, 8);
  lcd.print("MEMS ");
  lcd.setColor((state & STATE_MEMS_READY) ? RGB16_GREEN : RGB16_RED);
  lcd.draw((state & STATE_MEMS_READY) ? tick : cross, 16, 16);

  lcd.setColor(RGB16_WHITE);
  lcd.setCursor(60, 8);
  lcd.print(" GPS ");
  unsigned long rates[2] = {38400, 115200};
  byte sizeofRates = sizeof(rates) / sizeof(rates[0]);
  
  /* for(byte i=0; i<sizeofRates; i++) {
    Serial.print("\n");
    Serial.print(rates[i]);
    Serial.print(" (");
    Serial.print((rates[i] == 38400 || rates[i] == 115200));
    Serial.print(")");
  } */
  
  unsigned long t = millis();
  do {
    if (GPSUART.available() && GPSUART.read() == '\r' && gps.satellites()>=0) {
      state |= STATE_GPS_CONNECTED;
      break;
    }
  } while (millis() - t <= 5000);
  
  unsigned long oldRate = 0;
  for (byte i = 0; i <= sizeofRates; i++) {// try to auto detect GPS baud rate based on pre-defined possibilities
    
    lcd.setColor(RGB16_WHITE);
    lcd.setCursor(60, 8);
    lcd.print(" GPS ");
    lcd.setColor(RGB16_YELLOW);
    lcd.print("Loading"); 
    for(byte j = 0; j<i; j++) {
      lcd.print(".");
    }
    if (state & STATE_GPS_CONNECTED) {
      //lcd.setColor(RGB16_BLACK);
      lcd.setColor(RGB16_WHITE);
      lcd.setCursor(60, 8);
      lcd.print(" GPS ");
      for(byte j = 0; j < i + 7; j++) {
        lcd.print(" ");
      }
      lcd.setCursor(100, 8);
      lcd.setColor(RGB16_GREEN);
      lcd.draw(tick, 16, 16);
      lcd.print(" ");
      
      /* #if USE_SERIAL_LOGGING
      lcd.setColor(RGB16_WHITE);
      lcd.setCursor(125, 8);
      lcd.print(oldRate);
      #endif */
      
      if(i>0) {
        #if USE_SERIAL_LOGGING
        Serial.print("\nGPS: New baud rate: ");
        Serial.print(oldRate);
        #endif
      } else {
        #if USE_SERIAL_LOGGING
        Serial.print("\nGPS: Baud rate already correct");
        #endif
      }
      break;// successfully connected to GPS
    } else {
      if(i < sizeofRates) {
        GPSUART.begin(rates[i]);
        #if USE_SERIAL_LOGGING
        Serial.print("\nGPS: Trying other baud rate(");
        Serial.print(i);
        Serial.print("/");
        Serial.print(sizeofRates);
        Serial.print("): ");
        Serial.print(rates[i]);
        /* Serial.print("(");
        Serial.print(rates[i]);
        Serial.print(")"); */
        #endif
        
        t = millis();
        #if USE_SERIAL_LOGGING
        Serial.print("\nGPS: checking available and ready");
        #endif
        do {
          if (GPSUART.available() && GPSUART.read() == '\r' && gps.satellites()>=0) {
            state |= STATE_GPS_CONNECTED;
            break;
          }
        } while (millis() - t <= 5000);
        
      } else {
        lcd.setColor(RGB16_WHITE);
        lcd.setCursor(60, 8);
        lcd.print(" GPS ");
        for(byte j = 0; j < i + 7; j++) {
          lcd.print(" ");
        }
        lcd.setCursor(125, 8);
        lcd.setColor(RGB16_RED);
        lcd.draw(cross, 16, 16);
        break;
      }
    }
    // oldRate = rate;
    oldRate = rates[i];
  }
  lcd.setColor(RGB16_WHITE);
}

void testOut()
{
#if USE_SERIAL_LOGGING
  Serial.print("\n\nTesting Commands\n");
#endif
  static const char PROGMEM cmds[][6] = {"ATZ\r", "ATL1\r", "ATRV\r", "0100\r", "010C\r", "0902\r"};
  char buf[OBD_RECV_BUF_SIZE];
  lcd.setFontSize(FONT_SIZE_SMALL);
  lcd.setCursor(0, 11);

  for (byte i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
    String nextOut = "";
    char cmd[6];
    memcpy_P(cmd, cmds[i], sizeof(cmd));
    lcd.setColor(RGB16_WHITE);
    /* 
    for(byte j = 0; j < 20; j++) {
      lcd.print('.');
    }
    lcd.print(cmd);*/
    #if USE_SERIAL_LOGGING
      Serial.print(cmd);
      Serial.print(": ");
    #endif 
    lcd.println(cmd);
    lcd.setColor(RGB16_CYAN);
    if (obd.sendCommand(cmd, buf)) {
      char *p = strstr(buf, cmd);
      if (p)
        p += strlen(cmd);
      else
        p = buf;
      while (*p == '\r') p++;
      while (*p) {
        lcd.write(*p);
        nextOut += *p;
        if (*p == '\r' && *(p + 1) != '\r') {
          lcd.write('\n');
          nextOut += ": ";
        }
        p++;
      }
#if USE_SERIAL_LOGGING
      Serial.println(nextOut.substring(0, nextOut.length() - 2)); // Print everything except for the last ": " at the end
#endif
    } else {
#if USE_SERIAL_LOGGING
      Serial.println("Timeout");
#endif
      lcd.println("Timeout");
    }
    //delay(1000);
  }
  lcd.println();
}

void setup()
{
  #if USE_SERIAL_LOGGING
    Serial.begin(SERIAL_BAUD);      // open the serial port at 19200 baud
    Serial.print("Beginning Serial Logging");
    inputString.reserve(200);
  #endif
  lcd.begin();
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setColor(0xFFE0);
<<<<<<< HEAD
  lcd.println("MEGA LOGGER - OBD-II/GPS/MEMS");
  lcd.println("Basic GUI");
=======
  lcd.println("TraX - OBD-II/GPS/MEMS");
  lcd.println();
>>>>>>> Campbell
  lcd.setColor(RGB16_WHITE);
  #if USE_SERIAL_LOGGING
    Serial.print("\nLCD Ready");
  #endif
  #if USE_TOUCH
      TouchInit();
    #if USE_SERIAL_LOGGING
      Serial.print("\nTouch Ready");
    #endif
  #endif
  #if USE_MPU6050 || USE_MPU9150
    Wire.begin();
    accelgyro.initialize();
    if (accelgyro.testConnection()) state |= STATE_MEMS_READY;
    #if USE_SERIAL_LOGGING
      Serial.print("\nAccelerometer Ready");
    #endif
  #endif

  #if USE_GPS
    lcd.setColor(RGB16_WHITE);
    lcd.setCursor(60, 8);
    // lcd.print(" GPS ");
    lcd.print("     ");
    lcd.setColor(RGB16_YELLOW);
    lcd.print("Loading");
    lcd.setColor(RGB16_WHITE);
    lcd.setCursor(0, 0);
    lcd.println();
    GPSUART.begin(GPS_BAUDRATE);
    // switching to 10Hz mode, effective only for MTK3329
    //GPSUART.println(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    //GPSUART.println(PMTK_SET_NMEA_UPDATE_10HZ);
    lastGPSDataTime = 0;
    #if USE_SERIAL_LOGGING
      Serial.print("\nGPS begin on suggested baud rate of ");
      Serial.print(GPS_BAUDRATE);
    #endif
  #endif
  logger.initSender();
  #if USE_SERIAL_LOGGING
    Serial.print("\nLogger Ready");
  #endif

<<<<<<< HEAD
=======
  
  bool fileOpened = false;
  unsigned long fix_age;
  int year;
  uint16_t index;
  byte month, day, hour, minute, second, hundredths;
  // start SD Card logging with GPS date and time
  #if ENABLE_DATA_LOG || ENABLE_DATA_FILE
    if(!fileOpened) {
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      lcd.setCursor(0,0);
      lcd.println("\n");
      if (checkSD()) {
        long t = millis();
        do {
          #if USE_GPS
            
            long t = millis();
            // int c = GPSUART.read();
            // if(gps.encode(c)) {
              gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &fix_age);
              
              float flat, flon;
              gps.f_get_position(&flat, &flon, &fix_age);
              if (fix_age == TinyGPS::GPS_INVALID_AGE)
              {
                /* Serial.print("GPS: No fix detected ");
                Serial.print(fix_age);
                Serial.print(", "); */
              }
              else if (fix_age > 5000)
                Serial.print("\nGPS: Warning: possible stale data!");
              else
                Serial.print("\nGPS: Data is current.");
              
            // }
            if(fix_age != TinyGPS::GPS_INVALID_AGE && fix_age < 10000){
              #if USE_SERIAL_LOGGING
                Serial.print("\nGPS: Fix Age, Sat: ");
                Serial.print(fix_age);
                Serial.print(", ");
                Serial.print(gps.satellites());
              #endif
              index = logger.openFile(year, month, day, hour, minute, second);
              fileOpened = true;
            }
          #else
            // index = logger.openFile();
            index = logger.openFile(year, month, day, hour, minute, second);
            fileOpened = true;
          #endif
          
          if(fileOpened) {
            lcd.setCursor(0,0);
            lcd.println();
            if (index > 0) {
              lcd.print("File ID:");
              lcd.println(index);
            } else {
              lcd.print("No File");
            }
          }
        } while(!fileOpened && millis()- t < 5000);
        if(!fileOpened) {
          // uint16_t index = logger.openFile();
          index = logger.openFile(year, month, day, hour, minute, second);
          fileOpened = true;
          lcd.setCursor(0,0);
          lcd.println();
          if (index > 0) {
            lcd.print("File ID:");
            lcd.println(index);
          } else {
            lcd.print("No File");
          }
        }
      }
    }
    // lcd.setCursor(0,0);
    // lcd.println("\n\n");
    // if(checkSD()) {
    // }
    #if USE_SERIAL_LOGGING
    if(fileOpened) {
        Serial.print("\nSD Ready (");
        Serial.print(fix_age);
        Serial.print(")");
    } else {
        Serial.print("\nSD FAILED to ready (");
        Serial.print(fix_age);
        Serial.print(")");
    }
    #endif
  #endif
  
>>>>>>> Campbell

/* #if USE_SERIAL_LOGGING
  Serial.print("\nStates: 1st Run");
#endif */

  showStates();

  unsigned long t = millis();
  #if USE_GPS
    #if USE_SERIAL_LOGGING
      Serial.print("\nGPS: checking available and ready");
    #endif
    do {
      if (GPSUART.available() && GPSUART.read() == '\r') {
        state |= STATE_GPS_CONNECTED;
        /* #if USE_SERIAL_LOGGING
          Serial.print("\nGPS: Ready");
        #endif */
        break;
      }
    } while (millis() - t <= 5000);
    /* #if USE_SERIAL_LOGGING
      Serial.print("\nStates: 2nd Run");
    #endif */
    showStates();
  #endif

  obd.begin();
#if USE_SERIAL_LOGGING
  Serial.print("\nOBD: Begin");
#endif

  // this will send a bunch of commands and display response
  testOut();
#if USE_SERIAL_LOGGING
  Serial.print('\n');
#endif

  // initialize the OBD until success
#if USE_SERIAL_LOGGING
  Serial.print("\nOBD: init and wait until ready... (Breakout: ");
  Serial.print(OBD_BREAKOUT);
  Serial.print(")");
#endif
  lcd.setCursor(0, 25);
  lcd.print("\nConnecting to OBD... ");
  int value;
  t = millis();
  do {
    if(millis() - t > 15000 && OBD_BREAKOUT) {// Breakout of OBD init loop
      // lcd.setCursor(0, 25);
      lcd.setColor(RGB16_RED);
      lcd.setCursor(0, 25);
      lcd.println("\nFailed to connect to OBD!          ");
      lcd.setColor(RGB16_CYAN);
      #if USE_SERIAL_LOGGING
        Serial.print("\nFailed to initialize OBD!\n");
      #endif
      break;
    } else if(OBD_BREAKOUT) {
      lcd.setCursor(180, 26);
      unsigned int timeLeftInt = ((15000 - (millis() - t))/1000/*100*/);
      /*double timeLeftDoub = (double)timeLeftInt/10;*/
      lcd.print(timeLeftInt);
      lcd.print("  ");
    }
  } while (!obd.init(OBD_PROTOCOL)/* && !obd.read(PID_RPM,value) && !(value > 0)*/);
  if (millis() - t <= 15000 || !OBD_BREAKOUT) {// if the init completed successfully before the breakout
    lcd.println("\nConnected to OBD!\n");
    #if USE_SERIAL_LOGGING
      Serial.print("\nOBD: Ready! ");
      //Serial.print(value);
    #endif
  }
  
  state |= STATE_OBD_READY;

  lcd.setColor(RGB16_GREEN);
  lcd.setFontSize(FONT_SIZE_MEDIUM);
  lcd.setCursor(0,27);
  lcd.println("READY!");
  lcd.setColor(RGB16_YELLOW);

  char buf[OBD_RECV_BUF_SIZE];
  if (obd.getVIN(buf)) {
    lcd.print("VIN:");
    lcd.print(buf);
    Serial.print("\nVIN: ");
    Serial.print(buf);
  }

  //lcd.setFont(FONT_SIZE_MEDIUM);
  //lcd.setCursor(0, 14);
  //lcd.print("VIN: XXXXXXXX");

  showECUCap();
  delay(3000);

  fadeOutScreen();
  
  lcd.clear();
  lcd.setBackColor(BackgroundColor);
  fillBackground(RGB16_RED);
  initScreen();

  startTime = millis();
  lastSpeedTime = startTime;
  lastRefreshTime = startTime;
  lastTime = startTime;
  // lastRefreshTime = logger.dataTime;
}

void writeSD()
{
  // Write data to SD Card \\//
  #if ENABLE_DATA_FILE && ENABLE_DATA_LOG == 0
    #if USE_SERIAL_LOGGING
      Serial.print("\nWriting to SD Card");
    #endif
    #if USE_GPS
      int32_t lat, lon;
      gps.get_position(&lat, &lon, 0);
      byte numSat = gps.satellites();
      long alt = gps.altitude()/100.0;
    #else
      numSat=-1;
      alt = -9999;
    #endif
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t mx, my, mz;
    
    #if USE_MPU6050 || USE_MPU9150
      #if USE_MPU9150
        accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
      #else
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        mx=-1;
        my=-1;
        mz=-1;
      #endif
    #else
      ax=-1;
      ay=-1;
      az=-1;
      gx=-1;
      gy=-1;
      gz=-1;
      mx=-1;
      my=-1;
      mz=-1;
    #endif
    ax /= ACC_DATA_RATIO;
    ay /= ACC_DATA_RATIO;
    az /= ACC_DATA_RATIO;
    gx /= GYRO_DATA_RATIO;
    gy /= GYRO_DATA_RATIO;
    gz /= GYRO_DATA_RATIO;
    #if USE_MPU9150
      mx /= COMPASS_DATA_RATIO;
      my /= COMPASS_DATA_RATIO;
      mz /= COMPASS_DATA_RATIO;
    #endif
    int throttle;
    int speed;
    int rpm;
    if(!obd.read(PID_THROTTLE, throttle)){
      throttle = 0;
    }
    if(!obd.read(PID_SPEED, speed)) {
      speed = 0;
    }
    if(!obd.read(PID_RPM, rpm)) {
      rpm = 0;
    }
    
    #if ACC_OFFSET
      if(speed == 0) {
        accXoffset = ax;
        accYoffset = ay;
        accZoffset = az;
      }
      ax -= accXoffset;
      ay -= accYoffset;
      az -= accZoffset;
    #endif
    
    #if USE_SERIAL_LOGGING
      Serial.print("\nLon: ");
      Serial.print(lon);
      Serial.print(", ");
      Serial.print(lon/100000);
      Serial.print('.');
      uint32_t poslon;
      if(lon<0) {
        poslon=lon*-1;
      } else {
        poslon=lon;
      }
      for(uint32_t i = 10000; i > 10; i/=10) {
        if(poslon%(i*10) < i) {
          Serial.print('0');
        } else {
          break;
        }
      }
      if(poslon%100000 < 10000) {
        Serial.print(poslon%100000);
      } else {
        Serial.print(poslon%100000);
      }
      Serial.print("\nLat: ");
      Serial.print(lat);
      Serial.print(", ");
      Serial.print(lat/100000);
      Serial.print('.');
      uint32_t poslat;
      if(lat<0) {
        poslat=lat*-1;
      } else {
        poslat=lat;
      }
      for(uint32_t i = 10000; i > 10; i/=10) {
        if(poslat%(i*10) < i) {
          Serial.print('0');
        } else {
          break;
        }
      }
      if(poslat%100000 < 10000) {
        Serial.print(poslat%100000);
      } else {
        Serial.print(poslat%100000);
      }
    #endif
    
    //            Lap,Timestamp (ms),Distance (km),Locked satellites,Latitude (deg),Longitude (deg),Speed (kph),Altitude (m),Bearing (deg),Longitudinal Acceleration (G),Lateral Acceleration (G),RPM (rpm),Throttle Percentage
    logger.logAll(-1, millis(), distance/1000, numSat, lat, lon, speed, alt, my, az, ax, rpm, throttle);
    /* #if USE_SERIAL_LOGGING
      Serial.print("\nWritten to SD Card: ");
    #endif */
  #endif
  //\\
  
}

void loop()
{

  /* if (stringComplete) {
    Serial.println("Input: " + inputString);
    if(inputString == "restart") {
      Reboot();
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  } */
  #if USE_SERIAL_LOGGING
    while (Serial.available() > 0) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      inputString += inChar;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if (inChar == '\n') {
        stringComplete = true;
      }
    }
    
    if(inputString == "restart") {
      Serial.println("\n\nARDUINO REBOOTING\n\n\n\n\n\n\n\n\nARDUINO REBOOTING");
      Reboot();
    } else if(inputString == "stop") {
      Serial.println("\n\nARDUINO HALTING");
      Halt();
    } else if(inputString == "close"){
      Serial.println("\n\CLOSING SD FILE");
      logger.closeFile();
    } else if(inputString == "open"){
      Serial.println("\n\OPENING SD FILE");
      logger.openFile();
    } else if(inputString.length() > 0) {
      Serial.print("\nInput: ");
      Serial.print(inputString);
    }
  #endif

  tX = -1;
  tY = -1;
  getTouch();
  
  inputString = "";
  stringComplete = false;

  static byte index = 0;
  static byte index2 = 0;
  static byte index3 = 0;
  uint32_t t = millis();
  byte pid;

  #if ENABLE_DATA_LOG || ENABLE_DATA_FILE
    // flush SD data every 1KB
    if ((logger.dataSize >> 10) != lastFileSize) {
      logger.flushFile();
      // display logged data sizes
      lcd.setFontSize(FONT_SIZE_SMALL);
      lcd.setCursor(242, 28);
      lcd.print((unsigned int)(logger.dataSize >> 10));
      lcd.print("KB");
      lastFileSize = logger.dataSize >> 10;
    }
    #if USE_SERIAL_LOGGING
      Serial.print("\nSD Card log size: ");
      Serial.print(logger.dataSize);
    #endif
  #endif
  //\\
  
  pid = pgm_read_byte(pidTier1 + index);
  logOBDData(pid);
  index++;
  t = millis() - t;

  if (index == TIER_NUM1) {
    index = 0;
    if (index2 == TIER_NUM2) {
      index2 = 0;
      pid = pgm_read_byte(pidTier3 + index3);
      if (obd.isValidPID(pid)) {
        logOBDData(pid);
      }
      index3 = (index3 + 1) % TIER_NUM3;
      if (index3 == 0) {
        float v = obd.getVoltage();
        //ShowVoltage(v);
        logger.logData(PID_BATTERY_VOLTAGE, (int)(v * 100));
      }
    } else {
      pid = pgm_read_byte(pidTier2 + index2);
      if (obd.isValidPID(pid)) {
        logOBDData(pid);
      }
      index2++;
    }
  }
<<<<<<< HEAD
  if (logger.dataTime == lastTime) {
    #if USE_SERIAL_LOGGING
    Serial.print("\nERROR GETTING CURRENT TIME! USING BACKUP CLOCK!");
    #endif
    /* lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(0,24);
    lcd.setColor(RGB16_RED);
    lcd.print("GPS ERROR"); */
    logger.dataTime = millis();
  } else {
    /* lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(0,24);
    lcd.setColor(BackgroundColor);
    lcd.print("         "); */
  }
  if (logger.dataTime -  lastRefreshTime >= 1000) {
=======

  if (logger.dataTime - lastRefreshTime >= 1000) {
>>>>>>> Campbell
    char buf[12];
    // display elapsed time
    unsigned int sec;
    if(fileOpen) {
      sec = (logger.dataTime - logStartTime) / 1000;
    } else {
      sec = (logger.dataTime - startTime) / 1000;
    }
    sprintf(buf, "%02u:%02u ", sec / 60, sec % 60);
    lcd.setFontSize(FONT_SIZE_XLARGE);
    lcd.setCursor(0, 4);
    lcd.print(buf);
    #if USE_SERIAL_LOGGING
      Serial.print("\nElapsed: ");
      Serial.print(buf);
      Serial.print(", Time: ");
      Serial.print(logger.dataTime);
      Serial.print(", LastRefreshTime: ");
      Serial.print(lastRefreshTime);
    #endif
<<<<<<< HEAD
    lastRefreshTime = logger.dataTime;
  }
  lastTime = logger.dataTime;
=======
    // display OBD time
    if (t < 30000) {
      lcd.setFontSize(FONT_SIZE_SMALL);
      lcd.setCursor(242, 26);
      lcd.printInt((uint16_t)t);
      lcd.print("ms");
      lcd.printSpace(2);
    }
    lastRefreshTime = logger.dataTime;
  }
  else if (logger.dataTime == lastRefreshTime) {
    #if USE_SERIAL_LOGGING
    Serial.print("\nERROR GETTING CURRENT TIME! USING BACKUP CLOCK!");
    #endif
    logger.dataTime = millis();
  }
>>>>>>> Campbell

  int value;
  if (obd.errors >= 5) {
    #if USE_SERIAL_LOGGING
<<<<<<< HEAD
      char buf[12];
      unsigned int sec;
      if(fileOpen) {
        sec = (logger.dataTime - logStartTime) / 1000;
      } else {
        sec = (logger.dataTime - startTime) / 1000;
      }
=======
      /* char buf[12];
      unsigned int sec = (logger.dataTime - startTime) / 1000;
>>>>>>> Campbell
      sprintf(buf, "%02u:%02u ", sec / 60, sec % 60);
      Serial.print("\nElapsed: ");
      Serial.print(buf);
      Serial.print(", Time: ");
      Serial.print(logger.dataTime);
      Serial.print(", LastRefreshTime: ");
      Serial.print(lastRefreshTime); */
      Serial.print("\nReconnecting to OBD ");
    #endif
    if(reconnect()) {
      lcd.setFontSize(FONT_SIZE_XLARGE);
      lcd.setColor(RGB16_GREEN);
      lcd.setCursor(0, 0);
      lcd.print("OBD CONNECTED   ");
      #if USE_SERIAL_LOGGING
      Serial.print("\nOBD Connected");
      #endif
    } else {
      lcd.setFontSize(FONT_SIZE_XLARGE);
      lcd.setColor(RGB16_YELLOW);
      lcd.setCursor(0, 0);
      lcd.print("OBD DISCONNECTED");
      #if USE_SERIAL_LOGGING
      Serial.print("\nOBD Disconnected");
      #endif
    }
    lcd.setColor(RGB16_WHITE);
    obd.errors = 0;
  } else if (obd.read(PID_RPM, value)) {
    lcd.setFontSize(FONT_SIZE_XLARGE);
    lcd.setColor(RGB16_GREEN);
    lcd.setCursor(0, 0);
    lcd.print("OBD CONNECTED   ");
    lcd.setColor(RGB16_WHITE);
  }

  #if USE_GPS
    float flat, flon;
    unsigned long fix_age; // returns +- latitude/longitude in degrees
    gps.f_get_position(&flat, &flon, &fix_age);
    if (fix_age == TinyGPS::GPS_INVALID_AGE)
      Serial.print("\nGPS: No fix detected");
    else if (fix_age > 5000)
      Serial.print("\nGPS: Warning: possible stale data!");
    else
      Serial.print("\nGPS: Data is current.");
    if (millis() - lastGPSDataTime > GPS_DATA_TIMEOUT || gps.satellites() < 3) {
      // GPS not ready
      state &= ~STATE_GPS_READY;
      #if USE_SERIAL_LOGGING
        Serial.print("\nGPS Not Ready");
        #if OBD_BREAKOUT
          if(gps.satellites() >= 3) {
            Serial.print(" (Error could be because OBD tried to reconnect. If it did not attempt to reconnect immediately before then there is another error, otherwise could mean that GPS is Ready.)");
          } else {
            Serial.print(" (No satellites in sight)");
          }
          Serial.print("\nDelay, Sat: ");
          Serial.print(millis() - lastGPSDataTime);
          Serial.print(", ");
          Serial.print(gps.satellites());
        #endif
      #endif
    } else {
      // GPS ready
      state |= STATE_GPS_READY;
      /* #if USE_SERIAL_LOGGING
        Serial.print("\nGPS Ready!");
      #endif */
    }
  #endif
}

void Reboot() {
  logger.closeFile();
  wdt_enable(WDTO_250MS);
  while(1){}
}
void Halt() {// FREEZES ARDUINO
  logger.closeFile();
  while(1) {
    Narcoleptic.delay(1000);
  }
}
/* void serialEvent() {
  Serial.print("\nSerial Event");
  while (Serial.available() > 0) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      Serial.print("\n");
    } else {
      Serial.print(inChar);
    }
  }
}
 */

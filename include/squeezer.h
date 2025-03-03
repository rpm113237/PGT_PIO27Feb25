#ifndef _SQUEEZER_H
#define _SQUEEZER_H
#include <NimBLEDevice.h>
#include <TickTwo.h>
#include <Adafruit_NeoPixel.h>
#include <HX711.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> //
#include <Wire.h>
#include <Preferences.h>
// OTA includes--taken from ElegantOTA demo example
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>

// NeoPins
extern const int NEOPIN;
extern const int NEOPIXELS; // One LED
extern const int Batt_CK_Interval;

extern const long int Batt_CK_Interval;
extern const int CNCT_LED_BLINK_TIME; // BLINK TIME FOR CONNECT LED
extern const long int CONN_WAIT_TM;   // time to wait to connect
extern const int SHAVE_HAIRCUT;       // use esc for shave and haircut
extern const int Battmah;
extern const int Runmah;
extern const int BattFullTime; // in minutes

// Seems if these have to be declared before TicTwo

SFE_MAX1704X lipo(MAX1704X_MAX17048); // Create a MAX17048
// Preferences prefs;
extern WebServer server;
HX711 scale;
Adafruit_NeoPixel pixels(NEOPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800); // 1 ea sk6812 on IO 8
TickTwo LEDtimer(LEDBlink, 10, 0, MILLIS);                         // calls LEDBlink, called every 10MS, repeats forever, resolution MS
TickTwo BattChecker(BatSnsCk, Batt_CK_Interval, 0, MILLIS);        // checks battery every Batt_Ck_Interval
TickTwo SleepChecker(RunTimeCheck, 10000, 0, MILLIS);              // check sleeptimers every ten seconds

extern String REV_LEVEL; // last part of commit number

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;

// Note--number of samples are caclulated in timesinit(); i.e HFrate = 5 => 80/6 = 16 samples
extern const int BaseSampleRate; // scale 80 sps
extern const int DfltFFRate;     // samples persecond
extern const int DfltHFRate;     // samples per second

extern const int DfltMeanTime; // Note--period as opposed to Time
extern const String DefaultSSID;
extern const String DefaultPWD;
extern int bootCount; // keep track of how many times since power on TODO--put this in flash

extern unsigned long oldmillis;   // to time the states
extern unsigned long int el_time; // elapsed time
// unsigned long EpochTime;    //for FF reporting

extern float scaleVal;            // scale data
extern const float scaleCalDeflt; // measured on SN10
extern const int NumWarmup = 10;
extern const int NumTare = 10;
// char TxString[25];  // used to transmit

extern bool deviceConnected; // defined in main
extern bool oldDeviceConnected;
// float txValue = 0;
extern String rxValue; // so can process outside of callback; maybe not the best idea
extern Preferences prefs;

// boolean to control serial diagnostic printouts.
extern bool SerOutHF;   // if true, serial out HF
extern bool SerOutFF;   // if true, serial out FF
extern bool SerOutMN;   // if true, serial out MN
extern bool SerOutIdle; // if true output idle time

// define pins
// HX711 pins:
extern const int HX711_dout = 39; // mcu > HX711 dout pin
extern const int HX711_sck = 38;  // mcu > HX711 sck pin
// GPIO
extern const int StartButton = 21; //
extern const int ShutDownPin = 20;
extern const int buzzPin = 48;
extern const int RatePin = 45; // = 0 for 10 sps, 1 for 80 sps
extern const int sda_rpm = 5;
extern const int scl_rpm = 4;

extern double BattVolts; // Variable to keep track of LiPo voltage
extern double BattSOC;   // Variable to keep track of LiPo state-of-charge (SOC)
extern double BattLife;  // calculated from lipo.getchangerate

struct ForceStruct
{
  /* Force is accumulated and averages calculated based on scale. Reporting is synchronous tied to TickTwo
  for now; ony rate for FF, HF is settable; the report time will be set as (1000/rate) milliseconds.
  */
  unsigned long EpochStart;      // Start of PGT EpochStart in ms
  unsigned long EpochTime;       // in ms == millis()-EpochStartTime
  unsigned long TotalRuntime;    // in seconds, in flash, updated at shutdown = millis() - epochstart
  int BaseRate = BaseSampleRate; // for Rev1--10 BASERATE #define 80--this is obsolete, I think
  // float BaseVal;  //updated every sample

  bool FFReport = true;    // if true, report FF
  int FFRate = DfltFFRate; // reports/sec
  int FFReportTime;        // report at this rate (ms)init in timesinit
  int FFNSamp;
  unsigned long FFLastReport; // millis of last report
  float FFVal;                // moving average over last BaseRate/FFRate Samples

  bool HFReport = true;
  int HFRate = DfltHFRate; // This is the samples per second the scale runs at
  int HFReportTime;        // number of milliseconds to report at init in inittimes()
  int HFNSamp;
  unsigned long HFLastReport; // millis of last hf report
  float HFVal;                // moving average over last BaseRate/HF rate samples

  bool MeanReport = true;
  int MeanTime = DfltMeanTime; // time(seconds) that Mean is calculated over
  int MeanReportTime;          // ms to report mean;  calc in inittimes
  int MNNSamp;
  unsigned long MeanLastReport;
  float MeanVal; // moving average over last MeanTime * BaseRate samples.
};
extern struct ForceStruct Force;

extern const int VIB_SND_INTERVAL; // = 1000; // ms

extern const int ditTime, chSpTime; // dit and dah
extern u_long cwFreq;

// int freq = 2000;
extern const int ledChannel;
extern const int resolution;
extern int dutycycle; // 127 = 50 percent +/-, max valume

struct COLORS
{
  int RED[3];
  int GREEN[3];
  int BLUE[3] = {0, 0, 255};
  int YELLOW[3] = {255, 255, 0};
  int WHITE[3] = {255, 255, 255};
  int OFF[3] = {0, 0, 0};
  int WKCLRS[3] = {0, 0, 0}; // used for the LED task.

};
extern struct COLORS clrs;

extern int tstarray[4];

int BlinkTime = CNCT_LED_BLINK_TIME; // blink ON/OFF TIME; if ==0, ON
extern int LEDSelect;                // 0 or 1; make enum
const int BatSns = 2;
const int NumADCRdgs = 10; // number of times to read ADC in floatADC
// float battvolts = 0.0;
extern float Batt_HI_Lvl;
extern float Batt_OK_Lvl;
extern float Batt_LO_Lvl;
extern float BatMultDefault; // TODO -find the nominal value
extern float BatSnsFactor;
// const int Batt_CK_Interval = 100 * MS_TO_SEC;
const int BattWarnPcnt = 40;     // turn connect LED Yellow/Orange
const int BattCritPcnt = 30;     // turn connect LED Red
const int BattShutDownPcnt = 20; // go to Sleep.pcnt
// #define Battmah 1000
// #define Runmah 70
// #define BattFullTime (Battmah / Runmah) * 60  //in minutes

extern uint16_t SleepTimer;      // in seconds reset if HF> MinForce
extern uint32_t SleepTimeMax;    // sleep timeout in sec
extern int MinForce;             // if HF < MinForce, sleeptimer
extern uint32_t SleepTimerStart; // if HF> MinForce, reset SleepTimerStart to current millis()/mstosec

// const int numSamples = 2;
extern long int scaleRead;

// Flash (preferences.h) setup
extern char SSstr[25]; // max from ble is about 20(?)- 2 for tag.
extern char PWDstr[25];
extern const char *ssid;
extern const char *password;

// protos
void setLED(int btime, int clrarray[]);
void VibSend(void);

void LEDBlink(void);
void BatSnsCk(void);
void RunTimeCheck(void);

void StringBLETX(String msg, bool SndSer);
// void LEDBlink(void);
// void BatSnsCk(void);
void BLEReconnect(void);
void CheckForce(void);
void RxStringParse(void);

// void SoundBuzz(u_long cwFreq, int sound_ms = 100);
void CalibrateADC(String strval);
void SetFFRate(String valStr);
void SetHFRate(String valStr);
void SetEpochTime(String valStr);
void SetMeanTime(String valStr);
void SetBootNum(String valStr);
void SetTotalRunTime(String valStr);
void ConnectWiFi(void);
void MorseChar(int cwChar);
void SetSSID(String ValStr);
void SetPwd(String ValStr);
void DoOTA(void);
void CalibrateScale(String strval);
void DoTare(void);
float getFloatADC(int numtimes);
// void RunTimeCheck(void);
void ResetSwitch(void);
void GoToSleep(String DSmsg);
void SendRev(String valStr);
void MeanSend(unsigned long ET);
void HFSend(unsigned long ET);
void FFSend(unsigned long ET);
void print_wakeup_reason();
void Soundwakeup(void);
void timesInit(void);
void setFlashDefaults(void);
unsigned long getEpochTime(void);
void SetVol(String valStr);

#endif
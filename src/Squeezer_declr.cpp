#include "squeezer.h"
TickTwo LEDtimer(LEDBlink, 10, 0, MILLIS);                         // calls LEDBlink, called every 10MS, repeats forever, resolution MS
TickTwo BattChecker(BatSnsCk, Batt_CK_Interval, 0, MILLIS);        // checks battery every Batt_Ck_Interval
TickTwo SleepChecker(RunTimeCheck, 10000, 0, MILLIS);              // check sleeptimers every ten seconds
Adafruit_NeoPixel pixels(NEOPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800); // 1 ea sk6812 on IO NeoPin
SFE_MAX1704X lipo(MAX1704X_MAX17048); // Create a MAX17048
HX711 scale;
//BLEServer *pserver=NULL;
// BLECharacteristic *pTxCharacteristic;

//#define MS_TO_SEC 1000 // convert to secs
String REV_LEVEL = "5Jan25 03ec5d0 ";  //last part of commit number
char SSstr[25] = "McClellan_Workshop";  //max from ble is about 20(?)- 2 for tag.
char PWDstr[25] = "Rangeland1";
const char* ssid = SSstr;
const char* password = PWDstr;
long int scaleRead = 0;

// NeoPins
const int NEOPIN = 47;
const int NEOPIXELS = 1; // One LED

const long int Batt_CK_Interval = 100 * MS_TO_SEC;
const int CNCT_LED_BLINK_TIME = 1000;         // BLINK TIME FOR CONNECT LED
const long int CONN_WAIT_TM = 25 * MS_TO_SEC; // time to wait to connect
const int SHAVE_HAIRCUT = 0x1b;               // use esc for shave and haircut
const int Battmah = 1000;
const int Runmah = 70;
const int BattFullTime = (Battmah / Runmah) * 60; // in minutes

// define pins
// HX711 pins:
const int HX711_dout = 39; // mcu > HX711 dout pin
const int HX711_sck = 38;  // mcu > HX711 sck pin
// GPIO
const int StartButton = 21; //
const int ShutDownPin = 20;
const int buzzPin = 48;
const int RatePin = 45; // = 0 for 10 sps, 1 for 80 sps
const int sda_rpm = 5;
const int scl_rpm = 4;

double BattVolts = 0; // Variable to keep track of LiPo voltage
double BattSOC = 0;   // Variable to keep track of LiPo state-of-charge (SOC)
double BattLife = 0;  // calculated from lipo.getchangerate

// Note--number of samples are caclulated in timesinit(); i.e HFrate = 5 => 80/6 = 16 samples
const int BaseSampleRate = 80; // scale 80 sps
const int DfltFFRate = 20;     // samples persecond
const int DfltHFRate = 5;      // samples per second
// const int DefaultHFReportTime = 200;  //ms
const int DfltMeanTime = 2; // Note--period as opposed to Time
const String DefaultSSID = "McClellan_Workshop";
const String DefaultPWD = "Rangeland1";
int bootCount=0; // keep track of how many times since power on TODO--put this in flash

unsigned long oldmillis;       // to time the states
unsigned long int el_time = 0; // elapsed time

float scaleVal =0.0;
float scaleCalVal = 8545.85;         // replace with typical number.
const float scaleCalDeflt = 8545.85; // measured on SN10
const int NumWarmup = 10;
const int NumTare = 10;

bool deviceConnected = false; // defined in main
bool oldDeviceConnected =false;
// float txValue = 0;
String rxValue; // so can process outside of callback; maybe not the best idea
Preferences prefs;

bool SerOutHF = true;  // if true, serial out HF
bool SerOutFF = true;  // if true, serial out FF
bool SerOutMN = true;  // if true, serial out MN
bool SerOutIdle = true;// if true output idle time

float Batt_HI_Lvl = 3.6;
float Batt_OK_Lvl = 3.5;
float Batt_LO_Lvl = 3.3;
float BatMultDefault = 0.001448;  //TODO -find the nominal value
float BatSnsFactor = 0.0;

struct ForceStruct Force; 
//TODO should this go in functions?
void initForce(void) // painful way to do it--has to be better way
{
    Force.BaseRate = BaseSampleRate;

    Force.FFReport = true;
    Force.FFRate = DfltFFRate; // reports/sec
    Force.HFReport = true;
    Force.HFRate = DfltHFRate; // This is the samples per second the scale runs at
    Force.HFLastReport;        // millis of last hf report

    Force.MeanReport = true;
    Force.MeanTime = DfltMeanTime; // time(seconds) that Mean is calculated over
}
const int VIB_SND_INTERVAL = 1000; // ms
const int ditTime = 75, chSpTime = 225;  //dit and dah
u_long cwFreq = 2500;

const int ledChannel = 0;
const int ledRes = 8;
int dutycycle= 127; // 127 = 50 percent +/-, max valume

struct COLORS clrs;

int BlinkTime = CNCT_LED_BLINK_TIME; // blink ON/OFF TIME; if ==0, ON
int LEDSelect =0;                // 0 or 1; make enum
const int BatSns = 2;
const int MS_TO_SEC =1000;
const int BattWarnPcnt = 40;     // turn connect LED Yellow/Orange
const int BattCritPcnt = 30;     // turn connect LED Red
const int BattShutDownPcnt = 20; // go to Sleep.pcnt

uint16_t SleepTimer;      // in seconds reset if HF> MinForce
uint32_t SleepTimeMax = 300;    // sleep timeout in sec
int MinForce =1;             // if HF < MinForce, sleeptimer
uint32_t SleepTimerStart; // if HF> MinForce, reset SleepTimerStart to current millis()/mstosec

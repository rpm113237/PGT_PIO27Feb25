#include "squeezer.h";
#define MS_TO_SEC 1000 // convert to secs

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
extern int bootCount=0; // keep track of how many times since power on TODO--put this in flash

unsigned long oldmillis;       // to time the states
unsigned long int el_time = 0; // elapsed time
// unsigned long EpochTime;    //for FF reporting

float scaleCalVal = 8545.85;         // replace with typical number.
const float scaleCalDeflt = 8545.85; // measured on SN10
const int NumWarmup = 10;
const int NumTare = 10;

bool deviceConnected; // defined in main
bool oldDeviceConnected;
// float txValue = 0;
String rxValue; // so can process outside of callback; maybe not the best idea
Preferences prefs;

struct ForceStruct {
    /* Force is accumulated and averages calculated based on scale. Reporting is synchronous tied to TickTwo
    for now; ony rate for FF, HF is settable; the report time will be set as (1000/rate) milliseconds.
    */
    unsigned long EpochStart;       //Start of PGT EpochStart in ms
    unsigned long EpochTime;        //in ms == millis()-EpochStartTime
    unsigned long TotalRuntime;     //in seconds, in flash, updated at shutdown = millis() - epochstart
    int BaseRate = BaseSampleRate;  //for Rev1--10 BASERATE #define 80--this is obsolete, I think
    //float BaseVal;  //updated every sample
  
    bool FFReport = true;     // if true, report FF
    int FFRate = DfltFFRate;  //reports/sec
    int FFReportTime;         //report at this rate (ms)init in timesinit
    int FFNSamp;
    unsigned long FFLastReport;  //millis of last report
    float FFVal;                 //moving average over last BaseRate/FFRate Samples
  
    bool HFReport = true;
    int HFRate = DfltHFRate;  //This is the samples per second the scale runs at
    int HFReportTime;         // number of milliseconds to report at init in inittimes()
    int HFNSamp;
    unsigned long HFLastReport;  //millis of last hf report
    float HFVal;                 //moving average over last BaseRate/HF rate samples
  
    bool MeanReport = true;
    int MeanTime = DfltMeanTime;  //time(seconds) that Mean is calculated over
    int MeanReportTime;           // ms to report mean;  calc in inittimes
    int MNNSamp;
    unsigned long MeanLastReport;
    float MeanVal;  //moving average over last MeanTime * BaseRate samples.
}  
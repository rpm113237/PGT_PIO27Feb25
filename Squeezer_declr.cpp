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
extern int bootCount = 0; // keep track of how many times since power on TODO--put this in flash

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

struct ForceStruct Force; 
int initForce(void) // painful way to do it--has to be better way
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

// int freq = 2000;
const int ledChannel = 0;
const int resolution = 8;
int dutycycle= 30; // 127 = 50 percent +/-, max valume
int tstarray[4]= {1,2,3,4};
struct COLORS clrs;

void init_clrs(void){
    clrs.RED[0] = 255;
    clrs.RED[1] = 0;
    clrs.RED[2] = 0;    
  //clrs.RED[3] = {255, 0, 0};
  clrs.GREEN[0]= 0; clrs.GREEN[1]=255; clrs.GREEN[2]= 0;
  //clrs.GREEN[3] = {0, 255, 0};
  clrs.BLUE[0] = 0;clrs.BLUE[1] = 0;clrs.BLUE[2] = 255;
//   clrs.BLUE[3] = {0, 0, 255};
//  clrs.YELLOW[3] = {255, 255, 0};
  clrs.WHITE[3] = {255, 255, 255};
  clrs.OFF[3] = {0, 0, 0};
  clrs.WKCLRS[3] = {0, 0, 0}; // used for the LED task.

}


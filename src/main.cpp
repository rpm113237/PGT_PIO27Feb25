#include <Arduino.h>
#include "squeezer.h"
#include <stdint.h>
//using namespace std;

// if it is the C3 or other single core ESP32" Use core 0
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else  //otherwise core 1
static const BaseType_t app_cpu = 1;
#endif

WebServer server(80);

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication at 115200 bits per second:
  u_long lagmsStart = millis();
  pinMode(StartButton, INPUT);
  pinMode(ShutDownPin, OUTPUT);    //pulled down with 100K, has to be active high
  digitalWrite(ShutDownPin, LOW);  // race against release. switch should still be closed.
  Serial.begin(115200); 
  ledcAttachPin(buzzPin,ledChannel);
  
  u_int32_t cwFreq= 2500;
  ledcChangeFrequency(ledChannel,cwFreq,resolution);
  //ledcAttach(buzzPin, cwFreq, resolution);  //eight bit resolution--why? (Jun24?); using for PWM
  setLED(00, clrs.GREEN);
  LEDBlink();     //Give them a green.
  Soundwakeup();  //wake up feedback

  while (digitalRead(StartButton) == HIGH) {
    Serial.println("Waiting Start Button Release");
  } //to avoid another shutdown

  Serial.println("Hello; PGT RevLevel =" + REV_LEVEL);
  // GoToSleep("Test Go Sleep, remove later");

  print_wakeup_reason();  //output and store wakeup count

  
  Wire.begin(sda_rpm, scl_rpm);    
  lipo.enableDebugging();     // Uncomment this line to enable helpful debug messages on Serial
  if (lipo.begin() == false)  // Connect to the MAX17048 using the default wire port
  {
    Serial.println(F("MAX17043 not detected. Please check wiring. Freezing forever."));
    while (1)
      ;
  }
  lipo.quickStart();  // Quick start restarts the MAX17048 in hopes of getting a more accurate guess for the SOC.

  //start the TickTwo timers
  LEDtimer.start();      //start LED timer
  BattChecker.start();   //start Batt checker
  SleepChecker.start();  //start sleepchecker
  //set up flash
  prefs.begin("SSID");  //prefs library wants length of string
  prefs.begin("PWD");
  prefs.begin("TotalRunTime");  //run time in minutes? 10K minutes = 166hrs.  Not long enough.
  prefs.begin("ScaleScale");
  prefs.begin("NumBoots");  //number of time the unit has powered up
  setFlashDefaults();

  //move next to connectWiFi??
  strcpy(SSstr, prefs.getString("SSID", DefaultSSID).c_str());  //SSstr is init in squeezer.h
  strcpy(PWDstr, prefs.getString("PWD", DefaultPWD).c_str());   //PWDstr is init in squeezer.h
  Serial.printf("SSID = %s \n", SSstr);
  Serial.printf("PWD = %s \n", PWDstr);

  //set up the scale
  pinMode(RatePin, OUTPUT);
  if (BaseSampleRate == 10) digitalWrite(RatePin, LOW);
  else if (BaseSampleRate == 80) digitalWrite(RatePin, HIGH);
  else Serial.println("Unknown Base Rate =" + String(BaseSampleRate));

  scale.begin(HX711_dout, HX711_sck);
  scale.reset();        //if coming out of deep sleep; scale might be powered down
  scale.tare(NumTare);  // reset the scale to 0

  scaleCalVal = prefs.getFloat("ScaleScale");
  if (isnan(scaleCalVal)) {
    Serial.println("Scale Cal not loaded, use default--check default typicality");
    scaleCalVal = scaleCalDeflt;
  }
  scale.set_scale(scaleCalVal);  //This value needs to be default; a cal function needs to be implemented
  Serial.printf("Scale setup, tared calvalue = %f  time = %lu ms\n", scaleCalVal, (millis() - lagmsStart));

  oldmillis = millis();
  el_time = millis() - oldmillis;
  setLED(500, clrs.BLUE);

  setupBLE();
  Serial.print("....Advertising for BLE connection.......\n");
  BlinkTime = CNCT_LED_BLINK_TIME;
  setLED(250, clrs.BLUE);  //blinking for connect
  Serial.println("Revision level = " + REV_LEVEL);
  BatSnsCk();  //sets color for connect led
  Serial.printf("VBAT = %.0f millivolts\n", BattVolts * 1000);
  Serial.printf("SOC = %.2f%%\n", BattSOC);
  

  timesInit();  //
}

//**************loop()***************************************
void loop() {
  BLEReconnect();   //
  ResetSwitch();    //check for OFF
  RxStringParse();  //check for orders from the boss (App)

  CheckForce();                          //check force updates force structure
  unsigned long lclET = getEpochTime();  //Epoch time stamp (ms)--could be done in CheckForce??

  FFSend(lclET);  // put send once in FFSend
  HFSend(lclET);
  MeanSend(lclET);

  BattChecker.update();   // BatSnsCk checks battery, doesn't send voltage--voltage send is query
  LEDtimer.update();      //should call the ledBlink every 10ms.
  SleepChecker.update();  //check for timeout

}  //end of loop

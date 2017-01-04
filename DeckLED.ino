/* 

LED Driver program
  v0.1 Terry Myers EZSoft-inc.com

Functional Requirements:
  1. Control a string of common anode RGB LEDs mounted underneath the railings of a deck via a dimmer switch or webpage
  2. Control a string of white LEDs mounted underneath a deck via a dimmer switch or motion detection

Detailed Design
  A server grade 12VDC nom. power supply supplies a rail of 12V to the arduino which has an ethernet shield and some protoshields for wiring
    This power supply has a 2A 12V rail that is always on and an input that when sinked to ground turns on a 60A rail which will power the LEDs
    The arduino can control this input via a transistor.  A SPDT with off toggle switch on the case allows the user to enable the arduino to control the power, or turn power on, or off.
  The arduino hosts a basic webpage that allows the user to control the lighting on the Deck, and allows access to some basic "cool" programs
  If the dimmer switch is moved, the RGB Deck LEDs are dimmed white according to the dimmer position.  The dimmer always makes the RGB white
    The RGB LEDs are color corrected to 3000K by capping the max value of the green and blue LED
    Dimmer switches use a voltage dividor.  The dimmer switch inputs are linearlized
    LED perceived brightness is nto linear with power, there he overall LED brightness is linearized to human perception with a gamma correction algorythm.
    Both dimmer switch analog inputs are heavily smoothed with a low pass filter.
  The Under Deck LEDs are just white LEDs and can only be controlled via an identical linearized dimmer switch
    Again the dimmer switch is linearized and the brightness is linearized to human perception.
    Generic 12V powered motion sensors (3 wire: Power, ground, relay switched 12V) are used to sense motion.
      The relay switched signal is passed through an LM7805 as a kind of "relay" of sorts as an input to the arduino.  An actual mechical relay should be used, but I didn't have one at the time of build.
    If motion is sensed the under deck LEDs will fade on for a preprogrammed time, then fade off.
  time.gov is polled once a day and an acurate time is maintained in a register.
    The motion sensor inputs are disabled during daylight according to monthly averages of civil twilight hours around Philadelphia PA
  All LEDs have a timeout timer if left on for more than 3 hours they will turn off
  All LEDs will turn off at midnigth on a one shot.
    
     

  
 */


#include <TimeLib.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <RGBMood.h>

//Low pass butterworth filter order=1 alpha1=0.001 
class  FilterBuLp001
{
  public:
    FilterBuLp001()
    {
      v[0]=0.0;
    }
  private:
    float v[2];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = (3.131764229192701265e-3 * x)
         + (0.99373647154161459660 * v[0]);
      return 
         (v[0] + v[1]);
    }
};
//Low pass butterworth filter order=1 alpha1=0.002 
class  FilterBuLp002
{
  public:
    FilterBuLp002()
    {
      v[0]=0.0;
    }
  private:
    float v[2];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = (6.244035046342855111e-3 * x)
         + (0.98751192990731428978 * v[0]);
      return 
         (v[0] + v[1]);
    }
};

//Low pass butterworth filter order=1 alpha1=0.01 
//95% of value in ~50 steps, cross over final value at ~120steps
class  FilterBuLp01
{
  public:
    FilterBuLp01()
    {
      v[0]=0.0;
    }
  private:
    float v[2];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = (3.046874709125380054e-2 * x)
         + (0.93906250581749239892 * v[0]);
      return 
         (v[0] + v[1]);
    }
};




//Setup PWM pins
  #define AOredPin 5
  #define AOgreenPin 6
  #define AObluePin 9
  #define AOUnderDeckPin 3

//Setup DO pins
  #define DOPowerPin 2

//Setup DI pins
  #define DIMotionPin 8

//Setup AI pins
  #define AIDeckDimmerPin 1
  #define AIUnderDeckDimmerPin 3

//Thermistor Setup
  #define THERMISTORPIN 0
  #define THERMISTORNOMINAL 100000     // resistance at 25 degrees C 
  #define TEMPERATURENOMINAL 25   // temp. for nominal resistance (almost always 25 C)
  #define BCOEFFICIENT 3950 // The beta coefficient of the thermistor (usually 3000-4000)
  #define SERIESRESISTOR 100250  // the value of the 'other' resistor
  float ThermistorTemp = 0.0;
  FilterBuLp01 ThermistorLPF;

//Debug Setup, Set to True to turn on DEBUG messaging via Serial
  bool debug=true;

//RGB setup
  //temporary values to set Deck Lighting before passing into RGBMood
  byte r;
  byte g;
  byte b;
  float GammaCorrection = 2.2; //LED light is not linear with power.  Any value between 2.2 and 2.8 gives good linearity
  int PWMResolution = 255;
  //Color correction, corrected for ~3000K
    byte rMax = 255;
    byte gMax = 197;
    byte bMax = 143;

//Setup some tags to monitor program performance including the number of loops (ScanCounter), and the total time through each loop (scan time) in microseconds
  unsigned long ScanCounter; //used for average
  unsigned long MinScanTimeuS = 4294967295; //smallest recorded ScanTime in us.  Set to the max number so that when a smaller number is recorded during the program its updated
  float AvgScanTimeuS; //ScanTime in us averaged over one sec
  unsigned long MaxScanTimeuS; //largest ScanTime in us
  unsigned long microsREM; //last micros() to remember

//Setup some temporary values
  byte btemp = 0;
  float ftemp = 0.0;
  int itemp = 0;

//Delay timer accumulator for powering off the main power a time after no poewr is required
  int DOPowerDelayOFFPRE = 10; //seconds to wait after no power needs to be applied to 12V rail before cutting power
  int DOPowerDelayOFFACC; //accumulator that is compared against the PRE

//Deck Dimmer
  float DeckDimmer; //A value that stores the current desired percent output in 0-100%  This is the output of a LPF
  FilterBuLp002 DeckDimmerLPF;
  float DeckDimmerREM;  //This is the output of a longer LPF that is compared to DeckDimmer to determine if DeckDimmer is changing.
  FilterBuLp001 DeckDimmerREMLPF;
  float DeckDimmerChangingLimit = 0.5; //When the difference between DeckDimmer and DeckDimmerREM is greater than this value a, bit is set,
  bool DeckDimmerChanging; //When the difference between DeckDimmer and DeckDimmerREM is greater than a certain value, this bit is set
  
  int DeckMode = 0; //0 = off, 1 = Ethernet Control, 2 = Dimmer control
  int DeckModeREM;   //Last Scan remember
  bool DeckDimmer0ONS;
  int DeckTimeoutACC; //seconds timer since the deck changed mode or brightness level
  int DeckTimeoutPRE = 10800; //Preset for Deck Mode 2 timeout in seconds 10800 =3hours
  int DeckModeEthernet=0; //Set a Type that determines what the deck should do during ethernet mode
  bool DeckModeEthernetCMDRecieved = 0;

//UnderDeckDimmer
  float UnderDeckDimmer; //A value that stores the current desired percent output in 0-100%  This is the output of a LPF
  FilterBuLp002 UnderDeckDimmerLPF;
  float UnderDeckDimmerREM; //This is the output of a longer LPF that is compared to UnderDeckDimmer to determine if UnderDeckDimmer is changing.
  FilterBuLp001 UnderDeckDimmerREMLPF;
  float UnderDeckDimmerChangingLimit = 0.5; //When the difference between DeckDimmer and DeckDimmerREM is greater than this value a, bit is set,
  bool UnderDeckDimmerChanging;  //When the difference between DeckDimmer and DeckDimmerREM is greater than a certain value, this bit is set

  int UnderDeckMode = 0; //0 = off, 1 = Motion Detected, 2 = Dimmer control
  int UnderDeckModeREM; //Last Scan remember
  int UnderDeckTimeoutACC; //seconds timer since the deck has been put into Mode 2.
  int UnderDeckTimeoutPRE = 10800; //Preset for Deck Mode 2 timeout in seconds 10800 =3hours
  int UnderDeckModeMotion=0; //Set a Type that determines what the deck should do during ethernet mode
  int UnderDeckModeMotionREM=0; //Remember register for type
  bool UnderDeckDimmer0ONS;
  float UnderDeckDimmerCurrent;
  float UnderDeckDimmerTarget;
  FilterBuLp002 UnderDeckOutputLPF; //Filter to delay the output by a few seconds to create a dimming effect


bool DIMotion; //State of the motion detected switches
bool DayNight=1; //0=day, 1=night


//Setup some precise pulse timers
//Set up boolean pulses that go true for one scan of the code.
  bool TimerOneSecondPulse = 0;
  bool TimerHalfSecondPulse = 0;
  bool TimerTenthSecondPulse = 0;
  bool TimerTenMSPulse = 0;
  bool TimerOneMSPulse = 0;

//Create an array to store pulse timer data
  const int PulseTimerLen = 5;
  unsigned long PulseTimerPRE[PulseTimerLen]; //Create an array of timer preset values
  unsigned long PulseTimerREM[PulseTimerLen]; //Create an array to store millis()

//Set array index values to reference the above timer elements
  const int TimerOneSecondPulseIndex = 4;
  const int TimerHalfSecondPulseIndex = 3;
  const int TimerTenthSecondPulseIndex = 2;
  const int TimerTenMSPulseIndex = 1;
  const int TimerOneMSPulseIndex = 0;

//Ethernet Setup
  #define EthernetSelectPin 4
  String readString;
  byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
  IPAddress ip(192, 168, 1, 240);
  EthernetServer server(80);
  EthernetUDP Udp;

//Setup RGBMood
  RGBMood m(AOredPin, AOgreenPin, AObluePin);

//Setup NTP
  unsigned int localPort = 8888;       // local port to listen for UDP packets
  IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
  //const int timeZone = 1;     // Central European Time
  const int timeZone = -5;  // Eastern Standard Time (USA)
  //const int timeZone = -4;  // Eastern Daylight Time (USA)
  //const int timeZone = -8;  // Pacific Standard Time (USA)
  //const int timeZone = -7;  // Pacific Daylight Time (USA)
  time_t prevDisplay = 0; // when the digital clock was displayed

//Store the civil twlight hour/minute of the 15th of each month in an array to set the day/night bit
int CivilTwlightStartMin[13]  ={0, 411, 385, 343, 292, 253, 238, 254, 285, 316, 345, 378, 406};
 
int CivilTwlightEndMin[13]    ={0,1040,1085,1116,1149,1181,1205,1199,1164,1114,1066,1032,1027};

//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
void setup() {

  //ethernet select pin
  pinMode(EthernetSelectPin, OUTPUT);
  digitalWrite(EthernetSelectPin, HIGH);

  //Setup pins
  pinMode(AOredPin, OUTPUT);
  pinMode(AOgreenPin, OUTPUT);
  pinMode(AObluePin, OUTPUT);
  pinMode(AOUnderDeckPin, OUTPUT);
  pinMode(DOPowerPin, OUTPUT);

  Serial.begin(115200);
  //while (!Serial) { delay(1);} // wait for serial port to connect. Needed for native USB
//  Serial.setTimeout(10);

  //Start Ethernet
  Ethernet.begin(mac, ip);
  Serial.print(F("IP:")) ;Serial.println(Ethernet.localIP());
  Udp.begin(localPort);
  server.begin();
  setSyncProvider(getNtpTime);
  setSyncInterval(86400); //once a day update the time
  
  //Set up the PRE for the pulse timers.  Note that the closest prime number was choosen to prevent overlapping with other timers
  PulseTimerPRE[TimerOneSecondPulseIndex] = 997;
  PulseTimerPRE[TimerHalfSecondPulseIndex] = 499; //Set up the PRE for the timer.  Note that the closest prime number was choosen to prevent overlapping with other timers
  PulseTimerPRE[TimerTenthSecondPulseIndex] = 101; //Set up the PRE for the timer.  Note that the closest prime number was choosen to prevent overlapping with other timers
  PulseTimerPRE[TimerTenMSPulseIndex] = 11;  //Set up the PRE for the timer.
  PulseTimerPRE[TimerOneMSPulseIndex] = 1;  //Set up the PRE for the timer.

  //Turn on the power, cycle all of the colors, then turn power off
  digitalWrite(DOPowerPin, HIGH);
  delay(500);
  LEDTest(); //Test Each LED channel
  digitalWrite(DOPowerPin, LOW);
}
//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP


//MAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOP
//MAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOP
void loop() {
  
  UpdateAllTimers();
  GetInputs();
  if (TimerTenthSecondPulse==1) {Web();}
  SetDeckMode(); //process the current mode of the Deck LEDs
  SetDeckModeEthernet(); //If the mode is based on ethernet set the Deck LED lighting level
  m.tick();
  SetUnderDeckMode(); //process the current mode of the Deck LEDs   
  SetOutputs();




  UpdateScanCounter(); //Keep this at the bottom of the loop
 
}
//MAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOP
//MAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOPMAINLOOP
//==============================================================================================
void Web() {
  // Create a client connection
  
  EthernetClient client = server.available();
  if (client) {
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        //read char by char HTTP request
        if (readString.length() < 100) {

          //store characters to string
          readString += c;
          //Serial.print(c);
        }

        //if HTTP request has ended
        if (c == '\n') {
         
          //print to serial monitor for debuging
         // Serial.println(F("___Client Request:___"));
          Serial.println(readString); //print to serial monitor for debuging
         // Serial.println(F("___End Client Request:___"));

          client.println(F("HTTP/1.1 200 OK")); //send new page
          client.println(F("Content-Type: text/html"));
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println();

          client.println(F("<HTML>"));
          client.println(F("<HEAD>"));
          client.println(F("<TITLE>Deck Lighting Control</TITLE>"));
          client.println(F("</HEAD>"));

          client.println(F("<BODY bgcolor=#000000"));

          client.print(F("<H1 style=color:#BABABA;font-size:200%;>Deck Lighting Control ("));
          client.print(ThermistorTemp,1); client.print("C)");
          client.println(F("</H1><br>"));
          
          client.print(hour());
          client.print(":");
          if (minute() < 10) {
            client.print(F("0"));
          }
          client.print(minute());
          if (isAM()==1) {
            client.print(F("am"));
          } else {
            client.print(F("pm<br>"));
            }
            
          client.print(F("Deck LEDs: "));
          if (DeckMode==0) { client.print(F("Off"));}
          if (DeckMode==1) { client.print(F("ENet"));}
          if (DeckMode==2) { client.print(F("Dimmer(")); client.print(DeckDimmer,0); client.print(F("%)"));}
          
            client.println(F("<br>"));
          client.print(F("Under Deck LEDs: "));
          if (UnderDeckMode==0) { client.print(F("Off"));}
          if (UnderDeckMode==1) { client.print(F("Motion"));}
          if (UnderDeckMode==2) { client.print(F("Dimmer(")); client.print(UnderDeckDimmer,0); client.print(F("%)"));}
         
          client.println(F("<br>"));
          String buttontype=F("<input type=submit value="); //Style is saved once and used throughout each button of the webpage
          String style=F(" style=width:20%;height:100px;font-size:40px onClick=location.href='/?"); //Style is saved once and used throughout each button of the webpage
          client.print(buttontype); client.print(F("'ALL OFF'")); client.print(style); client.println(F("Z;'><br>"));
          client.print(buttontype); client.print(F("OFF")); client.print(style); client.println(F("A;'>"));
            client.print(buttontype); client.print(F("Random")); client.print(style); client.println(F("M;'>"));
            client.println(F("<br>"));
          client.print(buttontype); client.print(F("min")); client.print(style); client.println(F("B;'>"));
            client.print(buttontype); client.print(F("Rainbow")); client.print(style); client.println(F("N;'>"));
            client.println(F("<br>"));
          client.print(buttontype); client.print(F("10%")); client.print(style); client.println(F("C;'>"));
            client.print(buttontype); client.print(F("Red")); client.print(style); client.println(F("O;'>"));
            client.println(F("<br>"));
          client.print(buttontype); client.print(F("20%")); client.print(style); client.println(F("?D;'>"));
            client.print(buttontype); client.print(F("Green")); client.print(style); client.println(F("P;'>"));
            client.println(F("<br>"));
          client.print(buttontype); client.print(F("30%")); client.print(style); client.println(F("E;'>"));
            client.print(buttontype); client.print(F("Blue")); client.print(style); client.println(F("Q;'>"));
            client.println(F("<br>"));
          client.print(buttontype); client.print(F("40%")); client.print(style); client.println(F("F;'>"));
            client.print(buttontype); client.print(F("Fire")); client.print(style); client.println(F("R;'>"));
            client.println(F("<br>"));
          client.print(buttontype); client.print(F("50%")); client.print(style); client.println(F("G;'>"));
            client.print(buttontype); client.print(F("Candle")); client.print(style); client.println(F("S;'>"));
            client.println(F("<br>"));
          client.print(buttontype); client.print(F("60%")); client.print(style); client.println(F("H;'><br>"));
          client.print(buttontype); client.print(F("70%")); client.print(style); client.println(F("I;'><br>"));
          client.print(buttontype); client.print(F("80%")); client.print(style); client.println(F("J;'><br>"));
          client.print(buttontype); client.print(F("90%")); client.print(style); client.println(F("K;'><br>"));
          client.print(buttontype); client.print(F("100%")); client.print(style); client.println(F("L;'><br>"));
          client.print(buttontype); client.print(F("MAX")); client.print(style); client.println(F("T;'><br>"));
          
          client.println(F("</BODY>"));
          client.println(F("</HTML>"));

          delay(1);
          //stopping client
          client.stop();
          int type=0;//1=Deck RGB change, 2=Deck RGBMode Change, 3=Brightness
          int temp = readString.indexOf('?');
          if (temp > 0) {
           // Serial.print("temp=");
          //  Serial.println(temp);
            if (readString.indexOf('Z', temp) == temp + 1) {DeckMode = 0; UnderDeckMode = 0; DeckModeEthernetCMDRecieved = 1;} //All Off;
            if (readString.indexOf('A', temp) == temp + 1) {DeckModeEthernet = 0;  DeckModeEthernetCMDRecieved = 1;} //LED OFF
            if (readString.indexOf('B', temp) == temp + 1) {DeckModeEthernet = 1;  DeckModeEthernetCMDRecieved = 1;} //LED min
            if (readString.indexOf('C', temp) == temp + 1) {DeckModeEthernet = 10;  DeckModeEthernetCMDRecieved = 1;} //LED 10
            if (readString.indexOf('D', temp) == temp + 1) {DeckModeEthernet = 20;  DeckModeEthernetCMDRecieved = 1;} //LED 20
            if (readString.indexOf('E', temp) == temp + 1) {DeckModeEthernet = 30;  DeckModeEthernetCMDRecieved = 1;} //LED 30
            if (readString.indexOf('F', temp) == temp + 1) {DeckModeEthernet = 40;  DeckModeEthernetCMDRecieved = 1;} //LED 40
            if (readString.indexOf('G', temp) == temp + 1) {DeckModeEthernet = 50;  DeckModeEthernetCMDRecieved = 1;} //LED 50
            if (readString.indexOf('H', temp) == temp + 1) {DeckModeEthernet = 60;  DeckModeEthernetCMDRecieved = 1;} //LED 60
            if (readString.indexOf('I', temp) == temp + 1) {DeckModeEthernet = 70;  DeckModeEthernetCMDRecieved = 1;} //LED 70
            if (readString.indexOf('J', temp) == temp + 1) {DeckModeEthernet = 80;  DeckModeEthernetCMDRecieved = 1;} //LED 80
            if (readString.indexOf('K', temp) == temp + 1) {DeckModeEthernet = 90;  DeckModeEthernetCMDRecieved = 1;} //LED 90
            if (readString.indexOf('L', temp) == temp + 1) {DeckModeEthernet = 100;  DeckModeEthernetCMDRecieved = 1;} //LED 100
            if (readString.indexOf('T', temp) == temp + 1) {DeckModeEthernet = 101;  DeckModeEthernetCMDRecieved = 1;} //LED Max

            if (readString.indexOf('M', temp) == temp + 1) {DeckModeEthernet = 200;  DeckModeEthernetCMDRecieved = 1;} //LED Random Hue Mode;
            if (readString.indexOf('N', temp) == temp + 1) {DeckModeEthernet = 300;  DeckModeEthernetCMDRecieved = 1;} //LED Rainbow Hue Mode;
            if (readString.indexOf('O', temp) == temp + 1) {DeckModeEthernet = 400;  DeckModeEthernetCMDRecieved = 1;} //LED Red Hue Mode;
            if (readString.indexOf('P', temp) == temp + 1) {DeckModeEthernet = 500;  DeckModeEthernetCMDRecieved = 1;} //LED Green Hue Mode;
            if (readString.indexOf('Q', temp) == temp + 1) {DeckModeEthernet = 600;  DeckModeEthernetCMDRecieved = 1;} //LED Blue Hue Mode;
            if (readString.indexOf('R', temp) == temp + 1) {DeckModeEthernet = 700;  DeckModeEthernetCMDRecieved = 1;} //LED Fire Hue Mode;
            if (readString.indexOf('S', temp) == temp + 1) {DeckModeEthernet = 800;  DeckModeEthernetCMDRecieved = 1;} //LED Candle Hue Mode;
            
          }

 
          //clearing string for next read
          readString = "";
        }
      }
    }
  }
}
//==============================================================================================
void LEDTest() {
  //Set each color as a debug tool
  m.setRGB(255, 0, 0); m.tick(); delay(500);
  m.setRGB(0, 255, 0); m.tick(); delay(500);
  m.setRGB(0, 0, 255); m.tick();  delay(500);
  m.setRGB(0, 0, 0); 
  analogWrite(AOUnderDeckPin,255); delay(500);
}
//==============================================================================================
byte GammaValue(byte value, byte Max) {
  //Perform a gamma correction.  LEDs do nto have a linear brightness.  This helps to simulate that.
  byte temp;

  //Ensure all LEDs turn on at the same time since different colors could have different starting points for when they turn on.
  if (value < 10) {
    temp = 0;
  } else if (value >= 10 && value < 20) { //Generally all Max's would be 1 up to number 20, so this ensures that all LEDs are at level 1/255 between 10 and 20
    temp = 1;
  } else if (value > 255) {
    temp = 255;
  } else {//calculate gamma
     temp = byte(round(pow( float(value) / float(PWMResolution), GammaCorrection) * float(Max) + 0.49));
  }
  return temp;
}
//==============================================================================================
void SetDeckMode() {

  //Capture if the deck dimmer is changing by comparing the values of the outputs of two different LPFs
  if (DeckDimmerREM > DeckDimmer+DeckDimmerChangingLimit | DeckDimmerREM < DeckDimmer-DeckDimmerChangingLimit) {
    DeckDimmerChanging=true;
  } else {
    DeckDimmerChanging=false;
  }
      
  if (DeckDimmer > 0.0 & DeckDimmerChanging==true) { 
    DeckMode = 2;
    btemp = byte(Scalar(DeckDimmer,0.0,100.0,0.0,255.0));
    r = GammaValue(btemp, rMax);
    g = GammaValue(btemp, gMax);
    b = GammaValue(btemp, bMax);
    m.setRGB(r, g, b);
  }
  
  if (DeckDimmer == 0.0 & DeckDimmer0ONS==false) {
    DeckMode = 0;
    DeckDimmer0ONS=true;
  } else if (DeckDimmer > 0.0) {
    DeckDimmer0ONS=false;
  }
  
  if (DeckModeEthernetCMDRecieved==true  & DeckDimmerChanging==false) { DeckMode = 1;  }
  

  //====Deck lights timeout timer.
      //Reset timer if something changed
      if (DeckModeEthernetCMDRecieved==true | DeckDimmerChanging==true) {DeckTimeoutACC=0;}//Reset the timeout timer if the DeckMode Ethernet Changed or dimmer is changing
     if (DeckMode !=0) {
      if (TimerOneSecondPulse==1) {DeckTimeoutACC++; }//increment a timer once a second while the deck lights are on
      if (DeckTimeoutACC >= DeckTimeoutPRE) {DeckMode = 0;}//If the timer is greater than the preset, turn the deck lights off  
    } else {
      DeckTimeoutACC=0;
    }

   
   //Turn off the lights at night once at midnight
   if (hour()==0 & minute()==0 & second()<=10 ) {DeckMode = 0;}

  //Set some text when mdoe changes
    if (DeckMode!=DeckModeREM) {
      if (DeckMode==0) { 
        Serial.println(F("Deck 0(off)"));}
        m.setMode(RGBMood::FIX_MODE);
        m.setFadingSteps(200); // 200 steps.
        m.setFadingSpeed(25); // 25 ms * 200 steps = 5 seconds.
        m.fadeRGB(0, 0, 0);
      if (DeckMode==1) { 
        //Serial.println(F("Deck 1(Enet)"));
        }
      if (DeckMode==2) {
        //Serial.println(F("Deck 2(Dimmer)"));
        }
    }
    DeckModeREM = DeckMode;

}
//=============================================================================================
void SetDeckModeEthernet() {
  
      
if (DeckModeEthernetCMDRecieved==1 & DeckMode==1) {  //If the Deck Type changed
  
 
    if (DeckModeEthernet >= 0 & DeckModeEthernet <= 101) { //Ethernet set between min, 10 and 100%, and max

      if (DeckModeEthernet==0) {
        DeckMode = 0; //turn deck LEDs off
      }
      
       //if minimim is choosen overwrite rgb
       if (DeckModeEthernet == 1) {
          r = 1;  g = 1;  b = 1;
       }
       
        if (DeckModeEthernet >= 10 & DeckModeEthernet <= 100) { //Ethernet set between  10 and 100%
           btemp = byte(float(DeckModeEthernet) / 100.0 * 255.0); //calculate programmatically what the percentage should be
           r = GammaValue(btemp, rMax);
           g = GammaValue(btemp, gMax);
           b = GammaValue(btemp, bMax);
        }
        
       
       //if max is choosen overwrite rgb
       if (DeckModeEthernet == 101) {
          r = 255;  g = 255;  b = 255;
       }
       
       m.setMode(RGBMood::FIX_MODE);
       m.setFadingSteps(200); // 200 steps.
       m.setFadingSpeed(5); // 25 ms * 200 steps = 5 seconds.
       m.fadeRGB(r, g, b);
       
    }
 
  if ( DeckModeEthernet == 200 ) {
    m.setFadingSteps(200); // 200 steps.
    m.setFadingSpeed(25); // 25 ms * 200 steps = 5 seconds.
    m.setHoldingTime(0);
    m.setMode(RGBMood::RANDOM_HUE_MODE);
    m.setHoldingTime(5000);
    m.fadeHSB(0, 255, 255); // Rainbow mode only change Hue so we first set the saturation and brightness.
  }
  if (DeckModeEthernet == 300 ) {
    m.setMode(RGBMood::RAINBOW_HUE_MODE); // Internally fade from red to red.
    m.setFadingSteps(255); // 200 steps.
    m.setFadingSpeed(78); // 78 ms * 255 steps = 20 seconds.
    m.setHoldingTime(0); // No need to stay red.
    m.setHSB(0, 255, 255); // Rainbow mode only change Hue so we first set the saturation and brightness.
  }
  if (DeckModeEthernet == 400 ) {
    m.setFadingSteps(200); // 200 steps.
    m.setFadingSpeed(25); // 25 ms * 200 steps = 5 seconds.
    m.setHoldingTime(0);
    m.setMode(RGBMood::RED_MODE);
  }
  if (DeckModeEthernet == 500 ) {
    m.setFadingSteps(200); // 200 steps.
    m.setFadingSpeed(25); // 25 ms * 200 steps = 5 seconds.
    m.setHoldingTime(0);
    m.setMode(RGBMood::GREEN_MODE);
  }
  if (DeckModeEthernet == 600 ) {
    m.setFadingSteps(200); // 200 steps.
    m.setFadingSpeed(25); // 25 ms * 200 steps = 5 seconds.
    m.setHoldingTime(0);
    m.setMode(RGBMood::BLUE_MODE);
  }
  if (DeckModeEthernet == 700 ) {
    m.setHoldingTime(0);
    m.setMode(RGBMood::FIRE_MODE);
  }
  if (DeckModeEthernet == 800 ) {
    m.setFadingSteps(10); // 200 steps.
    m.setFadingSpeed(5); // 25 ms * 200 steps = 5 seconds.
    m.setHoldingTime(0);
    m.setMode(RGBMood::CANDLE_MODE); // CANDLE MODE
  }
}

DeckModeEthernetCMDRecieved=0;
}
//==============================================================================================
void SetUnderDeckMode() {
//TRANSITIONS=====================
  //Handle transitions of the Deck dimmer from off to on and on to off

  //Capture if the deck dimmer is changing by comparing the values of the outputs of two different LPFs
  if (UnderDeckDimmerREM > UnderDeckDimmer+UnderDeckDimmerChangingLimit | UnderDeckDimmerREM < UnderDeckDimmer-UnderDeckDimmerChangingLimit) {
    UnderDeckDimmerChanging=true;
  } else {
    UnderDeckDimmerChanging=false;
  }

  //Set the Mode
  if (UnderDeckDimmer > 0.0 & UnderDeckDimmerChanging==true) { UnderDeckMode = 2; } //If dimmer is greater than zero and changing
  if (UnderDeckDimmer == 0.0 & UnderDeckDimmer0ONS==false) {
    UnderDeckMode = 0;
    UnderDeckDimmer0ONS=true;
  } else if (UnderDeckDimmer > 0.0) {
    UnderDeckDimmer0ONS=false;
  }
  
  CheckDayNight();
  
  if (DayNight==true & DIMotion==true  & UnderDeckMode==0 & UnderDeckDimmerChanging==false) { 
    UnderDeckMode = 1;
  } else if ( DIMotion==false & UnderDeckMode==1 & UnderDeckDimmerChanging==false) {
    UnderDeckMode=0;
  }

  //Based on Mode Set the Target to fade to
  if (UnderDeckMode == 0) {
    UnderDeckDimmerTarget=0.0;
  } else if (UnderDeckMode==1) { //DIMotion detected
    UnderDeckDimmerTarget = 100.0;
  } else if (UnderDeckMode==2) {//Dimmer control
    UnderDeckDimmerTarget = UnderDeckDimmer;
  }

  //====Deck lights timeout timer.
    //Reset timer if the dimmer changed changed
    if (UnderDeckDimmerChanging==true) {UnderDeckTimeoutACC=0;}//Reset the timeout timer if the DeckMode Ethernet Changed or dimmer is changing
    if (UnderDeckMode==2) { //While the mode is dimmer controlled, accumullate a timer
       if (TimerOneSecondPulse==1) {UnderDeckTimeoutACC++; }//increment a timer once a second while the deck lights are on
       if (UnderDeckTimeoutACC >= UnderDeckTimeoutPRE) {UnderDeckMode = 0;}//If the timer is greater than the preset, turn the deck lights off  
    } else {
      UnderDeckTimeoutACC=0; //Reset the timer when the DeckMode==0
    }

   //Turn off the lights at night once at midnight
   if (hour()==0 & minute()==0 & second()<=10 ) {UnderDeckMode = 0;}
   

  //Set some text when mode changes
    if (UnderDeckMode!=UnderDeckModeREM) {
      if (UnderDeckMode==0) { Serial.println(F("UDeck 0(Off)"));}
      if (UnderDeckMode==1) { Serial.println(F("UDeck 1(Motion)"));}
      if (UnderDeckMode==2) { Serial.println(F("UDeck 2(Dimmer)"));}
    }
    
    UnderDeckModeREM = UnderDeckMode;



}

//==============================================================================================
void GetInputs() {
  //Stagger the input reads

  if (TimerTenMSPulse == 1) {
    itemp = analogRead(AIDeckDimmerPin); //0-1023
    
    ftemp = DeckDimmerLPF.step(float(itemp)); //smoothed 0-1023
    DeckDimmer = LinearizeDimmer(float(ftemp)); //0-100%
    
    ftemp = DeckDimmerREMLPF.step(float(itemp)); //smoothed 0-1023
    DeckDimmerREM = LinearizeDimmer(float(ftemp)); //0-100%
     /*
    Serial.print(F("DeckDimmer="));
    Serial.print(DeckDimmer);
    Serial.print(F("     itemp="));
    Serial.print(itemp);
    Serial.print(F("     ftemp="));
    Serial.print(ftemp);
    Serial.print(F("     btemp="));
    Serial.print(byte(Scalar(DeckDimmer,0.0,100.0,0.0,255.0)));
    Serial.println("");
    */
    
    itemp = analogRead(AIUnderDeckDimmerPin); //0-1023
    
    ftemp = UnderDeckDimmerLPF.step(float(itemp)); //smoothed 0-1023
    UnderDeckDimmer = LinearizeDimmer(ftemp); //0-100%
    
    ftemp = UnderDeckDimmerREMLPF.step(float(itemp)); //smoothed 0-1023
    UnderDeckDimmerREM = LinearizeDimmer(ftemp); //0-100%

    /*
    Serial.print(F("UnderDeckDimmer="));
    Serial.print(UnderDeckDimmer);
    Serial.print(F("     itemp="));
    Serial.print(itemp);
    Serial.print(F("     ftemp="));
    Serial.print(ftemp);
    Serial.print(F("     btemp="));
    Serial.print(byte(Scalar(UnderDeckDimmer,0.0,100.0,0.0,255.0)));
    Serial.println("");
    */
    
  }
  
  if (TimerHalfSecondPulse == 1) {
    DIMotion = digitalRead(DIMotionPin);
    ThermistorTemp = steinhart(float(analogRead(THERMISTORPIN)));
  }

  if (TimerOneSecondPulse == 1 & debug==1) {
    Serial.print(F("DeckDimmer="));
    Serial.print(DeckDimmer);
    Serial.print(F("%"));
    Serial.print(F("("));
    Serial.print(analogRead(AIDeckDimmerPin));
    Serial.print(F(")"));
    Serial.print(F("   "));
    
    Serial.print(F("DeckDimmerREM="));
    Serial.print(DeckDimmerREM);
    Serial.print(F("%"));
    Serial.print(F("   "));
    
    Serial.print(F("UnderDeckDimmer="));
    Serial.print(UnderDeckDimmer);
    Serial.print(F("%"));
    Serial.print(F("("));
    Serial.print(analogRead(AIUnderDeckDimmerPin));
    Serial.print(F(")"));
    Serial.print(F("   "));
    
    Serial.print(F("UnderDeckDimmerREM="));
    Serial.print(UnderDeckDimmerREM);
    Serial.print(F("%"));
    Serial.print(F("   "));
    
    Serial.print(F("Motion="));
    Serial.print(DIMotion);
    Serial.print("(");
    if (DayNight==0) {
    Serial.print("Day");
    }else {
    Serial.print("Night");
    }
    Serial.print(")");
    digitalClockDisplay();
    Serial.print(F("   "));
    
    Serial.print(F("Temp="));
    Serial.print(ThermistorTemp);
    Serial.print(F("C"));
    Serial.print(F("("));
    Serial.print(analogRead(THERMISTORPIN));
    Serial.print(F(")"));
    Serial.println("");
  }

}
//=========================================================================================
float LinearizeDimmer(float Raw) {
  
  //Linearize a 250K poteniometer that is 0ohms when allt he way to the left
  //~10K ohms when barely off the "zero"
  //~225Kohms when almost all the way to the right
  //Open circuit when indented to the right
  //potentiometer is fed with 5V rail, and has has a 220KOhm pulldown resistor in a dividor network
  if (Raw<667.0) {
    return Scalar(Raw, 667.0, 506.0, 50, 100.0);
  } else
    return Scalar(Raw, 1000.0, 667.0, 0.0, 50.0);
}
//=========================================================================================
void SetOutputs() {
  
  if (DeckMode == 0 & UnderDeckMode == 0) {
    if (TimerOneSecondPulse==1 & DOPowerDelayOFFACC < DOPowerDelayOFFPRE) {DOPowerDelayOFFACC++;} //Delay turning off main power for 10sec to allow LEDs to fade out and to avoid uncessesary cycling
    if (DOPowerDelayOFFACC >= DOPowerDelayOFFPRE) {digitalWrite(DOPowerPin, LOW);};
  } else {
    digitalWrite(DOPowerPin, HIGH);
    if (TimerOneSecondPulse==1) {Serial.println("Power on");}
    DOPowerDelayOFFACC=0;
  }

  if (TimerTenMSPulse==1 ) { //on a regular interval use an LPF to dim the lights toward the target or to zero
    UnderDeckDimmerCurrent=UnderDeckOutputLPF.step(UnderDeckDimmerTarget);
    btemp = GammaValue(byte(Scalar(UnderDeckDimmerCurrent,0.0,100.0,0.0,255.0)),255); //Scale to a byte and gamma correct
    analogWrite(AOUnderDeckPin,btemp);
  }
}
//===========================================================================
float Scalar(float Raw, float RawLo, float RawHi, float ScaleLo, float ScaleHi) {
  //perform Y = m*x + b scaling
  float m;
  float b;
  if (RawHi - RawLo == 0) {
    return 0; //Prevent DIV0
  }
  m = (ScaleHi - ScaleLo) / (RawHi - RawLo);
  b = ScaleHi - m * RawHi;
  return constrain(Raw * m + b, ScaleLo, ScaleHi);

}
//==============================================================================================
void   UpdateAllTimers() {
  //Update all pulse timers

  //Set the pulse to 0 to clear it out if it was set from the last scan
  TimerOneSecondPulse = 0;
  TimerHalfSecondPulse = 0;
  TimerTenthSecondPulse = 0;
  TimerTenMSPulse = 0;
  TimerOneMSPulse = 0;

  //For each pulse timer, process the pulse
  for (int i = 0; i < PulseTimerLen; i++) {
    PulseTimerUpdate(i);
  }
}
//==============================================================================================
void CheckDayNight() { 
  //Check the civil twilight hours and set if its day or night right now 

  int minuteofday = hour()*60 + minute();
  if (minuteofday >= CivilTwlightStartMin[month()] && minuteofday <= CivilTwlightEndMin[month()]) {
    DayNight = false; //daytime
  } else {
    DayNight = true; //nighttime
  }

}
//==============================================================================================

void PulseTimerUpdate(int index) {

  //when timer is enabled on a one shot, record the timestamp
  if (PulseTimerREM[index] == 0) {
    PulseTimerREM[index] = millis();
  }



  if (millis() >=  PulseTimerREM[index] + PulseTimerPRE[index]  ) { //Evaluate if the timer is finished
    PulseTimerREM[index] = PulseTimerREM[index] + PulseTimerPRE[index]; //Set the REM to the REM plus the preset to trigger exactly off of the next second

    if (index == 0) {
      TimerOneMSPulse = 1;
    } else if (index == 1) {
      TimerTenMSPulse = 1;
    } else if (index == 2) {
      TimerTenthSecondPulse = 1;
    } else if (index == 3) {
      TimerHalfSecondPulse = 1;
    } else if (index == 4) {
      TimerOneSecondPulse = 1;
    } else {
      Serial.print(F("Error code 01"));
      Serial.println(index);
    }

  }
}
//==============================================================================================
void UpdateScanCounter() {
  //Update scan time statistics.  This routine should run every scan of the program
  ScanCounter++;
  unsigned long microsCurrent = micros(); // Record the current micros
  unsigned long LastScanTime = microsCurrent - microsREM; //calculate the last scan time
  microsREM = microsCurrent; //Remember for next time

  if (LastScanTime > MaxScanTimeuS) {
    MaxScanTimeuS = LastScanTime;
  }
  if (LastScanTime < MinScanTimeuS) {
    MinScanTimeuS = LastScanTime;
  }

  if (TimerOneSecondPulse == 1) {
    AvgScanTimeuS = float(PulseTimerPRE[TimerOneSecondPulseIndex]) * 1000.0 / float(ScanCounter);


    if (debug==1) {
      //Debug code for the scan time
      Serial.print(F("perf "));
      
      Serial.print(F("Avg:"));
      Serial.print(AvgScanTimeuS/1000,3);
      Serial.print(F("ms "));
      
      Serial.print(F("min="));
      Serial.print(float(MinScanTimeuS)/1000);
      Serial.print(F("ms "));
      
      Serial.print(F("max="));
      Serial.print(float(MaxScanTimeuS)/1000);
      Serial.print(F("ms "));
      
      Serial.print(F("scans per sec:"));
      Serial.print(ScanCounter);
      
      Serial.println("");
    }
    ScanCounter = 0;
    MaxScanTimeuS = 0;
    MinScanTimeuS = 4294967295;
  }
}
//==============================================================================================
float steinhart(float value) {
      // convert the value to resistance
      value = 1023.0 / value - 1.0;
      value = SERIESRESISTOR * value;

      value = value / THERMISTORNOMINAL;     // (R/Ro)
      value = log(value);                  // ln(R/Ro)
      value /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
      value += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
      value = 1.0 / value;                 // Invert
      value -= 273.15;                         // convert to C
      return value;
}
//==============================================================================================



/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println(F("Transmit NTP Request"));
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println(F("Receive NTP Response"));
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println(F("No NTP Response :-("));
  return 0; // return 0 if unable to get the time
}

//===============================================
// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
//===============================================
void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(month());
  Serial.print("/");
  Serial.print(day());
  Serial.print("/");
  Serial.print(year()); 
}

void printDigits(int digits){
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

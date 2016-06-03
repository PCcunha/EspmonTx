/*
 * Emonitor_Core.cpp
 *
 * Created: 06/03/2012 10:30:58
 *  Author: pcunha
 */

#include "Arduino.h"
#ifndef F_CPU
#define F_CPU 20000000L
#endif
#include <avr/wdt.h>

#include <stdio.h>
#include <avr/eeprom.h>
#include <ELClient.h>
#include <ELClientRest.h>
#include <ELClientMqtt.h>

//For floating point use linker arguments -Wl,-u,vfprintf -lprintf_flt -lm


// Initialize a connection to esp-link using the normal hardware serial port both for
// SLIP and for debug messages.
//com debug:
ELClient esp(&Serial, &Serial);

//ELClient esp(&Serial);
// Initialize a REST client on the connection to esp-link
ELClientRest rest(&esp);
ELClientMqtt mqtt(&esp);

bool mqttconnected;


//------------------------------------------------------------------------------------
//DEFINES
//------------------------------------------------------------------------------------

#define VERSION 4

#define POST_HOST "emoncms.org"
#define POST_URL "/input/post.json?apikey=bfc343f8ae0309a7a2392d6a7ce4a137&json=%s"
#define MQTT_TOPIC "/nodes/emontx/status"

#define CALC_INTERVAL 5 //in seconds - time wich arduino will do power calculations.
#define FREQUENCY 60 // IN HZ
//How many CTS for the main breaker (may not be on the same phase)
#define  PHASE_SENSORS  3
// how many voltage sensors do you have (Open energy monitor dafaults to 1)
#define  VOLTAGE_SENSORS 1
//Phase shift angle between phases (defaults to 120 for 3 phase circuits)
#define  PHASE_SHIFT  = 120
//samples per second are F_CPU / ADC prescaler(128) / 13 clock cycles per adc / channel count
#define SAMPLES_PER_SECOND (F_CPU / 128 / 13 / (PHASE_SENSORS + VOLTAGE_SENSORS))

#define LED_PIN 6

//SAMPLES_PER_SECOND are F_CPU / ADC prescaler(128) / 13 clock cycles per adc / channel count
//the buffer size is samples per second / degrees to go back
//for 3-phase circuits, the degree is 240
//for 2-phase is 90 - i cannot test this...
//for 2-phases of a 3-phase system in theory is 120.

//MAX_SHIFT_DEGREE = 240
#define ADC_BUFFER_SIZE  33 // ((((SAMPLES_PER_SECOND/FREQUENCY) * MAX_SHIFT_DEGREE) / 360)  +1);
#define PHASE1_OFFSET 16
#define PHASE2_OFFSET 32


/*
Input pins
[index] - Arduino AD
Example: for 3-phase with 1 voltage sensors:
[0] Phase1 voltage
[1] Phase1 current
[2] Phase2 current
[3] Phase3 current

Example: for 3-phase with 3 voltage sensors:
[0] Phase1 voltage
[1] Phase1 current
[2] Phase2 voltage
[3] Phase2 current
[4] Phase3 voltage
[5] Phase3 current
*/


/*
Pooling Order:
3-Phase voltages and 3-CT:
Voltage1, Current1, Voltage2, Current2, V3, C3, V1, C1...
1Phase Voltage - 3 Phase CT:
Voltage1 - Current1, Current2, Current3, Voltage1...
*/

unsigned char input_analog_pins[PHASE_SENSORS + VOLTAGE_SENSORS] = {2,3,0,1};

#define TRANSMIT_INTERVAL 2
#define PHASE_CALIBRATION_0 1.7
#define PHASE_CALIBRATION_1 1.7
#define PHASE_CALIBRATION_2 1.7
#define CURRENT_CALIBRATION_0 0.1080
#define CURRENT_CALIBRATION_1 0.1080
#define CURRENT_CALIBRATION_2 0.1080
#define VOLTAGE_CALIBRATION_0 0.4581
#define VOLTAGE_CALIBRATION_1 0.4581
#define VOLTAGE_CALIBRATION_2 0.4581
#define VOLTAGE 127.0

//End of User defineable variables
//---------------------------------------------------
// Do not touch the code below this point, unless you know what you are doing
//---------------------------------------------------

//1 voltage sensor
#if (VOLTAGE_SENSORS == 1)
  #define V0 0
  #if (PHASE_SENSORS > 0)
    #define I0 1
    #if (PHASE_SENSORS > 1)
       #define I1 2
       #if (PHASE_SENSORS > 2)
        #define I2 3
        #endif
    #endif
  #endif
//2 voltage sensors
#elif (VOLTAGE_SENSORS == 2)
  #define V0 0
  #if (PHASE_SENSORS > 0)
    #define I0 1
    #define V1 2
    #if (PHASE_SENSORS > 1)
      #define I1 3
      #if (PHASE_SENSORS > 2)
        #define I2 4
      #endif
    #endif
  #else
    #define V1 1
  #endif
//3 voltage sensors
#elif (VOLTAGE_SENSORS == 3)
  #define V0 0
  #if (PHASE_SENSORS == 1)
    #define I0 1
    #define V1 2
    #define V2 3
  #elif (PHASE_SENSORS == 2)
    #define I0 1
    #define V1 2
    #define I1 3
    #define V2 4
  #elif (PHASE_SENSORS == 3)
    #define I0 1
    #define V1 2
    #define I1 3
    #define V2 4
    #define I2 5
  #else
    #define V1 1
    #define V2 2
  #endif
#else
  //no voltage sensors
  #if (PHASE_SENSORS > 0)
    #define I0 0
    #if (PHASE_SENSORS > 1)
       #define I1 1
       #if (PHASE_SENSORS > 2)
         #define I2 2
       #endif
    #endif
  #endif
#endif


//Temp variables
long adcval;
volatile unsigned char AdcIndex = 1;   //keeps track of sampled channel
volatile unsigned int  SampleCounter, crossCounter, LastSampleCounter;
volatile boolean Cycle_Full; //stores wherever the cycle has been computed
boolean lastVCross, checkVCross;    //Used to measure number of times threshold is crossed.
unsigned char tempidx;

struct        CurrentSampleS {
                float New, Previous;
                float Filtered,PreviousFiltered, Calibrated, Sum, InstPower;
              }  CurrentSample[3]; //for now i will maintain 3 here hardcoded because i dont want to put 300 ifs on the code yet.

struct        VoltageSampleS {
                float New, Previous;
                float Filtered,PreviousFiltered, PhaseShifted, Calibrated, Sum;
              } VoltageSample[3]; //Yes phases here, because we are going to create the voltage data based on what we have //for now i will maintain 3 here hardcoded because i dont want to put 300 ifs on the code yet.

struct        LastCurrentS {
                float Sum, InstPower;
              } LastCurrent[3];//for now i will maintain 3 here hardcoded because i dont want to put 300 ifs on the code yet.
struct        LastVoltageS {
                float  Sum;
              } LastVoltage[VOLTAGE_SENSORS];


float AccVrms[3];
float AccIrms[3];
float AccRealPower[3] ;
float AccAparentPower[3];
float AccPowerFactor[3];

float totalP;
float cummKw;
float cummP;  //cummulative power

//TODO fix if the 3 sensors are on the same phase, put some code here
#if VOLTAGE_SENSORS == 1 && PHASE_SENSORS > 1
  #define ADC_BUFFER 1
  //declare voltage buffer
  float ADC_V_BUFFER[ADC_BUFFER_SIZE+2];
  unsigned char adc_buffer_index = (ADC_BUFFER_SIZE-1);
#endif

boolean temp = 0;
float tempdbl = 0.0;
unsigned char tmpchar;
volatile unsigned long LastInterruptTime;

unsigned char tcount;
unsigned int counts;


//------------------------------------------------------------------------------------
//ADC INTERRUPT SERVICE ROUTINE
//------------------------------------------------------------------------------------
ISR(ADC_vect){		//ADC interrupt
//unsigned long t1;
unsigned char IndexAtual = 0;
//t1 = micros();

unsigned char nextindex = AdcIndex;
if (++nextindex >= (PHASE_SENSORS + VOLTAGE_SENSORS))  nextindex=0;
ADMUX=(input_analog_pins[nextindex]);// Select next ADC input

//
if (AdcIndex > 0)
	IndexAtual = AdcIndex - 1;
else
	IndexAtual = PHASE_SENSORS + VOLTAGE_SENSORS -1;
  //IndexAtual = (PHASE_SENSORS + VOLTAGE_SENSORS - 1); // adc is always 1 nextindex negative


//Voltage1 - First voltage sensor (Open Energy Monitor only have this voltage sensor)
#ifdef V0
	if (IndexAtual == V0){ //Very strange behavior when using ifdef here with 0 value
		// Store Last Values for future use
		VoltageSample[0].Previous = VoltageSample[0].New;        //Put last sample on its place
		VoltageSample[0].PreviousFiltered = VoltageSample[0].Filtered;        //Put last sample on its place
		// Read in raw voltage and current samples
		VoltageSample[0].New=ADC;      //Read in raw voltage signal from index sample
		// Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
		VoltageSample[0].Filtered = 0.996 * ( VoltageSample[0].PreviousFiltered + VoltageSample[0].New - VoltageSample[0].Previous );
		// Root-mean-square method Index Channel
		// -> sum the square of voltage values
		VoltageSample[0].Sum  += VoltageSample[0].Filtered * VoltageSample[0].Filtered;
		// Phase calibration
		VoltageSample[0].PhaseShifted = VoltageSample[0].PreviousFiltered + PHASE_CALIBRATION_0 * (VoltageSample[0].Filtered - VoltageSample[0].PreviousFiltered);
		//If using ADC BUFFERING
		//unsigned int ADC_V_BUFFER[ADC_BUFFER_SIZE];
		//volatile unsigned char adc_buffer_index;
		#ifdef ADC_BUFFER
			if (--adc_buffer_index == 255)   adc_buffer_index=(ADC_BUFFER_SIZE-1);
			ADC_V_BUFFER[adc_buffer_index] = VoltageSample[0].PhaseShifted;
		#endif
	}
#endif

//Current1
#if ((IO == 0) || (IO == 1))//Very strange behavior when using ifdef here with 0 value
	if (IndexAtual == I0){ //First CT
		//A) Store Last Values for future use
		CurrentSample[0].Previous = CurrentSample[0].New;        //Put last sample on its place
		CurrentSample[0].PreviousFiltered = CurrentSample[0].Filtered;        //Put last sample on its place
		// Read in raw voltage and current samples
		CurrentSample[0].New=ADC;      //Read in raw voltage signal from index sample
		// Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
		CurrentSample[0].Filtered = 0.996 * ( CurrentSample[0].PreviousFiltered + CurrentSample[0].New - CurrentSample[0].Previous );
		// Root-mean-square method Index Channel
		// -> sum the square of voltage values
		CurrentSample[0].Sum  += CurrentSample[0].Filtered * CurrentSample[0].Filtered;
		// Instantaneous power calc now that we have I and V
		#if VOLTAGE_SENSORS > 0
			CurrentSample[0].InstPower += VoltageSample[0].PhaseShifted * CurrentSample[0].Filtered; //Instantaneous Power //mudar  pphaseshift
		#else
			CurrentSample[0].InstPower = CurrentSample[0].Filtered;
		#endif
	}
#endif

#ifdef V1
if (IndexAtual == V1){
    //A) Store Last Values for future use
    VoltageSample[1].Previous = VoltageSample[1].New;        //Put last sample on its place
    VoltageSample[1].PreviousFiltered = VoltageSample[1].Filtered;        //Put last sample on its place
    // Read in raw voltage and current samples
    VoltageSample[1].New=ADC;      //Read in raw voltage signal from index sample
    // Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
    VoltageSample[1].Filtered = 0.996 * ( VoltageSample[1].PreviousFiltered + VoltageSample[1].New - VoltageSample[1].Previous );
    // Root-mean-square method Index Channel
    // -> sum the square of voltage values
    VoltageSample[1].Sum  += VoltageSample[1].Filtered * VoltageSample[1].Filtered;
    // Phase calibration
    VoltageSample[1].PhaseShifted = VoltageSample[1].PreviousFiltered + PHASE_CALIBRATION_1 * (VoltageSample[1].Filtered - VoltageSample[1].PreviousFiltered);
  }
#endif

#ifdef I1
if (IndexAtual == I1){ //First CT
	//A) Store Last Values for future use
	CurrentSample[1].Previous = CurrentSample[1].New;        //Put last sample on its place
	CurrentSample[1].PreviousFiltered = CurrentSample[1].Filtered;        //Put last sample on its place
	// Read in raw voltage and current samples
	CurrentSample[1].New=ADC;      //Read in raw voltage signal from index sample
	// Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
	CurrentSample[1].Filtered = 0.996 * ( CurrentSample[1].PreviousFiltered + CurrentSample[1].New - CurrentSample[1].Previous );
	// Root-mean-square method Index Channel
	// -> sum the square of voltage values
	CurrentSample[1].Sum  += CurrentSample[1].Filtered * CurrentSample[1].Filtered;
	// Instantaneous power calc now that we have I and V
	#if VOLTAGE_SENSORS > 1
		CurrentSample[1].InstPower += VoltageSample[1].PhaseShifted * CurrentSample[1].Filtered; //Instantaneous Power
	#else
		#ifdef ADC_BUFFER //if there is adc buffering
			 tmpchar = (adc_buffer_index + PHASE1_OFFSET) % ADC_BUFFER_SIZE;
			 tempdbl = ADC_V_BUFFER[tmpchar];// (adc_buffer_index + PHASE1_OFFSET)% ADC_BUFFER_SIZE);
			//aqui
			CurrentSample[1].InstPower += tempdbl * CurrentSample[1].Filtered;
		#else //if adc_buffer is inactve
			CurrentSample[1].InstPower = CurrentSample[1].Filtered;
		#endif
	#endif
	}
#endif

#ifdef V2
  if (IndexAtual == V2){
    //A) Store Last Values for future use
    VoltageSample[2].Previous = VoltageSample[2].New;        //Put last sample on its place
    VoltageSample[2].PreviousFiltered = VoltageSample[2].Filtered;        //Put last sample on its place
    // Read in raw voltage and current samples
    VoltageSample[2].New=ADC;      //Read in raw voltage signal from index sample
    // Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
    VoltageSample[2].Filtered = 0.996 * ( VoltageSample[2].PreviousFiltered + VoltageSample[2].New - VoltageSample[2].Previous );
    // Root-mean-square method Index Channel
    // -> sum the square of voltage values
    VoltageSample[2].Sum  += VoltageSample[2].Filtered * VoltageSample[2].Filtered;
    // Phase calibration
    VoltageSample[2].PhaseShifted = VoltageSample[2].PreviousFiltered + PHASE_CALIBRATION_2 * (VoltageSample[2].Filtered - VoltageSample[2].PreviousFiltered);
  }
#endif


#ifdef I2
	if (IndexAtual == I2){ //First CT
		//A) Store Last Values for future use
		CurrentSample[2].Previous = CurrentSample[2].New;        //Put last sample on its place
		CurrentSample[2].PreviousFiltered = CurrentSample[2].Filtered;        //Put last sample on its place
		// Read in raw voltage and current samples
		CurrentSample[2].New=ADC;      //Read in raw voltage signal from index sample
		// Apply digital high pass filters to remove 2.5V DC offset (centered on 0V).
		CurrentSample[2].Filtered = 0.996 * ( CurrentSample[2].PreviousFiltered + CurrentSample[2].New - CurrentSample[2].Previous );
		// Root-mean-square method Index Channel
		// -> sum the square of voltage values
		CurrentSample[2].Sum  += CurrentSample[2].Filtered * CurrentSample[2].Filtered;
		// Instantaneous power calc now that we have I and V
		#if VOLTAGE_SENSORS > 2
			CurrentSample[2].InstPower += VoltageSample[2].PhaseShifted * CurrentSample[2].Filtered; //Instantaneous Power
		#else
			#ifdef ADC_BUFFER //if there is adc buffering
				tmpchar = (adc_buffer_index + PHASE2_OFFSET) % ADC_BUFFER_SIZE;
				tempdbl = ADC_V_BUFFER[tmpchar];// (adc_buffer_index + PHASE2_OFFSET)% ADC_BUFFER_SIZE);
				CurrentSample[2].InstPower += tempdbl * CurrentSample[2].Filtered;
			#else //if adc_buffer is inactve
				CurrentSample[2].InstPower = CurrentSample[2].Filtered;
			#endif
		#endif
	}
#endif

if (AdcIndex == 0) SampleCounter++;

  //4807 for 2 channels
  //1602 for 6 channels
   //1  sec
  if (SampleCounter > (SAMPLES_PER_SECOND * CALC_INTERVAL)){
   LastSampleCounter = SampleCounter;

   #if VOLTAGE_SENSORS > 0
     LastVoltage[0].Sum = VoltageSample[0].Sum;
     VoltageSample[0].Sum = 0;
     #if VOLTAGE_SENSORS > 1
       LastVoltage[1].Sum = VoltageSample[1].Sum;
       VoltageSample[1].Sum = 0;
       #if VOLTAGE_SENSORS > 2
         LastVoltage[2].Sum = VoltageSample[2].Sum;
         VoltageSample[2].Sum = 0;
       #endif
     #endif
   #endif

   #if PHASE_SENSORS > 0
      LastCurrent[0].Sum = CurrentSample[0].Sum;
      CurrentSample[0].Sum = 0;
      LastCurrent[0].InstPower = CurrentSample[0].InstPower;
      CurrentSample[0].InstPower = 0;
      #if PHASE_SENSORS > 1
        LastCurrent[1].Sum = CurrentSample[1].Sum;
        CurrentSample[1].Sum = 0;
        LastCurrent[1].InstPower = CurrentSample[1].InstPower;
        CurrentSample[1].InstPower = 0;
        #if PHASE_SENSORS > 2
          LastCurrent[2].Sum = CurrentSample[2].Sum;
          CurrentSample[2].Sum = 0;
          LastCurrent[2].InstPower = CurrentSample[2].InstPower;
          CurrentSample[2].InstPower = 0;
         #endif
      #endif
   #endif

    SampleCounter = 0;
    Cycle_Full = true;
  }

// Maintain multiplexing of input channels
if (++AdcIndex >= (PHASE_SENSORS + VOLTAGE_SENSORS))  AdcIndex=0;

//LastInterruptTime = micros() - t1;
}


// Callback when MQTT is connected
void mqttConnected(void* response) {
  Serial.println(F("MQTT connected!"));
  mqttconnected = true;
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  Serial.println(F("MQTT disconnected"));
  mqttconnected = false;
}

void mqttPublished(void* response) {
  Serial.println(F("MQTT published"));
}


  void setup()
  {
	#ifdef SERIAL
      Serial.begin(57600);     //begin Serial comm
      Serial.println();
      Serial.print(F("emonTX v"));
      Serial.print(VERSION);
      Serial.println (F("UP and Running"));
    #endif

      // Check source of reset
    if (MCUSR & 1)               // Power-on Reset
    {
      // put POR handler here, if required
    	Serial.println(F("POR reset"));
    }
    else if (MCUSR & 2)          // External Reset
    {
      // put external reset handler here, if required
    	Serial.println(F("EXT reset"));
    }
    else if (MCUSR & 4)          // Brown-Out Reset
    {
      // put BOR handler here, if required
    	Serial.println(F("BOR reset"));
    }
    else                          // Watchdog Reset
    {
      // put watchdog reset handler here, if required
    	Serial.println("WTC reset");
    };

    pinMode(6,OUTPUT);

    digitalWrite(6,HIGH);

    Serial.println(F("Configuração Atual:"));
	Serial.print(F("Transmit Interval:")); Serial.println(TRANSMIT_INTERVAL);
	Serial.print(F("P1_cal:")); Serial.println(PHASE_CALIBRATION_0,4);
	Serial.print(F("P2_cal:")); Serial.println(PHASE_CALIBRATION_1,4);
	Serial.print(F("P3_cal:")); Serial.println(PHASE_CALIBRATION_2,4);
	Serial.print(F("C1_cal:")); Serial.println(CURRENT_CALIBRATION_0,4);
	Serial.print(F("C2_cal:")); Serial.println(CURRENT_CALIBRATION_1,4);
	Serial.print(F("C3_cal:")); Serial.println(CURRENT_CALIBRATION_2,4);
	Serial.print(F("V1_cal:")); Serial.println(VOLTAGE_CALIBRATION_0,4);
	Serial.print(F("V2_cal:")); Serial.println(VOLTAGE_CALIBRATION_1,4);
	Serial.print(F("V3_cal:")); Serial.println(VOLTAGE_CALIBRATION_2,4);
	Serial.print(F("Voltage:")); Serial.println(VOLTAGE,3);

   bool ok;
   do {
     ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
     if (!ok) Serial.println(F("EL-Client sync failed!"));
   } while(!ok);
   Serial.println(F("EL-Client synced!"));
   Serial.println(F("Begin EL-REST Interface"));

   /*
    esp.GetWifiStatus();
    ELClientPacket *packet;
    if ((packet=esp.WaitReturn()) != NULL) {
      Serial.print("Wifi status: ");
      Serial.println(packet->value);
    }
    */

    // Set up the REST client
    int err = rest.begin(POST_HOST);
    if (err != 0) {
      Serial.print(F("REST begin failed: "));
      Serial.println(err);
      while(1) ;
    }

    Serial.println(F("EL-REST ready"));
    Serial.println(F("Begin MQTT Interface"));
    // Set-up callbacks for events and initialize with es-link.
      mqtt.connectedCb.attach(mqttConnected);
      mqtt.disconnectedCb.attach(mqttDisconnected);
      mqtt.publishedCb.attach(mqttPublished);
      mqtt.setup();

      //Serial.println("ARDUINO: setup mqtt lwt");
      //mqtt.lwt("/lwt", "offline", 0, 0); //or mqtt.lwt("/lwt", "offline");

      Serial.println(F("EL-MQTT ready"));

      digitalWrite(6,LOW);

      wdt_enable(WDTO_8S);

    // ADC initialization:
    // Enable ADC interrupt and use external voltage reference.
    // Set multiplexer input to first channel and do not set
    // ADC to High Speed Mode. Set Auto Trigger Source to Free Running Mode.
    //
    //  fCLK | ADPS | Prescaler | ADCCLK | Sampl.Rate
    // ------+------+-----------+--------+-----------
    // 4.000 |  111 |       128 | 125000 |       9615

    //ADC INIT
    ADMUX = (input_analog_pins[0] & 0x07); // Set ADC reference to external VFREF and first defined port
    ADCSRA |= (1 << ADEN);  // Enable ADC
    ADCSRA |= (1 << ADATE); // Enable auto-triggering
    ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt

    sei();		     // Enable Global Interrupts

    //put code here to start adc after zero crossing. *TODO*  but not a problem, because we do not pick up first sampled data...

    //Start ADC
    //Serial.println("Open Energy Monitor Hack by Pcunha");
    //Serial.println("Starting ADC in next cycle");
    ADCSRA=0xEF;   // Enable ADC, start, auto trigger, int enable, presc = 128

  }


  void loop()
  {
	  wdt_reset();
	  esp.Process();

      float Vrms[3];
      float Irms[3];
      float realPower[3] ;
      float apparentPower[3];
      float powerFactor[3];

    //skip first cycle to stabilize readings
    if (tcount == 0 && Cycle_Full == true){
      tcount = 1;
      Cycle_Full = false;
    }

    //-------------------------------------------
    if (Cycle_Full == true)
        {Cycle_Full = false;
          //Calculation of the root of the mean of the voltage and current squared (rms)
          //Calibration coeficients applied.
       #if VOLTAGE_SENSORS > 0
         Vrms[0] = VOLTAGE_CALIBRATION_0 * sqrt(LastVoltage[0].Sum / LastSampleCounter);
         #ifdef ADC_BUFFER
           #if PHASE_SENSORS > 1
            Vrms[1] = Vrms[0];
             #if PHASE_SENSORS > 2
              Vrms[2] = Vrms[0];
             #endif
           #endif
         #else
           #if VOLTAGE_SENSORS > 1
             Vrms[1] = VOLTAGE_CALIBRATION_1 * sqrt(LastVoltage[1].Sum / LastSampleCounter);
             #if VOLTAGE_SENSORS > 2
               Vrms[2] = VOLTAGE_CALIBRATION_2 * sqrt(LastVoltage[2].Sum / LastSampleCounter);
             #endif
           #endif
         #endif
       #else
         Vrms[0] = VOLTAGE;
         Vrms[1] = VOLTAGE;
         Vrms[2] = VOLTAGE;
       #endif


       #if PHASE_SENSORS > 0
         //colocar if para 1 ct.
         Irms[0] = CURRENT_CALIBRATION_0 * sqrt(LastCurrent[0].Sum / LastSampleCounter);
           #if VOLTAGE_SENSORS > 0
             realPower[0] = VOLTAGE_CALIBRATION_0 * CURRENT_CALIBRATION_0 * (LastCurrent[0].InstPower / LastSampleCounter);
           #else
             realPower[0] = Vrms[0] * Irms[0];
           #endif
         apparentPower[0] = Vrms[0] * Irms[0];
         powerFactor[0] = realPower[0] / apparentPower[0];
     //--------------------------
         #if PHASE_SENSORS > 1
           //colocar if para 1 ct.
           Irms[1] = CURRENT_CALIBRATION_1 * sqrt(LastCurrent[1].Sum / LastSampleCounter);
           #if VOLTAGE_SENSORS > 1
             realPower[1] = VOLTAGE_CALIBRATION_1 * CURRENT_CALIBRATION_1 * (LastCurrent[1].InstPower / LastSampleCounter);
             apparentPower[1] = Vrms[1] * Irms[1];
           #else
             #ifdef ADC_BUFFER
               realPower[1] = VOLTAGE_CALIBRATION_0 * CURRENT_CALIBRATION_1 * (LastCurrent[1].InstPower / LastSampleCounter);
               apparentPower[1] = Vrms[0] * Irms[1];
             #else
               realPower[1] = Vrms[1] * Irms[1];
               apparentPower[1] = realPower[1];
             #endif
           #endif
           powerFactor[1] = realPower[1] / apparentPower[1];
            //--------------------------
           #if PHASE_SENSORS > 2
             //colocar if para 1 ct.
             Irms[2] = CURRENT_CALIBRATION_2 * sqrt(LastCurrent[2].Sum / LastSampleCounter);
             #if VOLTAGE_SENSORS > 2
               realPower[2] = VOLTAGE_CALIBRATION_2 * CURRENT_CALIBRATION_2 * (LastCurrent[2].InstPower / LastSampleCounter);
               apparentPower[2] = Vrms[2] * Irms[2];
             #else
               #ifdef ADC_BUFFER
                 realPower[2] = VOLTAGE_CALIBRATION_0 * CURRENT_CALIBRATION_2 * (LastCurrent[2].InstPower / LastSampleCounter);
                 apparentPower[2] = Vrms[0] * Irms[2];
               #else
                 realPower[2] = Vrms[2] * Irms[2];
                 apparentPower[2] = realPower[2];
               #endif
             #endif
             powerFactor[2] = realPower[2] / apparentPower[2];
           #endif
         #endif
       #endif



    //calc transmission interval data

      #if VOLTAGE_SENSORS > 0
        AccVrms[0] = AccVrms[0] + (Vrms[0] / TRANSMIT_INTERVAL) ;
        #ifdef ADC_BUFFER
        AccVrms[1] = AccVrms[0];
        AccVrms[2] = AccVrms[0];
        #endif
        #if VOLTAGE_SENSORS > 1
          AccVrms[1] = AccVrms[1] + (Vrms[1] / config.transmitinterval) ;
          #if VOLTAGE_SENSORS > 2
            AccVrms[2] = AccVrms[2] + (Vrms[2] / TRANSMIT_INTERVALL) ;
          #endif
        #endif
      #else
        AccVrms[0] = Voltage;
        AccVrms[1] = Voltage;
        AccVrms[2] = Voltage;
      #endif

      #if PHASE_SENSORS > 0
        AccIrms[0] = AccIrms[0] + (Irms[0] / TRANSMIT_INTERVAL) ;
        AccRealPower[0] = AccRealPower[0] + (realPower[0] / TRANSMIT_INTERVAL) ;
  //      AccAparentPower[0] = AccAparentPower[0] + (apparentPower[0] / TRANSMIT_INTERVAL) ;
        AccPowerFactor[0] = AccPowerFactor[0] + (powerFactor[0] / TRANSMIT_INTERVAL) ;
        #if PHASE_SENSORS > 1
          AccIrms[1] = AccIrms[1] + (Irms[1] / TRANSMIT_INTERVAL) ;
          AccRealPower[1] = AccRealPower[1] + (realPower[1] / TRANSMIT_INTERVAL) ;
  //        AccAparentPower[1] = AccAparentPower[1] + (apparentPower[1] / TRANSMIT_INTERVAL) ;
          AccPowerFactor[1] = AccPowerFactor[1] + (powerFactor[1] / TRANSMIT_INTERVAL) ;
          #if PHASE_SENSORS > 2
            AccIrms[2] = AccIrms[2] + (Irms[2] / TRANSMIT_INTERVAL) ;
            AccRealPower[2] = AccRealPower[2] + (realPower[2] / TRANSMIT_INTERVAL) ;
  //          AccAparentPower[2] = AccAparentPower[2] + (apparentPower[2] / TRANSMIT_INTERVAL) ;
            AccPowerFactor[2] = AccPowerFactor[2] + (powerFactor[2] / TRANSMIT_INTERVAL) ;
          #endif
        #endif
      #endif
      tcount++;

      totalP = AccRealPower[0]
            #if PHASE_SENSORS > 1
              + AccRealPower[1]
              #if PHASE_SENSORS > 2
                + AccRealPower[2]
              #endif
            #endif
      ;


      float whInc = totalP *((CALC_INTERVAL * TRANSMIT_INTERVAL)/3600.0);
      cummKw += whInc / 1000;

      //transmit
      if (tcount > TRANSMIT_INTERVAL) {
        tcount = 1;
        counts++;


        digitalWrite(6,HIGH);

  	  char buff[256];

  	  sprintf(buff, "%s{counts:%u,power:%.3f,kw:%.3f,vr0:%.3f,ir0:%.3f,ir1:%.3f,ir2:%.3f,rp0:%.3f,rp1:%.3f,rp2:%.3f,pf0:%.2f,pf1:%.2f,pf2:%.2f}",
  			POST_URL,
  			counts,
  			(double)totalP,
  			(double)cummKw,
  			(double)AccVrms[0],
  			(double)AccIrms[0],
  			(double)AccIrms[1],
  			(double)AccIrms[2],
  			(double)AccRealPower[0],
  			(double)AccRealPower[1],
  			(double)AccRealPower[2],
  			(double)AccPowerFactor[0],
  			(double)AccPowerFactor[1],
  			(double)AccPowerFactor[2]
  			);

  	  Serial.println(buff);

  	  rest.get((const char*)buff);
  	  Serial.println("ARDUINO: send get");
  	  //if(rest.getResponse(response, 266) == HTTP_STATUS_OK){
  	  // 	  Serial.println(F("Post OK"));
  	  //}


  	  if (mqttconnected){
  	  	  sprintf(buff, "{counts:%u,power:%.3f,kw:%.3f,vr0:%.3f,ir0:%.3f,ir1:%.3f,ir2:%.3f,rp0:%.3f,rp1:%.3f,rp2:%.3f,pf0:%.2f,pf1:%.2f,pf2:%.2f}",
			counts,
			(double)totalP,
			(double)cummKw,
			(double)AccVrms[0],
			(double)AccIrms[0],
			(double)AccIrms[1],
			(double)AccIrms[2],
			(double)AccRealPower[0],
			(double)AccRealPower[1],
			(double)AccRealPower[2],
			(double)AccPowerFactor[0],
			(double)AccPowerFactor[1],
			(double)AccPowerFactor[2]
  	  		);

  		  mqtt.publish(MQTT_TOPIC,buff);
  	  }


      digitalWrite(6,LOW);

       //erase data
       #if VOLTAGE_SENSORS > 0
        AccVrms[0] = 0 ;
        #if VOLTAGE_SENSORS > 1
          AccVrms[1] = 0 ;
          #if VOLTAGE_SENSORS > 2
            AccVrms[2] = 0 ;
          #endif
        #endif
      #else
        AccVrms[0] = 0;
        AccVrms[1] = 0;
        AccVrms[2] = 0;
      #endif

      #if PHASE_SENSORS > 0
        AccIrms[0] = 0 ;
        AccRealPower[0] = 0 ;
        AccAparentPower[0] = 0 ;
        AccPowerFactor[0] = 0 ;
        #if PHASE_SENSORS > 1
          AccIrms[1] = 0 ;
          AccRealPower[1] = 0 ;
          AccAparentPower[1] = 0 ;
          AccPowerFactor[1] = 0 ;
          #if PHASE_SENSORS > 2
            AccIrms[2] = 0 ;
            AccRealPower[2] = 0 ;
            AccAparentPower[2] = 0 ;
            AccPowerFactor[2] = 0 ;
          #endif
        #endif
      #endif
      }//transmit
    }//cycle full


}


int main(void) {

  init();
  setup();

  while(true) {
    loop();
  }
}

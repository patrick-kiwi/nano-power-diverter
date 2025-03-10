#include <PinChangeInterrupt.h>
#include <Arduino.h>
#include <TimerOne.h>
#include "nonBlockingDelay.h"

// Configuration
#define ADC_TIMER_PERIOD 130  // uS (sets sampling rate)
//#define WORKLOAD_CHECK
#define DISPLAY_POWER_CONSUMPTION
//#define DISPLAY_LOWEST_SAMPLES_PER_CYCLE

// Constants
#define SECONDS_PER_MINUTE 60
#define MINUTES_PER_HOUR 60
#define JOULES_PER_WATT_HOUR 3600
#define CYCLES_PER_SECOND 50
#define WORKING_RANGE_IN_JOULES 360  // 0.1 Wh
#define REQUIRED_EXPORT_IN_WATTS 0   // negative = PV generator
#define ANTI_CREEP_LIMIT 5           // in Joules per mains cycle
#define PERSISTENCE_FOR_POLARITY_CHANGE 1  // sample sets
#define CONTINUITY_CHECK_MAXCOUNT 250      // mains cycles

// Function declarations
inline void writeStates(void) __attribute__((always_inline));
void timerIsr(void);
void allGeneralProcessing(void);

// Enums
enum polarities { NEGATIVE, POSITIVE };
enum ledStates { LED_OFF, LED_ON };
enum loadStates { LOAD_ON, LOAD_OFF };
static enum loadStates nextStateOfLoad = LOAD_OFF;

// Pin allocations
const byte voltageSensor = 7;
const byte currentSensor_grid = 3;
const byte outputForBlueLED = 11;  // Active low
const byte outputForRedLED = 10;   // Active low
const byte outputForTrigger = 9;   // Active low
const byte outputForExternal = 8;  // Active High, output to ESP32 via a voltage divider
const byte EspInput = 3;           // Input from Esp32
const byte overrideButton = 2;     // Input from manual boost button

// Global variables
const byte delayBeforeSerialStarts = 1;
const byte startUpPeriod = 3;
const int DCoffset_I = 512;
long antiCreepLimit_inIEUperMainsCycle;
bool beyondStartUpPhase = false;
long energyInBucket_long;
long capacityOfEnergyBucket_long;
long singleEnergyThreshold_long;
long DCoffset_V_long;
long DCoffset_V_min;
long DCoffset_V_max;
long divertedEnergyRecent_IEU = 0;
unsigned int divertedEnergyTotal_Wh = 0;
long IEU_per_Wh;
unsigned long absenceOfDivertedEnergyCount = 0;
long mainsCyclesPerHour;
const unsigned long manualOverrideInterval = 3600000L;  // 1 hour

#ifdef DISPLAY_POWER_CONSUMPTION
long energyThisSecond_grid;
int perSecondCounter;
long IEU_per_Joule_grid;
#endif

// Sampling variables
volatile boolean dataReady = false;
volatile int sampleI_grid;
volatile int sampleV;

// Hardware interrupt variables
volatile int IsrEvent = 0;
void executeOverride() { IsrEvent = 1; }
void executeEsp() { IsrEvent = 2; }

// Polarity detection
enum polarities polarityOfMostRecentVsample;
enum polarities polarityConfirmed;
enum polarities polarityConfirmedOfLastSampleV;

// Continuity checking
int sampleCount_forContinuityChecker;
int sampleSetsDuringThisMainsCycle;
int lowestNoOfSampleSetsPerMainsCycle;

// Calibration - powerCal is the reciprocal of power conversion rate
const float powerCal_grid = 0.192;
long requiredExportPerMainsCycle_inIEU;

void setup() {
  pinMode(overrideButton, INPUT);
  pinMode(outputForTrigger, OUTPUT);
  pinMode(outputForBlueLED, OUTPUT);
  pinMode(outputForRedLED, OUTPUT);
  pinMode(outputForExternal, OUTPUT);
  digitalWrite(outputForTrigger, HIGH);
  digitalWrite(outputForBlueLED, HIGH);
  digitalWrite(outputForRedLED, HIGH);
  digitalWrite(outputForExternal, LOW);
  digitalWrite(outputForTrigger, LOAD_OFF);
  
  delay(delayBeforeSerialStarts * 1000);
  Serial.begin(9600);
  Serial.println();
  Serial.println("-------------------------------------");
  Serial.println("Sketch ID:      Mk2_fasterControl_3.ino");
  Serial.println();

  capacityOfEnergyBucket_long = (long)WORKING_RANGE_IN_JOULES * CYCLES_PER_SECOND * (1 / powerCal_grid);
  energyInBucket_long = 0;
  singleEnergyThreshold_long = capacityOfEnergyBucket_long * 0.5;
  antiCreepLimit_inIEUperMainsCycle = (float)ANTI_CREEP_LIMIT * (1 / powerCal_grid);
  mainsCyclesPerHour = (long)CYCLES_PER_SECOND * SECONDS_PER_MINUTE * MINUTES_PER_HOUR;
  requiredExportPerMainsCycle_inIEU = (long)REQUIRED_EXPORT_IN_WATTS * (1 / powerCal_grid);

#ifdef DISPLAY_POWER_CONSUMPTION
  IEU_per_Joule_grid = (long)CYCLES_PER_SECOND * (1 / powerCal_grid);
#endif

  // Define operating limits for the LP filter
  DCoffset_V_long = 512L * 256;
  DCoffset_V_min = (long)(512L - 100) * 256;
  DCoffset_V_max = (long)(512L + 100) * 256;

  Serial.print("ADC mode:       ");
  Serial.print(ADC_TIMER_PERIOD);
  Serial.println(" uS fixed timer");

  // Set up the ADC
  ADCSRA = (1 << ADPS0) + (1 << ADPS1) + (1 << ADPS2);  // Set clock to system clock / 128
  ADCSRA |= (1 << ADEN);                                // Enable ADC

  Timer1.initialize(ADC_TIMER_PERIOD);
  Timer1.attachInterrupt(timerIsr);
  attachPCINT(digitalPinToPCINT(overrideButton), executeOverride, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EspInput), executeEsp, RISING);

  Serial.print("powerCal_grid =      ");
  Serial.println(powerCal_grid, 4);
  Serial.print("Anti-creep limit (Joules / mains cycle) = ");
  Serial.println(ANTI_CREEP_LIMIT);
  Serial.print("Export rate (Watts) = ");
  Serial.println(REQUIRED_EXPORT_IN_WATTS);
  Serial.print("zero-crossing persistence (sample sets) = ");
  Serial.println(PERSISTENCE_FOR_POLARITY_CHANGE);
  Serial.print("continuity sampling display rate (mains cycles) = ");
  Serial.println(CONTINUITY_CHECK_MAXCOUNT);
  Serial.print("  capacityOfEnergyBucket_long = ");
  Serial.println(capacityOfEnergyBucket_long);
  Serial.print("  singleEnergyThreshold_long   = ");
  Serial.println(singleEnergyThreshold_long);
  Serial.print(">>free RAM = ");
  Serial.println(freeRam());

  Serial.println("----");

#ifdef WORKLOAD_CHECK
  Serial.println("WELCOME TO WORKLOAD_CHECK ");
  Serial.println("  The displayed value is the amount of spare time, per set of V & I samples, ");
  Serial.println("that is available for doing additional processing.");
  Serial.println();
#endif
}

// ADC interrupt service routine - executes when ADC timer expires
void timerIsr(void) {
  static unsigned char sample_index = 0;
  static int sampleI_grid_raw;

  switch (sample_index) {
    case 0:
      sampleV = ADC;
      ADMUX = 0x40 + currentSensor_grid;
      ADCSRA |= (1 << ADSC);
      sample_index++;
      sampleI_grid = sampleI_grid_raw;
      dataReady = true;
      break;
    case 1:
      sampleI_grid_raw = ADC;
      ADMUX = 0x40 + voltageSensor;
      ADCSRA |= (1 << ADSC);
      sample_index = 0;
      break;
    default:
      sample_index = 0;  // prevent lockup
  }
}

inline void writeStates(void) {
  // Fast port manipulation replaces multiple digitalWrite calls
  PORTB = ((!nextStateOfLoad & 1) << 3) |  // pin11, blue LED
          ((nextStateOfLoad & 1) << 2) |   // pin10, red LED
          ((nextStateOfLoad & 1) << 1) |   // pin9, trigger
          (!nextStateOfLoad & 1);          // pin8, esp32
}

void loop() {
#ifdef WORKLOAD_CHECK
  static int del = 0;
  static int res = 0;
  static byte count = 0;
  static byte displayFlag = 0;
#endif

  if (dataReady) {
    dataReady = false;
    
    if (IsrEvent) {
      if (IsrEvent == 1) {
        IsrEvent = 0;
        nblock_delay manualOverrideDelay(manualOverrideInterval);
        nextStateOfLoad = LOAD_ON;
        writeStates();
        while (manualOverrideDelay.trigger()) {}
      } else if (IsrEvent == 2) {
        IsrEvent = 0;
        nextStateOfLoad = LOAD_ON;
        writeStates();
        while (digitalRead(EspInput) == HIGH) {}
      } else {
        IsrEvent = 0;
      }
    } else {
      allGeneralProcessing();
    }

#ifdef WORKLOAD_CHECK
    delayMicroseconds(del);
    if (dataReady) {
      res = del;
      del = 1;
      count = 0;
      displayFlag = 0;
    } else {
      count++;
      if (count > 50) {
        count = 0;
        del++;
      }
    }
#endif
  }

#ifdef WORKLOAD_CHECK
  switch (displayFlag) {
    case 0:
      displayFlag++;
      break;
    case 1:
      Serial.print(res);
      displayFlag++;
      break;
    case 2:
      Serial.println("uS");
      displayFlag++;
      break;
    default:;
  }
#endif
}

void allGeneralProcessing() {
  static long sumP_grid;
  static long cumVdeltasThisCycle_long;
  static byte timerForDisplayUpdate = 0;
  static int sampleSetsDuringNegativeHalfOfMainsCycle;
  static long energyInBucket_prediction;
  
  // LPF to improve processing of data samples from CT1
  static long lpf_long;
  const float lpf_gain = 8;
  const float alpha = 0.002;
  
  // Remove DC offset from voltage sample
  long sampleVminusDC_long = ((long)sampleV << 8) - DCoffset_V_long;

  // Determine polarity of latest voltage sample
  polarityOfMostRecentVsample = (sampleVminusDC_long > 0) ? POSITIVE : NEGATIVE;
  confirmPolarity();

  if (polarityConfirmed == POSITIVE) {
    if (polarityConfirmedOfLastSampleV != POSITIVE) {
      // Start of new +ve half cycle
      if (beyondStartUpPhase) {
        // Check performance of ISR structure
        if (sampleSetsDuringThisMainsCycle < lowestNoOfSampleSetsPerMainsCycle) {
          lowestNoOfSampleSetsPerMainsCycle = sampleSetsDuringThisMainsCycle;
        }

        // Calculate real power and energy
        long realPower_grid = sumP_grid / sampleSetsDuringThisMainsCycle;
        realPower_grid -= requiredExportPerMainsCycle_inIEU;
        long realEnergy_grid = realPower_grid;

#ifdef DISPLAY_POWER_CONSUMPTION
        energyThisSecond_grid += realPower_grid;
        if (perSecondCounter >= CYCLES_PER_SECOND) {
          perSecondCounter = 0;
          int powerInWatts = energyThisSecond_grid / IEU_per_Joule_grid;
          energyThisSecond_grid = 0;
          Serial.println(powerInWatts);
        } else
          perSecondCounter++;
#endif

        // Update energy bucket
        energyInBucket_long += realEnergy_grid;

        // Apply limits to bucket's level
        bool endOfRangeEncountered = false;
        if (energyInBucket_long > capacityOfEnergyBucket_long) {
          energyInBucket_long = capacityOfEnergyBucket_long;
          endOfRangeEncountered = true;
        } else if (energyInBucket_long < 0) {
          energyInBucket_long = 0;
          endOfRangeEncountered = true;
        }

        // Continuity checker
        sampleCount_forContinuityChecker++;
        if (sampleCount_forContinuityChecker >= CONTINUITY_CHECK_MAXCOUNT) {
          sampleCount_forContinuityChecker = 0;
#ifdef DISPLAY_LOWEST_SAMPLES_PER_CYCLE
          Serial.print("Lowest number of samples per mains cycle: ");
          Serial.println(lowestNoOfSampleSetsPerMainsCycle);
#endif
          lowestNoOfSampleSetsPerMainsCycle = 999;
        }

        // Reset accumulators for new cycle
        sampleSetsDuringThisMainsCycle = 0;
        sumP_grid = 0;
        sampleSetsDuringNegativeHalfOfMainsCycle = 0;
      } else {
        // Wait for DC-blocking filters to settle
        if (millis() > (delayBeforeSerialStarts + startUpPeriod) * 1000) {
          beyondStartUpPhase = true;
          sumP_grid = 0;
          sampleSetsDuringThisMainsCycle = 0;
          sampleCount_forContinuityChecker = 1;
          lowestNoOfSampleSetsPerMainsCycle = 999;
          Serial.println("Go!");
        }
      }
    }
  } else { // NEGATIVE polarity
    if (polarityConfirmedOfLastSampleV != NEGATIVE) {
      // Start of new -ve half cycle
      // Update LPF for DC-offset removal
      long previousOffset = DCoffset_V_long;
      DCoffset_V_long = previousOffset + (cumVdeltasThisCycle_long >> 12);
      cumVdeltasThisCycle_long = 0;

      // Keep LPF output in valid range
      if (DCoffset_V_long < DCoffset_V_min) {
        DCoffset_V_long = DCoffset_V_min;
      } else if (DCoffset_V_long > DCoffset_V_max) {
        DCoffset_V_long = DCoffset_V_max;
      }

      // Predict energy state at end of cycle using first half measurements
      long averagePower = sumP_grid / sampleSetsDuringThisMainsCycle;
      energyInBucket_prediction = energyInBucket_long + averagePower;
    }

    // Check if trigger device can be armed (after 3 sample sets in negative half)
    if (sampleSetsDuringNegativeHalfOfMainsCycle == 3) {
      if (beyondStartUpPhase) {
        enum loadStates prevStateOfLoad = nextStateOfLoad;
        nextStateOfLoad = (energyInBucket_prediction < singleEnergyThreshold_long) ? LOAD_OFF : LOAD_ON;
        writeStates();
      }
    }

    sampleSetsDuringNegativeHalfOfMainsCycle++;
  }

  // Process every sample set
  // Remove DC offset from current sample
  long sampleIminusDC_grid = ((long)(sampleI_grid - DCoffset_I)) << 8;
  
  // Apply extra filtering to offset HPF effect of CT1
  long last_lpf_long = lpf_long;
  lpf_long = last_lpf_long + alpha * (sampleIminusDC_grid - last_lpf_long);
  sampleIminusDC_grid += (lpf_gain * lpf_long);
  
  // Calculate real power and add to sum
  long filtV_div4 = sampleVminusDC_long >> 2;
  long filtI_div4 = sampleIminusDC_grid >> 2;
  long instP = filtV_div4 * filtI_div4;
  instP = instP >> 12;
  sumP_grid += instP;

  sampleSetsDuringThisMainsCycle++;
  cumVdeltasThisCycle_long += sampleVminusDC_long;
  polarityConfirmedOfLastSampleV = polarityConfirmed;
}

void confirmPolarity() {
  static byte count = 0;
  if (polarityOfMostRecentVsample != polarityConfirmedOfLastSampleV) {
    count++;
  } else {
    count = 0;
  }

  if (count > PERSISTENCE_FOR_POLARITY_CHANGE) {
    count = 0;
    polarityConfirmed = polarityOfMostRecentVsample;
  }
}

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

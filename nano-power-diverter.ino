#include <PinChangeInterrupt.h>  //because we're using a non standard interrupt pins
#include <Arduino.h>
#include <TimerOne.h>
#include "nonBlockingDelay.h"

// Idle time configuration
#define ADC_TIMER_PERIOD 130  // uS (idle time sets, and is inversely proportional to, the number of samples per mains wave cycle)

// Debugging-using-Serial code blocks
//#define WORKLOAD_CHECK
#define DISPLAY_POWER_CONSUMPTION
//#define DISPLAY_LOWEST_SAMPLES_PER_CYCLE

// Physical constants, please do not change!
#define SECONDS_PER_MINUTE 60
#define MINUTES_PER_HOUR 60
#define JOULES_PER_WATT_HOUR 3600  //  (0.001 kWh = 3600 Joules)

// Change these values to suit the local mains frequency and supply meter
#define CYCLES_PER_SECOND 50
#define WORKING_RANGE_IN_JOULES 360  // 0.1 Wh, reduced for faster start-up
#define REQUIRED_EXPORT_IN_WATTS 0   // when set to a negative value, this acts as a PV generator

// to prevent the diverted energy total from 'creeping'
#define ANTI_CREEP_LIMIT 5  // in Joules per mains cycle (has no effect when set to 0)
long antiCreepLimit_inIEUperMainsCycle;

#ifdef DISPLAY_POWER_CONSUMPTION
long energyThisSecond_grid;
int perSecondCounter;
long IEU_per_Joule_grid;
#endif

//function declarations
inline void writeStates(void) __attribute__((always_inline));
void timerIsr(void);
void allGeneralProcessing(void);


// definition of enumerated types
enum polarities { NEGATIVE, POSITIVE };
enum ledStates { LED_OFF, LED_ON };  // for use at port D3 which is active-high
enum loadStates { LOAD_ON, LOAD_OFF };  // for use at port D4 which is active-low
static enum loadStates nextStateOfLoad = LOAD_OFF;

// For this go-faster version, the unit's operation will effectively always be "Normal";
// there is no "Anti-flicker" option. The controlMode variable has been removed.

// Analogue pin allocations
const byte voltageSensor = 7;       // Analogue voltage sensor
const byte currentSensor_grid = 3;  // Analogue current sensor

// PORT_B Digital pin allocations
const byte outputForBlueLED = 11;  // Active low
const byte outputForRedLED = 10;   // Active low
const byte outputForTrigger = 9;   // Active low
const byte outputForExternal = 8;  // Active High, output to 3.3V ESP32 via a voltage divider

// Hardware interrupt Digital pin allocations
const byte EspInput = 3;        // Input from Esp32 via voltage divider
const byte overrideButton = 2;  //  Input from manual boost button


// General global variables that are used in multiple blocks so cannot be static.
// For integer maths, many variables need to be 'long'

const byte delayBeforeSerialStarts = 1;  // in seconds, to allow Serial window to be opened
const byte startUpPeriod = 3;            // in seconds, to allow LP filter to settle
const int DCoffset_I = 512;              // nominal mid-point value of ADC @ x1 scale

bool beyondStartUpPhase = false;          // start-up delay, allows things to settle
long energyInBucket_long;                 // in Integer Energy Units
long capacityOfEnergyBucket_long;         // depends on powerCal, frequency & the 'sweetzone' size.
long singleEnergyThreshold_long;          // for go-faster use
long DCoffset_V_long;                     // <--- for LPF
long DCoffset_V_min;                      // <--- for LPF
long DCoffset_V_max;                      // <--- for LPF
long divertedEnergyRecent_IEU = 0;        // Hi-res accumulator of limited range
unsigned int divertedEnergyTotal_Wh = 0;  // WattHour register of 63K range
long IEU_per_Wh;                          // depends on powerCal, frequency & the 'sweetzone' size.
unsigned long absenceOfDivertedEnergyCount = 0;
long mainsCyclesPerHour;
const unsigned long manualOverrideInterval = 3600000L;  //ie 1 hour

// for interaction between the main processor and the timer ISRs
volatile boolean dataReady = false;
volatile int sampleI_grid;
volatile int sampleV;

// Hardware interrupt functions
volatile int IsrEvent = 0;
void executeOverride() {
  IsrEvent = 1;
}
void executeEsp() {
  IsrEvent = 2;
}

// For an enhanced polarity detection mechanism, which includes a persistence check
#define PERSISTENCE_FOR_POLARITY_CHANGE 1  // sample sets
enum polarities polarityOfMostRecentVsample;
enum polarities polarityConfirmed;
enum polarities polarityConfirmedOfLastSampleV;

// For a mechanism to check the continuity of the sampling sequence
#define CONTINUITY_CHECK_MAXCOUNT 250  // mains cycles
int sampleCount_forContinuityChecker;
int sampleSetsDuringThisMainsCycle;
int lowestNoOfSampleSetsPerMainsCycle;




// Calibration values
//-------------------
// Two calibration values are used: powerCal and phaseCal.
// A full explanation of each of these values now follows:
//
// powerCal is a floating point variable which is used for converting the
// product of voltage and current samples into Watts.
//
// The correct value of powerCal is dependent on the hardware that is
// in use.  For best resolution, the hardware should be configured so that the
// voltage and current waveforms each span most of the ADC's usable range.  For
// many systems, the maximum power that will need to be measured is around 3kW.
//
// My sketch "RawSamplesTool_2chan.ino" provides a one-shot visual display of the
// voltage and current waveforms.  This provides an easy way for the user to be
// confident that their system has been set up correctly for the power levels
// that are to be measured.
//
// The ADC has an input range of 0-5V and an output range of 0-1023 levels.
// The purpose of each input sensor is to convert the measured parameter into a
// low-voltage signal which fits nicely within the ADC's input range.
//
// In the case of 240V mains voltage, the numerical value of the input signal
// in Volts is likely to be fairly similar to the output signal in ADC levels.
// 240V AC has a peak-to-peak amplitude of 679V, which is not far from the ideal
// output range.  Stated more formally, the conversion rate of the overall system
// for measuring VOLTAGE is likely to be around 1 ADC-step per Volt (RMS).
//
// In the case of AC current, however, the situation is very different.  At
// mains voltage, a power of 3kW corresponds to an RMS current of 12.5A which
// has a peak-to-peak range of 35A.  This is smaller than the output signal by
// around a factor of twenty.  The conversion rate of the overall system for
// measuring CURRENT is therefore likely to be around 20 ADC-steps per Amp.
//
// When calculating "real power", which is what this code does, the individual
// conversion rates for voltage and current are not of importance.  It is
// only the conversion rate for POWER which is important.  This is the
// product of the individual conversion rates for voltage and current.  It
// therefore has the units of ADC-steps squared per Watt.  Most systems will
// have a power conversion rate of around 20 (ADC-steps squared per Watt).
//
// powerCal is the RECIPR0CAL of the power conversion rate.  A good value
// to start with is therefore 1/20 = 0.05 (Watts per ADC-step squared)
//
const float powerCal_grid = 0.192;  //0.0435;  // for CT1

// for this go-faster sketch, the phaseCal logic has been removed.

long requiredExportPerMainsCycle_inIEU;

void setup() {
  pinMode(overrideButton, INPUT);  //pulled low by hardware, active high
  pinMode(outputForTrigger, OUTPUT);
  pinMode(outputForBlueLED, OUTPUT);
  pinMode(outputForRedLED, OUTPUT);
  pinMode(outputForExternal, OUTPUT);
  digitalWrite(outputForTrigger, HIGH);  // the external trigger is active low activate internal pullup
  digitalWrite(outputForBlueLED, HIGH);  //active LOW (has hardware pullup)
  digitalWrite(outputForRedLED, HIGH);   //active LOW (has hardware pullup)
  digitalWrite(outputForExternal, LOW);  //pulled low  (active high for ESP32)
  digitalWrite(outputForTrigger, LOAD_OFF);
  delay(delayBeforeSerialStarts * 1000);  // allow time to open Serial monitor
  Serial.begin(9600);
  Serial.println();
  Serial.println("-------------------------------------");
  Serial.println("Sketch ID:      Mk2_fasterControl_3.ino");
  Serial.println();

  // When using integer maths, calibration values that have supplied in floating point
  // form need to be rescaled.

  // When using integer maths, the SIZE of the ENERGY BUCKET is altered to match the
  // scaling of the energy detection mechanism that is in use.  This avoids the need
  // to re-scale every energy contribution, thus saving processing time.  This process
  // is described in more detail in the function, allGeneralProcessing(), just before
  // the energy bucket is updated at the start of each new cycle of the mains.
  //
  // An electricity meter has a small range over which energy can ebb and flow without
  // penalty.  This has been termed its "sweet-zone".  For optimal performance, the energy
  // bucket of a PV Router should match this value.  The sweet-zone value is therefore
  // included in the calculation below.
  //
  // For the flow of energy at the 'grid' connection point (CT1)
  capacityOfEnergyBucket_long =
    (long)WORKING_RANGE_IN_JOULES * CYCLES_PER_SECOND * (1 / powerCal_grid);
  energyInBucket_long = 0;

  singleEnergyThreshold_long = capacityOfEnergyBucket_long * 0.5;

  // For recording the accumulated amount of diverted energy data (using CT2), a similar
  // calibration mechanism is required.  Rather than a bucket with a fixed capacity, the
  // accumulator for diverted energy just needs to be scaled correctly.  As soon as its
  // value exceeds 1 Wh, an associated WattHour register is incremented, and the
  // accumulator's value is decremented accordingly. The calculation below is to determine
  // the scaling for this accumulator.
  //IEU_per_Wh =  (long)JOULES_PER_WATT_HOUR * CYCLES_PER_SECOND * (1/powerCal_diverted);
  // to avoid the diverted energy accumulator 'creeping' when the load is not active
  antiCreepLimit_inIEUperMainsCycle = (float)ANTI_CREEP_LIMIT * (1 / powerCal_grid);
  mainsCyclesPerHour = (long)CYCLES_PER_SECOND * SECONDS_PER_MINUTE * MINUTES_PER_HOUR;
  requiredExportPerMainsCycle_inIEU = (long)REQUIRED_EXPORT_IN_WATTS * (1 / powerCal_grid);

#ifdef DISPLAY_POWER_CONSUMPTION
  IEU_per_Joule_grid = (long)CYCLES_PER_SECOND * (1 / powerCal_grid);
#endif
  // Define operating limits for the LP filter which identifies DC offset in the voltage
  // sample stream.  By limiting the output range, the filter always should start up
  // correctly.
  DCoffset_V_long = 512L * 256;               // nominal mid-point value of ADC @ x256 scale
  DCoffset_V_min = (long)(512L - 100) * 256;  // mid-point of ADC minus a working margin
  DCoffset_V_max = (long)(512L + 100) * 256;  // mid-point of ADC plus a working margin

  Serial.print("ADC mode:       ");
  Serial.print(ADC_TIMER_PERIOD);
  Serial.println(" uS fixed timer");

  // Set up the ADC to be triggered by a hardware timer of fixed duration
  ADCSRA = (1 << ADPS0) + (1 << ADPS1) + (1 << ADPS2);  // Set the ADC's clock to system clock / 128
  ADCSRA |= (1 << ADEN);                                // Enable ADC

  Timer1.initialize(ADC_TIMER_PERIOD);                                      // set Timer1 interval
  Timer1.attachInterrupt(timerIsr);                                         // declare timerIsr() as interrupt service routine
  attachPCINT(digitalPinToPCINT(overrideButton), executeOverride, CHANGE);  //use library to force an interrupt on a regular digital pin
  attachInterrupt(digitalPinToInterrupt(EspInput), executeEsp, RISING);

  Serial.print("powerCal_grid =      ");
  Serial.println(powerCal_grid, 4);
  //Serial.print ( "powerCal_diverted = "); Serial.println (powerCal_diverted,4);

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
  Serial.println(freeRam());  // a useful value to keep an eye on

  Serial.println("----");

#ifdef WORKLOAD_CHECK
  Serial.println("WELCOME TO WORKLOAD_CHECK ");
  Serial.println("  The displayed value is the amount of spare time, per set of V & I samples, ");
  Serial.println("that is available for doing additional processing.");
  Serial.println();
#endif
}

// An Interrupt Service Routine is now defined in which the ADC is instructed to
// measure each analogue input in sequence.  A "data ready" flag is set after each
// voltage conversion has been completed.
//   For each set of samples, the two samples for current  are taken before the one
// for voltage.  This is appropriate because each waveform current is generally slightly
// advanced relative to the waveform for voltage.  The data ready flag is cleared
// within loop().
//   This Interrupt Service Routine is for use when the ADC is fixed timer mode.  It is
// executed whenever the ADC timer expires.  In this mode, the next ADC conversion is
// initiated from within this ISR.
//

void timerIsr(void) {
  static unsigned char sample_index = 0;
  static int sampleI_grid_raw;


  switch (sample_index) {
    case 0:
      sampleV = ADC;                      // store the ADC value (this one is for Voltage)
      ADMUX = 0x40 + currentSensor_grid;  // set up the next conversion, which is for grid Current
      ADCSRA |= (1 << ADSC);              // start the ADC
      sample_index++;                     // increment the control flag
      sampleI_grid = sampleI_grid_raw;
      dataReady = true;  // all three ADC values can now be processed
      break;
    case 1:
      sampleI_grid_raw = ADC;        // store the ADC value (this one is for Grid Current)
      ADMUX = 0x40 + voltageSensor;  // set up the next conversion, which is for Voltage
      ADCSRA |= (1 << ADSC);         // start the ADC
      sample_index = 0;              // reset the control flag
      break;
    default:
      sample_index = 0;  // to prevent lockup (should never get here)
  }
}


// When using interrupt-based logic, the main processor waits in loop() until the
// dataReady flag has been set by the ADC.  Once this flag has been set, the main
// processor clears the flag and proceeds with all the processing for one set of
// V & I samples.  It then returns to loop() to wait for the next set to become
// available.
//   If there is insufficient processing capacity to do all that is required, the
//  workload rate can be reduced by increasing the duration of ADC_TIMER_PERIOD.
//



inline void writeStates(void) {
  /*
  Replaces multiple Digital Writes which take about 3.4 microseconds on each pin to complete
  Portmapping sets all the IO pins at in under ~ 0.4 us.
  PORTB is responsible for setting digital pins 8 to 13 
  For example PORTB = B00101001; sets digital pins 8,11,13 HIGH 
  (count 8 to 13 from right to left and the two leading digits must be zero)
  
  This was the old way of doing it!
  //digitalWrite(outputForBlueLED, !nextStateOfLoad);
  //digitalWrite(outputForRedLED, nextStateOfLoad);
  //digitalWrite(outputForExternal, !nextStateOfLoad);  //<-- esp32 active high
  digitalWrite(outputForTrigger, nextStateOfLoad);

  */
  PORTB =  (( !nextStateOfLoad & 1 ) << 3) |  /*pin11, blue LED ON indicator, active high  */
            (( nextStateOfLoad & 1 ) << 2) |  /*pin10, red LED OFF indicator, active high  */
            (( nextStateOfLoad & 1 ) << 1) |  /*pin9,  trigger, active low*/
            ( !nextStateOfLoad & 1 );         /*pin8,  esp32, active high*/
}

void loop() {
#ifdef WORKLOAD_CHECK
  static int del = 0;           // delay, as passed to delayMicroseconds()
  static int res = 0;           // result, to be displayed at the next opportunity
  static byte count = 0;        // to allow multiple runs per setting
  static byte displayFlag = 0;  // to determine when printing may occur
#endif
  if (dataReady) {              // flag is set after every set of ADC conversions
    dataReady = false;          // reset the flag
    if (IsrEvent) {
      if (IsrEvent == 1) {
        IsrEvent = 0;
        nblock_delay manualOverrideDelay(manualOverrideInterval);
        nextStateOfLoad = LOAD_ON;
        //digitalWrite(outputForTrigger, nextStateOfLoad);  //trigger is active low
        writeStates();
        while (manualOverrideDelay.trigger()) {}  //returns true until time has elapsed

      } else if (IsrEvent == 2) {
        IsrEvent = 0;
        nextStateOfLoad = LOAD_ON;
        //digitalWrite(outputForTrigger, nextStateOfLoad);
        writeStates();
        while (digitalRead(EspInput) == HIGH) {}

      } else {
        IsrEvent = 0;
      }
    } else {
      allGeneralProcessing();  // executed once for each set of V&I samples
    }



#ifdef WORKLOAD_CHECK
    delayMicroseconds(del);  // <--- to assess how much spare time there is
    if (dataReady)           // if data is ready again, delay was too long
    {
      res = del;  // note the exact value
      del = 1;    // and start again with 1us delay
      count = 0;
      displayFlag = 0;
    } else {
      count++;  // to give several runs with the same value
      if (count > 50) {
        count = 0;
        del++;  //  increase delay by 1uS
      }
    }
#endif
  }  // <-- this closing brace needs to be outside the WORKLOAD_CHECK blocks!
#ifdef WORKLOAD_CHECK
  switch (displayFlag) {
    case 0:  // the result is available now, but don't display until the next loop
      displayFlag++;
      break;
    case 1:  // with minimum delay, it's OK to print now
      Serial.print(res);
      displayFlag++;
      break;
    case 2:  // with minimum delay, it's OK to print now
      Serial.println("uS");
      displayFlag++;
      break;
    default:;  // for most of the time, displayFlag is 3
  }
#endif
}  // end of loop()


// This routine is called to process each set of V & I samples. The main processor and
// the ADC work autonomously, their operation being only linked via the dataReady flag.
// As soon as a new set of data is made available by the ADC, the main processor can
// start to work on it immediately.
//
void allGeneralProcessing() {
  static long sumP_grid;  // for per-cycle summation of 'real power'
  static long cumVdeltasThisCycle_long;  // for the LPF which determines DC offset (voltage)
  static byte timerForDisplayUpdate = 0;
  static int sampleSetsDuringNegativeHalfOfMainsCycle;  // for arming the triac/trigger
  static long energyInBucket_prediction;
  
  // extra items for an LPF to improve the processing of data samples from CT1
  static long lpf_long;  // new LPF, for ofsetting the behaviour of CT1 as a HPF
  
  // The next two constants determine the profile of the LPF.
  // They are matched to the physical behaviour of the YHDC SCT-013-000 CT
  // and the CT1 samples being 375 us apart
  
  const float lpf_gain = 8;   // <- setting this to 0 disables this extra processing
  const float alpha = 0.002;  //

  // remove DC offset from the raw voltage sample by subtracting the accurate value
  // as determined by a LP filter.
  long sampleVminusDC_long = ((long)sampleV << 8) - DCoffset_V_long;

  // determine the polarity of the latest voltage sample
  if (sampleVminusDC_long > 0) {
    polarityOfMostRecentVsample = POSITIVE;
  } else {
    polarityOfMostRecentVsample = NEGATIVE;
  }
  confirmPolarity();

  if (polarityConfirmed == POSITIVE) {
    if (polarityConfirmedOfLastSampleV != POSITIVE) {
      // This is the start of a new +ve half cycle (just after the zero-crossing point)
      if (beyondStartUpPhase) {
        // a simple routine for checking the performance of this new ISR structure
        if (sampleSetsDuringThisMainsCycle < lowestNoOfSampleSetsPerMainsCycle) {
          lowestNoOfSampleSetsPerMainsCycle = sampleSetsDuringThisMainsCycle;
        }

        // Calculate the real power and energy during the last whole mains cycle.
        //
        // sumP contains the sum of many individual calculations of instantaneous power.  In
        // order to obtain the average power during the relevant period, sumP must first be
        // divided by the number of samples that have contributed to its value.
        //
        // The next stage would normally be to apply a calibration factor so that real power
        // can be expressed in Watts.  That's fine for floating point maths, but it's not such
        // a good idea when integer maths is being used.  To keep the numbers large, and also
        // to save time, calibration of power is omitted at this stage.  Real Power (stored as
        // a 'long') is therefore (1/powerCal) times larger than the actual power in Watts.
        //
        long realPower_grid = sumP_grid / sampleSetsDuringThisMainsCycle;  // proportional to Watts
        realPower_grid -= requiredExportPerMainsCycle_inIEU;  // <- useful for PV simulation

        // Next, the energy content of this power rating needs to be determined.  Energy is
        // power multiplied by time, so the next step would normally be to multiply the measured
        // value of power by the time over which it was measured.
        //   Instanstaneous power is calculated once every mains cycle. When integer maths is
        // being used, a repetitive power-to-energy conversion seems an unnecessary workload.
        // As all sampling periods are of similar duration, it is more efficient to just
        // add all of the power samples together, and note that their sum is actually
        // CYCLES_PER_SECOND greater than it would otherwise be.
        //   Although the numerical value itself does not change, I thought that a new name
        // may be helpful so as to minimise confusion.
        //   The 'energy' variable below is CYCLES_PER_SECOND * (1/powerCal) times larger than
        // the actual energy in Joules.
        //
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

        // Energy contributions from the grid connection point (CT1) are summed in an
        // accumulator which is known as the energy bucket.  The purpose of the energy bucket
        // is to mimic the operation of the supply meter.  The range over which energy can
        // pass to and fro without loss or charge to the user is known as its 'sweet-zone'.
        // The capacity of the energy bucket is set to this same value within setup().
        //
        // The latest contribution can now be added to this energy bucket
        energyInBucket_long += realEnergy_grid;

        // Apply max and min limits to bucket's level.  This is to ensure correct operation
        // when conditions change, i.e. when import changes to export, and vici versa.
        //
        bool endOfRangeEncountered = false;
        if (energyInBucket_long > capacityOfEnergyBucket_long) {
          energyInBucket_long = capacityOfEnergyBucket_long;
          endOfRangeEncountered = true;
        } else if (energyInBucket_long < 0) {
          energyInBucket_long = 0;
          endOfRangeEncountered = true;
        }







        // continuity checker
        sampleCount_forContinuityChecker++;
        if (sampleCount_forContinuityChecker >= CONTINUITY_CHECK_MAXCOUNT) {
          sampleCount_forContinuityChecker = 0;
#ifdef DISPLAY_LOWEST_SAMPLES_PER_CYCLE
          Serial.print("Lowest number of samples per mains cycle: ");
          Serial.println(lowestNoOfSampleSetsPerMainsCycle);
#endif
          lowestNoOfSampleSetsPerMainsCycle = 999;
        }

        // clear the per-cycle accumulators for use in this new mains cycle.
        sampleSetsDuringThisMainsCycle = 0;
        sumP_grid = 0;
        sampleSetsDuringNegativeHalfOfMainsCycle = 0;

      } else {
        // wait until the DC-blocking filters have had time to settle
        if (millis() > (delayBeforeSerialStarts + startUpPeriod) * 1000) {
          beyondStartUpPhase = true;
          sumP_grid = 0;
          sampleSetsDuringThisMainsCycle = 0;    // not yet dealt with for this cycle
          sampleCount_forContinuityChecker = 1;  // opportunity has been missed for this cycle
          lowestNoOfSampleSetsPerMainsCycle = 999;
          Serial.println("Go!");
        }
      }
    }  // end of processing that is specific to the first Vsample in each +ve half cycle

    // still processing samples where the voltage is POSITIVE ...
    // (in this go-faster code, the action from here has moved to the negative half of the cycle)

  }  // end of processing that is specific to samples where the voltage is positive

  else  // the polatity of this sample is negative
  {
    if (polarityConfirmedOfLastSampleV != NEGATIVE) {
      // This is the start of a new -ve half cycle (just after the zero-crossing point)
      // which is a convenient point to update the Low Pass Filter for DC-offset removal
      //  The portion which is fed back into the integrator is approximately one percent
      // of the average offset of all the Vsamples in the previous mains cycle.
      
      long previousOffset = DCoffset_V_long;
      DCoffset_V_long = previousOffset + (cumVdeltasThisCycle_long >> 12);
      cumVdeltasThisCycle_long = 0;

      // To ensure that the LPF will always start up correctly when 240V AC is available, its
      // output value needs to be prevented from drifting beyond the likely range of the
      // voltage signal.  This avoids the need to use a HPF as was done for initial Mk2 builds.
      
      if (DCoffset_V_long < DCoffset_V_min) {
        DCoffset_V_long = DCoffset_V_min;
      } else if (DCoffset_V_long > DCoffset_V_max) {
        DCoffset_V_long = DCoffset_V_max;
      }

      // The average power that has been measured during the first half of this mains cycle can now be used
      // to predict the energy state at the end of this mains cycle.  That prediction will be used to alter
      // the state of the load as necessary. The arming signal for the triac can't be set yet - that must
      // wait until the voltage has advanced further beyond the -ve going zero-crossing point.
      
      long averagePower = sumP_grid / sampleSetsDuringThisMainsCycle;  // for 1st half of this mains cycle only
      
      // To avoid repetitive and unnecessary calculations, the increase in energy during each mains cycle is
      // deemed to be numerically equal to the average power.  The predicted value for the energy state at the
      // end of this mains cycle will therefore be the known energy state at its start plus the average power
      // as measured. Although the average power has been determined over only half a mains cycle, the correct
      // number of contributing sample sets has been used so the result can be expected to be a true measurement
      // of average power, not half of it.
      
      energyInBucket_prediction = energyInBucket_long + averagePower;  // at end of this mains cycle


    }  // end of processing that is specific to the first Vsample in each -ve half cycle

    // check to see whether the trigger device can now be reliably armed
    if (sampleSetsDuringNegativeHalfOfMainsCycle == 3) {
      if (beyondStartUpPhase) {
        enum loadStates prevStateOfLoad = nextStateOfLoad;
        if (energyInBucket_prediction < singleEnergyThreshold_long) {
          nextStateOfLoad = LOAD_OFF;
        } else
          nextStateOfLoad = LOAD_ON;

        // set the Arduino's output pin accordingly, and clear the flag
        //digitalWrite(outputForTrigger, nextStateOfLoad);
        writeStates();
      }
    }

    sampleSetsDuringNegativeHalfOfMainsCycle++;
  }  // end of processing that is specific to samples where the voltage is negative

  // processing for EVERY set of samples
  // First, deal with the power at the grid connection point (as measured via CT1)
  // remove most of the DC offset from the current sample (the precise value does not matter)
  long sampleIminusDC_grid = ((long)(sampleI_grid - DCoffset_I)) << 8;
  
  // extra filtering to offset the HPF effect of CT1
  long last_lpf_long = lpf_long;
  lpf_long = last_lpf_long + alpha * (sampleIminusDC_grid - last_lpf_long);
  sampleIminusDC_grid += (lpf_gain * lpf_long);

  
  // calculate the "real power" in this sample pair and add to the accumulated sum
  long filtV_div4 = sampleVminusDC_long >> 2;  // reduce to 16-bits (now x64, or 2^6)
  long filtI_div4 = sampleIminusDC_grid >> 2;  // reduce to 16-bits (now x64, or 2^6)
  long instP = filtV_div4 * filtI_div4;        // 32-bits (now x4096, or 2^12)
  instP = instP >> 12;                         // scaling is now x1, as for Mk2 (V_ADC x I_ADC)
  sumP_grid += instP;                          // cumulative power, scaling as for Mk2 (V_ADC x I_ADC)


  sampleSetsDuringThisMainsCycle++;

  // store items for use during next loop
  cumVdeltasThisCycle_long += sampleVminusDC_long;     // for use with LP filter
  polarityConfirmedOfLastSampleV = polarityConfirmed;  // for identification of half cycle boundaries
}
//  ----- end of main Mk2i code -----

void confirmPolarity() {
  /* This routine prevents a zero-crossing point from being declared until 
   * a certain number of consecutive samples in the 'other' half of the 
   * waveform have been encountered.  
   */
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

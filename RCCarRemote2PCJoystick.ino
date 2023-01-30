#include <Joystick.h>
#include <limits.h>

// Enable to export a joystick using the usb of the leonardo/micro pro (atmega32U4) 
#define JOYSTICK_OUTPUT_ENABLE

// Enable Debug
//#define JOYSTICK_DEBUG_ENABLE
//#define POSITION_DEBUG_ENABLE
const unsigned long DEBUG_PERIOD = 500000; // in micros
//#define PULSE_DEBUG_ENABLE
//#define COUNT_OF_PULSES_TO_SHOW_DEBUG 20


// DONT SET AVOBE 3 IF THE SERIAL DEBUG IS ENABLED (pins 0 and 1 cant be used to get interruptions and the serial port at the same time)
#define CHANNEL_COUNT 2 
// Number of pins that can trigger interruptions, 5 for arduino micro pro
#define SUPPORTED_CHANNEL_MAX_COUNT 5


const int channelPin[SUPPORTED_CHANNEL_MAX_COUNT] = {2,3,7,1,0}; // Available pins with interruptions for arduino micro pro
const int initialDiscardedBootReadings = 3;
const int calibrationReadings = 5;
const long VALUE_RESOLUTION = 2048; // Ammount of different levels in a reading
float hysteresisPercentage = 3.0;



/*
 Internal status
*/
long channelBootReadingCount[CHANNEL_COUNT];
long channeNewMaxReadingCount[CHANNEL_COUNT];
long channeNewMinReadingCount[CHANNEL_COUNT];
float newChannelMax[CHANNEL_COUNT];
float newChannelMin[CHANNEL_COUNT];
float channelValueAtBoot[CHANNEL_COUNT];
float channelMax[CHANNEL_COUNT];
float channelMin[CHANNEL_COUNT];
float lastChannelValue[CHANNEL_COUNT];
unsigned long pulseBeginningTime[CHANNEL_COUNT];
unsigned long pulseOnDuration[CHANNEL_COUNT];
unsigned long pulseDuration[CHANNEL_COUNT];
unsigned long lastDebugTime = 0;


/*
Interruption array
*/
void (*channelRisingFunc[SUPPORTED_CHANNEL_MAX_COUNT])() { channel0Rising, channel1Rising, channel2Rising, channel3Rising, channel4Rising };
void (*channelFallingFunc[SUPPORTED_CHANNEL_MAX_COUNT])() { channel0Falling, channel1Falling, channel2Falling, channel3Falling, channel4Falling };


#ifdef JOYSTICK_OUTPUT_ENABLE
const int MAX_JOYSTICK_VALUE = JOYSTICK_DEFAULT_SIMULATOR_MAXIMUM;
const int MIN_JOYSTICK_VALUE = JOYSTICK_DEFAULT_SIMULATOR_MINIMUM;

// Create the Joystick
Joystick_ joystick(
  JOYSTICK_DEFAULT_REPORT_ID, // hidReportId
  JOYSTICK_TYPE_JOYSTICK, //joystickType
        max(min(CHANNEL_COUNT - 2, 2), 0), // buttonCount
        JOYSTICK_DEFAULT_HATSWITCH_COUNT, // hatSwitchCount
        false, // includeXAxis
        false, // includeYAxis
        CHANNEL_COUNT == 5, // includeZAxis 
        false, // includeRxAxis
        false, // includeRyAxis
        false, // includeRzAxis
        false, // includeRudder
        false, // includeThrottle
        true, // includeAccelerator
        true, // includeBrake
        true  // includeSteering
);
#endif

#ifdef JOYSTICK_DEBUG_ENABLE
long joystickSteeringPosition = 0;
long joystickAcceleratorPosition = 0;
long joystickBreakPosition = 0;
#endif


#ifdef PULSE_DEBUG_ENABLE
int pulseCount = 0;
#endif

/*********************************
* Joystick export
**********************************/
void joystickValueUpdate(int channel, int value, int midPoint, int min, int max) {
#ifdef JOYSTICK_OUTPUT_ENABLE
  if (channel == 0) { // Steering
    updateSteering(value, midPoint, min, max);
  } else if (channel == 1) { // Accelerator/Brake
    updateAcceleratorBrake(value, midPoint, min, max);
  } else if (channel == 2) { // CH3
    updateCH3(value, min, max);
  } else if (channel == 3) { // CH4
    updateCH4(value, min, max);
  } else if (channel == 4) { // CH5
    updateCH5(value, min, max);
  }
#endif
}

#ifdef JOYSTICK_OUTPUT_ENABLE
void updateSteering(float value, float midPoint, float min, float max) {

  float joystickCenterPosition = (MAX_JOYSTICK_VALUE - MIN_JOYSTICK_VALUE) / 2;
  float joystickPosition = joystickCenterPosition;

  if (value >= midPoint) {
    joystickPosition += (value - midPoint) * (MAX_JOYSTICK_VALUE - joystickCenterPosition) / (max - midPoint);
  } else {
    joystickPosition += (midPoint - value) * (MIN_JOYSTICK_VALUE - joystickCenterPosition) / (midPoint - min);
  }
  joystick.setSteering(joystickPosition);
  #ifdef JOYSTICK_DEBUG_ENABLE
  joystickSteeringPosition = joystickPosition;
  #endif
}

void updateAcceleratorBrake(float value, float midPoint, float min, float max) {

  
    if (value >= midPoint) {
      float joystickPosition = ((value - midPoint) * (MAX_JOYSTICK_VALUE - MIN_JOYSTICK_VALUE) / (max - midPoint)) + MIN_JOYSTICK_VALUE;
      joystick.setAccelerator(joystickPosition);
      joystick.setBrake(MIN_JOYSTICK_VALUE);
#ifdef JOYSTICK_DEBUG_ENABLE
      joystickAcceleratorPosition = joystickPosition;
      joystickBreakPosition = MIN_JOYSTICK_VALUE;
#endif
    } else {
      float joystickPosition = ((midPoint - value) * (MAX_JOYSTICK_VALUE - MIN_JOYSTICK_VALUE) / (midPoint - min)) + MIN_JOYSTICK_VALUE;
      joystick.setAccelerator(MIN_JOYSTICK_VALUE);
      joystick.setBrake(joystickPosition);
#ifdef JOYSTICK_DEBUG_ENABLE
      joystickAcceleratorPosition = MIN_JOYSTICK_VALUE;
      joystickBreakPosition = joystickPosition;
#endif
    }

}

void updateCH3(float value, float min, float max) {
  
  
  if (value > min + ((max - min) / 2)) {
    joystick.setButton(0, HIGH);
  } else {
    joystick.setButton(0, LOW);
  }

}

void updateCH4(float value, float min, float max) {
  if (value > min + ((max - min) / 3)) {
    joystick.setButton(0, HIGH);
  } else {
    joystick.setButton(0, LOW);
  }

}

void updateCH5(float value, float min, float max) {

  long joystickPosition = ((value - min) * (MAX_JOYSTICK_VALUE - MIN_JOYSTICK_VALUE) / (max - min)) + MIN_JOYSTICK_VALUE;
  joystick.setZAxis(joystickPosition);
}

#endif

/*********************************
* Status management
**********************************/
void initializeChannel(int channel) {
  
  channelMax[channel] = -9999999;
  channelMin[channel] = 9999999;
  newChannelMax[channel] = 0;
  newChannelMin[channel] = 0;
  channeNewMaxReadingCount[channel] = 0;
  channeNewMinReadingCount[channel] = 0;
  channelBootReadingCount[channel] = 0;
  channelValueAtBoot[channel] = 0;

  pinMode(channelPin[channel], INPUT);
  attachInterrupt(digitalPinToInterrupt(channelPin[channel]), channelFallingFunc[channel], FALLING);
  pulseBeginningTime[channel] = micros();
  pulseOnDuration[channel] = 0;
  pulseDuration[channel] = 0;

}

bool channelLimitsUpdate(int channel, float reading) {
  // Check if new limit is reached

  bool limitChanged = false;
  if (reading < channelMin[channel]) {

    newChannelMin[channel] += reading;

    if (channeNewMinReadingCount[channel] >= (calibrationReadings - 1)) {
      channelMin[channel] = newChannelMin[channel] / calibrationReadings;
      channeNewMinReadingCount[channel] = 0;
      newChannelMin[channel] = 0;
      limitChanged = true;
    } else {
      channeNewMinReadingCount[channel]++;
    }
  } else {
    channeNewMinReadingCount[channel] = 0;
    newChannelMin[channel] = 0;
  }

  

  if (reading > channelMax[channel]) {

    newChannelMax[channel] += reading;
    if (channeNewMaxReadingCount[channel] >= (calibrationReadings - 1)) {
      channelMax[channel] = newChannelMax[channel] / calibrationReadings;
      channeNewMaxReadingCount[channel] = 0;
      newChannelMax[channel] = 0;
      limitChanged = true;
    } else {
      channeNewMaxReadingCount[channel]++;
    }

  } else {
    channeNewMaxReadingCount[channel] = 0;
    newChannelMax[channel] = 0;
  }
  return limitChanged;

}

bool channelValueUpdate(int channel, float value) {

  bool changed = false;
  if (value > ((lastChannelValue[channel] * (100.0 + hysteresisPercentage)) / 100.0 ) || 
    value < ((lastChannelValue[channel] * (100.0 - hysteresisPercentage)  ) / 100.0 ) ) {
    // update channel value
    lastChannelValue[channel] = value;
    changed = true;
  }
  return changed;
}


float computeValue(unsigned long pulseHighDurationInMicros, unsigned long pulseLengthInMicros) {
  //long pulseHighDurationInMicros = pulseTransitionMicros - pulseStartMicros;
  //return pulseHighDurationInMicros;

  //unsigned long pulseLengthInMicros = pulseEndMicros - pulseStartMicros;
  //unsigned long pulseHighDurationInMicros = pulseTransitionMicros - pulseStartMicros;
  return (VALUE_RESOLUTION * pulseHighDurationInMicros) / pulseLengthInMicros;
}

void channelAtBootCalibration(int channel, float value, int iteration, int maxIterations) {
  
  channelValueAtBoot[channel] += value;
  if (iteration == (maxIterations - 1)){
    channelValueAtBoot[channel] = channelValueAtBoot[channel] / maxIterations;
  }
}

void onReadValue(int channel, float value) {
    
    channelLimitsUpdate(channel, value);
    if (channelValueUpdate(channel, value)) {
      joystickValueUpdate(channel, value, channelValueAtBoot[channel], channelMin[channel], channelMax[channel]);
    }
}

/*********************************
* Pulse detection
**********************************/
void onPulseBegins(int channel, unsigned long startTime) {
  pulseBeginningTime[channel] = startTime;

}

void onPulseTransits(int channel, unsigned long transitTime) {
  pulseOnDuration[channel] = transitTime - pulseBeginningTime[channel]; 
}


void onPulseEnds(int channel, unsigned long endTime) {
  pulseDuration[channel] = endTime - pulseBeginningTime[channel];
  //debugPulse(channel, pulseBeginningTime[channel], pulseBeginningTime[channel] + pulseOnDuration[channel], endTime);
}


void onPulseRead(int channel, unsigned long pulseHighDuration, unsigned long pulseLength) {
  float value = computeValue(pulseHighDuration, pulseLength);

  debugReading(channel, pulseHighDuration, pulseLength, value);

  if (channelBootReadingCount[channel] < initialDiscardedBootReadings + calibrationReadings) {
      // Discarding the frist n samples
    if (channelBootReadingCount[channel] >= initialDiscardedBootReadings) {
      // Using the rest of the initial samples to calibrate the idle status.
      channelAtBootCalibration(channel, value, channelBootReadingCount[channel] - initialDiscardedBootReadings, calibrationReadings);
    }
    channelBootReadingCount[channel]++;

  } else {

    onReadValue(channel, value);

  }
}

void pulseLoop() { 

  for (int channel = 0; channel < CHANNEL_COUNT; channel++) {

    unsigned long pulseLength = pulseDuration[channel];
    if (pulseLength != 0) {
      pulseDuration[channel] = 0;
      unsigned long pulseHighDuration = pulseOnDuration[channel];
      onPulseRead(channel, pulseHighDuration, pulseLength);

    }
  }
}

/*********************************
* Interruption handling
**********************************/
void channelFalling(int channel) {
  attachInterrupt(digitalPinToInterrupt(channelPin[channel]), channelRisingFunc[channel], RISING);
  unsigned long time = micros();
  onPulseTransits(channel, time);
}

void channelRising(int channel) {
  attachInterrupt(digitalPinToInterrupt(channelPin[channel]), channelFallingFunc[channel], FALLING);
  unsigned long time = micros();
  onPulseEnds(channel, time);
  onPulseBegins(channel, time);
}


void channel0Falling() {
  channelFalling(0);
}

void channel0Rising() {
   channelRising(0);
}

void channel1Falling() {
  channelFalling(1);
}
void channel1Rising() {
  channelRising(1);
}

void channel2Falling() {
  channelFalling(2);
}
void channel2Rising() {
  channelRising(2);
}


void channel3Falling() {
  channelFalling(3);
}
void channel3Rising() {
  channelRising(3);
}


void channel4Falling() {
  channelFalling(4);
}
void channel4Rising() {
  channelRising(4);
}

/*********************************
* Debug functions
**********************************/
void debugLoop() {
#if defined(JOYSTICK_DEBUG_ENABLE) || defined(POSITION_DEBUG_ENABLE)

  unsigned long currentTime = micros();
  if (currentTime > lastDebugTime + DEBUG_PERIOD) {
    lastDebugTime = currentTime;
    debugPosition();
    debugJoystick();
    Serial.println("");
  }

#endif
}

void debugPosition() {
#ifdef POSITION_DEBUG_ENABLE
  for (int i = 0; i < CHANNEL_COUNT; i++) {
    if (i == 0) {
      Serial.print("Steering");
    } else if (i == 1) {
      Serial.print("Throttle");
    } else {
      Serial.print("Channel ");
      Serial.print( i + 1 );
    }
    Serial.print(" value ");
    Serial.print(lastChannelValue[i]) ;
    Serial.print(" min ");
    Serial.print(channelMin[i]);
    Serial.print(" max ");
    Serial.print(channelMax[i]);   
    Serial.print(" valueAtBoot ");
    Serial.print(channelValueAtBoot[i]);
    Serial.print(" position % ");
    long percentagePosition =  0;
    if (i < 2) {
      //percentagePosition = (long)(lastChannelValue[i] - channelMin[i]) * 200 / (long)(channelMax[i] - channelMin[i]) - 100;
      if (lastChannelValue[i] >= channelValueAtBoot[i]) {
        percentagePosition = (long)(lastChannelValue[i] - channelValueAtBoot[i]) * 100 / (long)(channelMax[i] - channelValueAtBoot[i]);
      } else {
        percentagePosition = (long)(channelValueAtBoot[i] - lastChannelValue[i]) * -100 / (long)(channelValueAtBoot[i] - channelMin[i]);
      }
    } else {
      percentagePosition = (long)(lastChannelValue[i] - channelMin[i]) * 100 / (long)(channelMax[i] - channelMin[i]);
    }
    Serial.print(percentagePosition);
    Serial.print("% ");
  }
#endif
}

void debugJoystick() {
 #ifdef JOYSTICK_DEBUG_ENABLE 
  Serial.print("JOY Steer ");
  Serial.print(joystickSteeringPosition);
  Serial.print(" Accel ");
  Serial.print(joystickAcceleratorPosition);
  Serial.print(" Brake ");
  Serial.print(joystickBreakPosition);
  Serial.print(" max ");
  Serial.print(MIN_JOYSTICK_VALUE);
  Serial.print(" min ");
  Serial.print(MAX_JOYSTICK_VALUE);
#endif
}


void debugPulse(int channel, unsigned long beginningTime, unsigned long transitionTime, unsigned long endTime) {
#ifdef PULSE_DEBUG_ENABLE
  if (pulseCount >= COUNT_OF_PULSES_TO_SHOW_DEBUG) {
    pulseCount = 0;
    Serial.print("Pulse channel ");
    Serial.print(channel);
    Serial.print(" starts ");
    Serial.print(beginningTime);
    Serial.print(" transits ");
    Serial.print(transitionTime);
    Serial.print(" ends ");
    Serial.println(endTime);
  } else {
    pulseCount++;
  }
#endif
}

void debugReading(int channel, unsigned long onTime, unsigned long pulseLength, float value) {
#ifdef PULSE_DEBUG_ENABLE
  if (pulseCount >= COUNT_OF_PULSES_TO_SHOW_DEBUG) {
    pulseCount = 0;
    Serial.print("Reading channel ");
    Serial.print(channel);
    Serial.print(" ON ");
    Serial.print(onTime);
    Serial.print(" length ");
    Serial.print(pulseLength);
    Serial.print(" value ");
    Serial.println(value);
  } else {
    pulseCount++;
  }
#endif
}

/*********************************
* Program control
**********************************/
void setup() {


  for (int i=0; i < CHANNEL_COUNT; i++) {
    initializeChannel(i);
  }


#ifdef JOYSTICK_OUTPUT_ENABLE
  // Initialize Joystick Library
  joystick.begin();

  joystick.setSteeringRange(MIN_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE);
  joystick.setAcceleratorRange(MIN_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE);
  joystick.setBrakeRange(MIN_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE);
  if (CHANNEL_COUNT == 5) {
    joystick.setZAxisRange(MIN_JOYSTICK_VALUE, MAX_JOYSTICK_VALUE);
  }

#endif

#if defined(JOYSTICK_DEBUG_ENABLE) || defined(POSITION_DEBUG_ENABLE) || defined(PULSE_DEBUG_ENABLE)
  Serial.begin(115200);  
#endif

}


void loop() {
  pulseLoop();
  debugLoop();
}


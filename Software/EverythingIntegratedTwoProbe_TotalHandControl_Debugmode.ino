#include <PWMServo.h>
#include <ADC.h>
ADC *adc = new ADC();  // adc object
#define handtriggerthreshold 400
#define secondhighestthreshold 100
PWMServo ID1;  // Define all servo motors for hand
PWMServo ID2;
PWMServo ID3;
PWMServo ID4;
PWMServo ID5;

int pin0 = 28;  // Dipswitches for debugging
int pin1 = 29;
int pin2 = 30;
int pin3 = 31;
int pin4 = 32;

bool handControl[3][5] = {
  { 0, 0, 0, 0, 0 },  // Checks the active state of each finger (handState)
  { 0, 0, 0, 0, 0 },  // Checks previous position of finger (handPosition)
  { 1, 1, 1, 1, 1 }   // Checks whether the finger should move based on active state & previous position (handActivate)
};

bool handTrigger[3] = { 0, 0, 0 };  // This is used for the hand open close function
bool trigger1 = true;

const int emgPin1 = A4;            // Analog input pin 18 for the sEMG signal
const int emgPin2 = A3;            // Analog input 17 pin for the sEMG signal
const int samplesPerCycle = 3000;  // Array samples

float normalizedValueArray1[samplesPerCycle];
float filteredValueArray1[samplesPerCycle];

float normalizedValueArray2[samplesPerCycle];
float filteredValueArray2[samplesPerCycle];

int rawValueArray1[samplesPerCycle];
int rawValueArray2[samplesPerCycle];

float recordPeak1 = 0.0;  // Highest recorded peak value
float recordPeak2 = 0.0;

float averageProbe = 0.0;
int cycle = 0;  // Number cycles elapsed

float dcOffset = 1.622;  // DC voltage offset is 1.622v in 9v operation, or 2.022v in 11.4v operation

float expGain = 4.0;

void setup() {
  Serial.begin(115000);                                             // Initialize serial communication buad rate
  adc->adc0->setAveraging(4);                                       // average 4 together
  adc->adc0->setResolution(10);                                     // 10 bits resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);  // Reference chart for speed. 4.5us per sample in this case
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_MED_SPEED);
  ID1.attach(0);  //23
  ID2.attach(1);
  ID3.attach(2);
  ID4.attach(3);
  ID5.attach(4);

  // attachInterrupt(digitalPinToInterrupt(27), interruptHandler, CHANGE); Didn't work as intended, using non-blocking method as substitute
}

// void interruptHandler() {
//   while(trigger1 == true){
//   Serial.println("Debug Mode");
//   debugMode();
//   trigger1 = digitalRead(27);
//   }
//   trigger1 = true;

// }

void debugMode() {

  trigger1 = digitalRead(27);

  while (trigger1 == true) {
    Serial.println("Debug Mode");

    debugControl();
    activatePos();  // This function avoids the need for setup, already initialized
    checkPos();     // Checks & updates position, state, and activate

    trigger1 = digitalRead(27);
    if (!trigger1) {
      handControl[0][0] = 0;  //This is for dipswitch configuration & testing
      handControl[0][1] = 0;  //EMG reading will call seperate filter function to set hand states
      handControl[0][2] = 0;
      handControl[0][3] = 0;
      handControl[0][4] = 0;
    }
  }



  activatePos();  // This function avoids the need for setup, already initialized
  checkPos();     // Checks & updates position, state, and activate
}

//Low pass chebyshev filter order=1 alpha1=0.0375
class FilterChLp1 {
public:
  FilterChLp1() {
    v[0] = v[1] = 0.0;
  }
private:
  float v[2];
public:
  float step(float x)  //class II
  {
    v[0] = v[1];
    v[1] = (1.060566927263208309e-1 * x)
           + (0.78788661454735831047 * v[0]);
    return (v[0] + v[1]);
  }
};

FilterChLp1 chebyshevFilter1;  // First order chebyshev LPF 40kHz at a 1500Hz low corner cut off
FilterChLp1 chebyshevFilter2;  // First order chebyshev LPF 40kHz at a 1500Hz low corner cut off

void emgSetup() {

  for (int i = 0; i < samplesPerCycle; i++) {
    rawValueArray1[i] = analogRead(emgPin1);  // Value is 0-1023
    rawValueArray2[i] = analogRead(emgPin2);  // Value is 0-1023
    delayMicroseconds(16);                    // FILTER NEEDS TO MATCH sample rate. 25uS = 40kHz currently. Anything slower will result in artifacting and distortion
  }
}

void debugControl() {

  handControl[0][0] = digitalRead(pin0);  //This is for dipswitch configuration & testing
  handControl[0][1] = digitalRead(pin1);  //EMG reading will call seperate filter function to set hand states
  handControl[0][2] = digitalRead(pin2);
  handControl[0][3] = digitalRead(pin3);
  handControl[0][4] = digitalRead(pin4);
}


void filterState() {

  for (int i = 0; i < samplesPerCycle; i++) {

    normalizedValueArray1[i] = rawValueArray1[i] / 1023.0;  // This is the normalized raw value array from 0.0 to 1.0
    normalizedValueArray2[i] = rawValueArray2[i] / 1023.0;

    filteredValueArray1[i] = pow(2.71828, expGain * abs((chebyshevFilter1.step(normalizedValueArray1[i]) * 3.3) - dcOffset)) - 1;  // Apply LPF to voltage minus the offset. Then rectify values. e^3.3x - 1. Arbitrary values
    filteredValueArray2[i] = pow(2.71828, expGain * abs((chebyshevFilter2.step(normalizedValueArray2[i]) * 3.3) - dcOffset)) - 1;  // Second probe, will possibly require much less gain.
  }
}

void peakDetector() {

  float filtPeak1 = 0.0;
  float filtPeak2 = 0.0;

  float tempFilt1 = 0.0;
  float tempFilt2 = 0.0;

  for (int i = 0; i < samplesPerCycle; i++) {

    if (tempFilt1 < filteredValueArray1[i]) {
      tempFilt1 = filteredValueArray1[i];
    }
    if (tempFilt2 < filteredValueArray2[i]) {
      tempFilt2 = filteredValueArray2[i];
    }
  }

  filtPeak1 = tempFilt1;  // filtPeak1 will hold the peak, we can use these to activate leds VIA threshold
  tempFilt1 = 0.0;        // Re-initialize tempFilt
  //Serial.print(filtPeak1, 1);
  //Serial.print(" ");
  filtPeak2 = tempFilt2;  // filtPeak2 will hold the peak, we can use these to activate leds VIA threshold
  tempFilt2 = 0.0;        // Re-initialize tempFilt
  //Serial.print(filtPeak2, 1);


  if (cycle < 15) {  // 20*25 = 500 = 0.5 second cycle reset, count starts from 0

    if (recordPeak1 < filtPeak1) {
      recordPeak1 = filtPeak1;
    }
    if (recordPeak2 < filtPeak2) {
      recordPeak2 = filtPeak2;
    }

    cycle = cycle + 1;  // increment cycle
  }

  //Serial.print(" ");
  Serial.print(recordPeak1, 1);  // Print highest recorded peak with the 5 second refresh

  Serial.print(" ");
  Serial.print(recordPeak2, 1);  // Print highest recorded peak with the 5 second refresh

  averageProbe = (recordPeak1 + recordPeak2) / 2;
  Serial.print(" ");
  Serial.println(averageProbe, 1);  // Print highest recorded peak with the 5 second refresh

  if (cycle == 15) {
    cycle = 0;  // 20*25 = 500 = 0.5 second cycle reset, for both emg sets
    recordPeak1 = 0.0;
    recordPeak2 = 0.0;
  }
}

void handOpen() {

  for (int j = 10; j < 170; j++)  // goes from 10 degrees to 170 degrees
  {                               // in steps of 1 degree
    ID1.write(170 - j);           // tell servo to go to open position
    ID2.write(j);                 // tell servo to go to open position
    ID3.write(j);                 // tell servo to go to open position
    ID4.write(j);                 // tell servo to go to open position
    ID5.write(j);                 // tell servo to go to open position
    delay(1);                     // waits 15ms for the servo to reach the position
  }
}

void handClose() {

  for (int j = 170; j >= 1; j--)  // goes from 180 degrees to 0 degrees
  {
    ID1.write(170 - j);  // tell servo to go to position in variable 'pos'
    ID2.write(j);        // tell servo to go to position in variable 'pos'
    ID3.write(j);        // tell servo to go to position in variable 'pos'
    ID4.write(j);        // tell servo to go to position in variable 'pos'
    ID5.write(j);        // tell servo to go to position in variable 'pos'
    delay(1);            // waits 15ms for the servo to reach the position
  }
}

void checkPos() {
  if (handTrigger[0] == 0) {
    for (int i = 0; i < 5; i++) {

      if (handControl[0][i] == handControl[1][i]) {

        handControl[2][i] = 0;


      }

      else if (handControl[0] != handControl[1]) {

        handControl[2][i] = 1;
        handControl[1][i] = handControl[0][i];
      }
    }
  }

  else if (handTrigger[0] == 1) {
    if (handTrigger[0] == handTrigger[1]) {

      handTrigger[2] = 0;

    }

    else if (handTrigger[0] != handTrigger[1]) {

      handTrigger[2] = 1;

      handTrigger[1] = handTrigger[0];
    }
  }
}

void activatePos() {


  if (handTrigger[0] == 0) {
    for (int i = 0; i < 5; i++) {

      if (handControl[2][i] == true) {

        switch (handControl[1][i]) {

          case 1:
            fingerOpen(i);
            break;

          case 0:
            fingerClose(i);
            break;
        }

        handControl[2][i] = false;
      }
    }
  }

  else if (handTrigger[0] == 1) {

    if (handTrigger[2] == true) {

      handOpen();

      handTrigger[2] = false;
    }
  }
}

void fingerOpen(int fingerActive) {

  switch (fingerActive) {

    case 0:
      for (int j = 20; j < 170; j++) {
        ID1.write(170 - j);
        delay(1);
      }
      break;

    case 1:
      for (int j = 20; j < 170; j++) {
        ID2.write(j);
        delay(1);
      }
      break;

    case 2:
      for (int j = 20; j < 170; j++) {
        ID3.write(j);
        delay(1);
      }
      break;

    case 3:
      for (int j = 20; j < 170; j++) {
        ID4.write(j);
        delay(1);
      }
      break;

    case 4:
      for (int j = 20; j < 170; j++) {
        ID5.write(j);
        delay(1);
      }
      break;
  }
}

void fingerClose(int fingerActive) {

  switch (fingerActive) {

    case 0:
      for (int j = 160; j >= 10; j--) {
        ID1.write(170 - j);
        delay(1);
      }
      break;

    case 1:
      for (int j = 160; j >= 10; j--) {
        ID2.write(j);
        delay(1);
      }
      break;

    case 2:
      for (int j = 160; j >= 10; j--) {
        ID3.write(j);
        delay(1);
      }
      break;

    case 3:
      for (int j = 160; j >= 10; j--) {
        ID4.write(j);
        delay(1);
      }
      break;

    case 4:
      for (int j = 160; j >= 10; j--) {
        ID5.write(j);
        delay(1);
      }
      break;
  }
}

void engageFinger() {

  if (recordPeak1 > handtriggerthreshold) {
    handTrigger[0] = 1;
    return;
  }

  else if (recordPeak1 <= handtriggerthreshold && handTrigger[1] == 1) {
    handClose();
    handTrigger[0] = 0;
    handTrigger[1] = 0;
    handTrigger[2] = 0;
    return;
  }

  if (recordPeak1 > secondhighestthreshold && recordPeak1 < handtriggerthreshold) {
    handControl[0][2] = 1;


  }

  else if (recordPeak1 < 10) {
    handControl[0][2] = 0;
  }

  if (recordPeak2 > 30 && recordPeak2 < 130) {
    handControl[0][3] = 1;

  }

  else if (recordPeak2 < 10) {
    handControl[0][3] = 0;
  }
}

void loop() {

  while (true) {

    debugMode();  // non-blocking method interrupt

    activatePos();  // this function avoids the need for setup, already initialized
    checkPos();     // checks & updates position, state and activate

    emgSetup();     // Setup for raw input array according to appropriate samples per cycle
    filterState();  // Rectified Filtered and Unfiltered voltage.

    peakDetector();  // Serial print cycle peak detection

    engageFinger();

    delay(25);  // Delay for imaging capture, this will also change the cycle refresh for recordPeak
    
  }
}
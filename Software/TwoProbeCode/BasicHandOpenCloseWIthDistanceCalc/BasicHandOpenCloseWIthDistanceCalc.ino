// #include <Servo.h>
#include <ADC.h>
#include <PWMServo.h>
ADC *adc = new ADC();  // adc object
PWMServo fingerServo[5];
// PWMServo ID1;  // Define all PWMservo motors for hand
// PWMServo ID2;
// PWMServo ID3;
// PWMServo ID4;
// PWMServo ID5;
#define SAMPLESPERCYCLE 1000  // Array samples
#define NUMBEROFSTATES 5
const uint8_t fingerArrangement[NUMBEROFSTATES] = { 0b11110, 0b00010, 0b11101, 0b11111, 0b00001 };  //arranged from pinky-thumb from high to low bit. 1 = open, opposite for thumb
float distance[NUMBEROFSTATES] = { 0, 0, 0, 0, 0 };
float LPStartThreshold1[NUMBEROFSTATES] = { 0, 0, 0, 0, 0 };
float LPStartThreshold2[NUMBEROFSTATES] = { 0, 0, 0, 0, 0 };
float BPStartThreshold1[NUMBEROFSTATES] = { 0, 0, 0, 0, 0 };
float BPStartThreshold2[NUMBEROFSTATES] = { 0, 0, 0, 0, 0 };
float LPIntermediateThreshold1[NUMBEROFSTATES] = { 0, 0, 0, 0, 0 };
float LPIntermediateThreshold2[NUMBEROFSTATES] = { 0, 0, 0, 0, 0 };
float BPIntermediateThreshold1[NUMBEROFSTATES] = { 0, 0, 0, 0, 0 };
float BPIntermediateThreshold2[NUMBEROFSTATES] = { 0, 0, 0, 0, 0 };


// int PWM1 = 0;  // the PWM pin the LED is attached to
// int PWM2 = 1;  // the PWM pin the LED is attached to
// int PWM3 = 2;  // the PWM pin the LED is attached to
// int PWM4 = 3;  // the PWM pin the LED is attached to
// int PWM5 = 4;
// int OutputPWMPins[5] = { PWM1, PWM2, PWM3, PWM4, PWM5 };

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

//bool trigger1 = false; // This is used for the hand open close function
//bool trigger2 = false; // This is used for the hand open close function

const int emgPin1 = A4;  // Analog input pin 18 for the sEMG signal
const int emgPin2 = A3;  // Analog input 17 pin for the sEMG signal


float filteredValueArray1[SAMPLESPERCYCLE];
float filteredValueArray2[SAMPLESPERCYCLE];
float BPFilteredValueArray1[SAMPLESPERCYCLE];
float BPFilteredValueArray2[SAMPLESPERCYCLE];

float rawValueArray1[SAMPLESPERCYCLE];
float rawValueArray2[SAMPLESPERCYCLE];

float LPInactiveThreshold1 = 2 ^ 31;
float LPInactiveThreshold2 = 2 ^ 31;


float recordPeak1 = 0.0;  // Highest recorded peak value
float recordPeak2 = 0.0;
float recordPeakBP1 = 0.0;
float recordPeakBP2 = 0.0;

float averageProbe = 0.0;
int cycle = 0;  // Number cycles elapsed

float dcOffset = 2.006;  // DC voltage offset

float expGain = 4.0;

void setup() {
  Serial.begin(115000);                                             // Initialize serial communication buad rate
  adc->adc0->setAveraging(4);                                       // average 4 together
  adc->adc0->setResolution(10);                                     // 10 bits resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);  // Reference chart for speed. 4.5us per sample in this case
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_MED_SPEED);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW

  fingerServo[0].attach(0);
  fingerServo[0].write(90);  //Initialize each finger
  fingerServo[1].attach(1);
  fingerServo[1].write(90);  //Initialize each finger
  fingerServo[2].attach(2);
  fingerServo[2].write(90);  //Initialize each finger
  fingerServo[3].attach(3);
  fingerServo[3].write(90);  //Initialize each finger
  fingerServo[4].attach(4);
  fingerServo[4].write(90);  //Initialize each finger
}

//Band pass chebyshev filter order=1 alpha1=0.0125 alpha2=0.05
class FilterChBp1 {
public:
  FilterChBp1() {
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;
  }
private:
  float v[3];
public:
  float step(float x)  //class II
  {
    v[0] = v[1];
    v[1] = v[2];
    v[2] = (1.105985005341628691e-1 * x)
           + (-0.78788661454735842149 * v[0])
           + (1.76577240199021168188 * v[1]);
    return (v[2] - v[0]);
  }
};

FilterChBp1 chebyshevBP1;  // First order chebyshev LPF 40kHz BP 500-2kHz
FilterChBp1 chebyshevBP2;  //  First order chebyshev LPF 40kHz BP 500-2kHz

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

  for (int i = 0; i < SAMPLESPERCYCLE; i++) {
    rawValueArray1[i] = analogRead(emgPin1);  // Value is 0-1023
    rawValueArray2[i] = analogRead(emgPin2);  // Value is 0-1023
    delayMicroseconds(16);                    // FILTER NEEDS TO MATCH sample rate. 25uS = 40kHz currently. Anything slower will result in artifacting and distortion
  }
  for (int i = 0; i < SAMPLESPERCYCLE; i++) {
    // filteredValueArray1[i] = pow(2.71828, expGain * abs((chebyshevFilter1.step(rawValueArray1[i] / 1023.0) * 3.3) - dcOffset)) - 1;  // Apply LPF to voltage minus the offset. Then rectify values. e^3.3x - 1. Arbitrary values
    // filteredValueArray2[i] = pow(2.71828, expGain * abs((chebyshevFilter2.step(rawValueArray2[i] / 1023.0) * 3.3) - dcOffset)) - 1;  // Second probe, will possibly require much less gain.
    // BPFilteredValueArray1[i] = pow(2.71828, expGain * abs((chebyshevBP1.step(rawValueArray1[i] / 1023.0) * 3.3) - dcOffset)) - 1;  // Apply LPF to voltage minus the offset. Then rectify values. e^3.3x - 1. Arbitrary values
    // BPFilteredValueArray2[i] = pow(2.71828, expGain * abs((chebyshevBP2.step(rawValueArray2[i] / 1023.0) * 3.3) - dcOffset)) - 1;  // Second probe, will possibly require much less gain.
    filteredValueArray1[i] = abs(chebyshevFilter1.step(rawValueArray1[i] / 1023.0) * 3.3 - dcOffset);  // Apply LPF to voltage minus the offset. Then rectify values. e^3.3x - 1. Arbitrary values
    filteredValueArray2[i] = abs(chebyshevFilter2.step(rawValueArray2[i] / 1023.0) * 3.3 - dcOffset);  // Second probe, will possibly require much less gain.
    BPFilteredValueArray1[i] = abs(chebyshevBP1.step(rawValueArray1[i] / 1023.0) * 3.3);               // Apply LPF to voltage minus the offset. Then rectify values. e^3.3x. Arbitrary values
    BPFilteredValueArray2[i] = abs(chebyshevBP2.step(rawValueArray2[i] / 1023.0) * 3.3);               // Second probe, will possibly require much less gain.
  }
}



void peakDetector() {

  float filtPeak1 = 0.0;
  float filtPeak2 = 0.0;

  float tempFilt1 = 0, tempFilt2 = 0, tempBPFilt1 = 0, tempBPFilt2 = 0;
  for (int i = 0; i < SAMPLESPERCYCLE; i++) {

    if (tempFilt1 < filteredValueArray1[i]) { tempFilt1 = filteredValueArray1[i]; }
    if (tempFilt2 < filteredValueArray2[i]) { tempFilt2 = filteredValueArray2[i]; }
    if (tempBPFilt1 < BPFilteredValueArray1[i]) { tempBPFilt1 = BPFilteredValueArray1[i]; }
    if (tempBPFilt2 < BPFilteredValueArray2[i]) { tempBPFilt2 = BPFilteredValueArray2[i]; }
  }

  // inactiveDistanceLP = sqrt(pow((0 - recordPeak1), 2) + pow((0 - recordPeak2), 2));
  // inactiveDistanceBP = sqrt(pow((0 - recordPeakBP1), 2) + pow((0 - recordPeakBP2), 2));
  // distanceLP = sqrt(pow((LPStartThreshold1[] - recordPeak1), 2) + pow((LPStartThreshold2[] - recordPeak2), 2));
  // distanceBP = sqrt(pow((BPStartThreshold1[] - recordPeakBP1), 2) + pow((BPStartThreshold2[] - recordPeakBP2), 2));
  // intermediateDistanceLP = sqrt(pow((LPIntermediateThreshold1[] - recordPeak1), 2) + pow((LPIntermediateThreshold2[] - recordPeak2), 2));
  // intermediateDistanceBP = sqrt(pow((BPIntermediateThreshold1[] - recordPeakBP1), 2) + pow((BPIntermediateThreshold2[] - recordPeakBP2), 2));

  filtPeak1 = tempFilt1;  // filtPeak1 will hold the peak, we can use these to activate leds VIA threshold
  tempFilt1 = 0.0;        // Re-initialize tempFilt
  // Serial.print(filtPeak1, 1);
  // Serial.print(" ");
  filtPeak2 = tempFilt2;  // filtPeak2 will hold the peak, we can use these to activate leds VIA threshold
  tempFilt2 = 0.0;        // Re-initialize tempFilt
  // Serial.print(filtPeak2, 1);

  if (cycle == 10) {
    cycle = 0;
    recordPeak1 = 0.0;
    recordPeak2 = 0.0;
  }
  if (cycle < 10) {  // 20*25 = 500 = 0.5 second cycle reset, count starts from 0

    if (recordPeak1 < filtPeak1) { recordPeak1 = filtPeak1; }
    if (recordPeak2 < filtPeak2) { recordPeak2 = filtPeak2; }

    cycle = cycle + 1;  // increment cycle
  }

  Serial.print(" ");
  Serial.print(recordPeak1, 1);  // Print highest recorded peak with the 5 second refresh


  Serial.print(" ");
  Serial.print(recordPeak2, 1);  // Print highest recorded peak with the 5 second refresh

  averageProbe = (recordPeak1 + recordPeak2) / 2;
  // Serial.print(" ");
  // Serial.println(averageProbe, 1);  // Print highest recorded peak with the 5 second refresh

  // 20*25 = 500 = 0.5 second cycle reset, for both emg sets
  // Serial.print("stuff \t");  // Print highest recorded peak with the 5 second refresh

  // Serial.print(handControl[2][1], 1);    // Print highest recorded peak with the 5 second refresh
  // Serial.print(handControl[2][2], 1);    // Print highest recorded peak with the 5 second refresh
  // Serial.print(handControl[2][3], 1);  // Print highest recorded peak with the 5 second refresh
  Serial.print("\n");
}

void handOpen() {

  for (int j = 10; j < 170; j++)    // goes from 10 degrees to 170 degrees
  {                                 // in steps of 1 degree
    fingerServo[0].write(170 - j);  // tell servo to go to open position
    fingerServo[1].write(j);        // tell servo to go to open position
    fingerServo[2].write(j);        // tell servo to go to open position
    fingerServo[3].write(j);        // tell servo to go to open position
    fingerServo[4].write(j);        // tell servo to go to open position
    delay(4);                       // waits 15ms for the servo to reach the position
  }
}

void handClose() {

  for (int j = 170; j >= 1; j--)  // goes from 180 degrees to 0 degrees
  {
    fingerServo[0].write(170 - j);  // tell servo to go to position in variable 'pos'
    fingerServo[1].write(j);        // tell servo to go to position in variable 'pos'
    fingerServo[2].write(j);        // tell servo to go to position in variable 'pos'
    fingerServo[3].write(j);        // tell servo to go to position in variable 'pos'
    fingerServo[4].write(j);        // tell servo to go to position in variable 'pos'
    delay(4);                       // waits 15ms for the servo to reach the position
  }
}

void checkPos() {

  // sqrt(pow((500 - recordPeak1), 2) + pow((500 - recordPeak2), 2)),
  // sqrt(pow((250 - recordPeak1), 2) + pow((250 - recordPeak2), 2)),
  // sqrt(pow((0 - recordPeak1), 2) + pow((0 - recordPeak2), 2))
  // distance = {
  //   sqrt(pow((2.5 - recordPeak1), 2)),
  //   sqrt(pow((10 - recordPeak1), 2)),
  //   sqrt(pow((100 - recordPeak1), 2))
  // };


  distance[0] = sqrt(pow((0 - recordPeak1), 2) + pow((0 - recordPeak2), 2));
  distance[1] = sqrt(pow((2 - recordPeak1), 2) + pow((2 - recordPeak2), 2));
  distance[2] = sqrt(pow((7.5 - recordPeak1), 2) + pow((7.5 - recordPeak2), 2));
  if (false) {
    Serial.print(" ");
    Serial.print(distance[0], 1);  // Print highest recorded peak with the 5 second refresh
    Serial.print(" ");
    Serial.print(distance[1], 1);  // Print highest recorded peak with the 5 second refresh
    Serial.print(" ");
    Serial.print(distance[2], 1);  // Print highest recorded peak with the 5 second refresh
  }
  int smallestindex = NUMBEROFSTATES;
  float smallest = pow(2, 31);
  for (int i = 0; i < NUMBEROFSTATES; i++) {
    if (distance[i] < smallest) {
      smallestindex = i;
      smallest = distance[i];
      Serial.print("new smallest\t ");
      Serial.print(smallest);
    }
  }
  Serial.print("smallest index \t ");
  Serial.println(smallestindex);

  // Serial.println(smallestindex);
  switch (smallestindex) {
    case 0:
      Serial.println("0");
      handControl[2][0] = 1;
      handControl[2][1] = 1;
      handControl[2][2] = 1;
      handControl[2][3] = 1;
      handControl[2][4] = 1;
      break;
    case 1:
      break;
    case 2:
      Serial.println("2");
      handControl[2][0] = 0;
      handControl[2][1] = 0;
      handControl[2][2] = 0;
      handControl[2][3] = 0;
      handControl[2][4] = 0;
      break;
    case 3:
      // Serial.println("1");
      // handControl[2][0] = 0;
      // handControl[2][1] = 0;
      // handControl[2][2] = 0;
      // handControl[2][3] = 0;
      // handControl[2][4] = 0;
      break;
    case 4:
      Serial.println("1");
      // handControl[2][0] = 0;
      // handControl[2][1] = 0;
      // handControl[2][2] = 0;
      // handControl[2][3] = 0;
      // handControl[2][4] = 0;
      break;
    default:
      Serial.println("foo");
      // handControl[2][0] = 0;
      // handControl[2][1] = 0;
      // handControl[2][2] = 0;
      // handControl[2][3] = 0;
      // handControl[2][4] = 0;
      break;
  }
}

// void activatePos() {

//   for (int fingeriteration = 0; fingeriteration < 5; fingeriteration++) {
//     if (handControl[2][fingeriteration]) {
//       if (handControl[0][fingeriteration] == 0) {
//         fingerOpen(fingeriteration);
//         handControl[0][fingeriteration] = 1;
//       }
//     } else {
//       if (handControl[0][fingeriteration] == 1) {
//         fingerClose(fingeriteration);
//         handControl[0][fingeriteration] = 0;
//       }
//     }
//   }
// }

void ExecuteFingerPositions(uint8_t positions)  //input should be 5 bits, high bit is pinky low bit is thumb. Remember thumb active is invers of rest of fingers. Close ==0b00001, Open == 0b11110
{
  bool unfinished = true;
  while (unfinished) {
    unfinished = false;  //assume everything will return as done, when something realizes it has to activate it will cause the loop to retrigger.
    for (int fingeriterator = 0; fingeriterator < 5; fingeriterator++) {
      bool tempfingeractive = 0b1 & (positions >> fingeriterator);  //is the finger supposed to be active?

      int tempcurrentangle = fingerServo[fingeriterator].read();  //get the current pwmangle of the finger
      if (tempfingeractive && tempcurrentangle < 150) {           //is the finger active and not at max angle?
        fingerServo[fingeriterator].write(tempcurrentangle + 10);
        unfinished = true;
      } else if ((!tempfingeractive) && tempcurrentangle > 30) {  //is the finger inactive and not a min angle?
        fingerServo[fingeriterator].write(tempcurrentangle - 10);
        unfinished = true;
      }
      if (false) {
        Serial.print("\n finger ");
        Serial.print(fingeriterator);
        Serial.print(" ");
        Serial.print(tempfingeractive);
        Serial.println(positions);
      }
    }
    delay(20);
  }
}


void ThresholdInit() {
  // Serial.print(LPInactiveThreshold1);
  // Serial.print("\t");
  // Serial.println(LPInactiveThreshold1);


  for (int positioniterator = 1; positioniterator < NUMBEROFSTATES; positioniterator++) {
    ExecuteFingerPositions(0b11110);  //open up hand
    // Serial.print("When LED turns on, Flex and hold finger\n");
    delay(1000);
    ExecuteFingerPositions(fingerArrangement[positioniterator]);                                                                                                                           //move hand into desired position
    float tempFilt1 = 0, tempFilt2 = 0, tempBPFilt1 = 0, tempBPFilt2 = 0, tempIntermediateFilt1 = 0, tempIntermediateFilt2 = 0, tempIntermediateBPFilt1 = 0, tempIntermediateBPFilt2 = 0;  //initialize values
    uint numberOfIntermediateSamples = 0;                                                                                                                                                  //used to average intermediate readings
    for (int checkthresholditerator = 0; checkthresholditerator < (int)5 * 40000 / SAMPLESPERCYCLE; checkthresholditerator++) {                                                            //for 5 seconds
      emgSetup();                                                                                                                                                                          //take a reading
      numberOfIntermediateSamples++;                                                                                                                                                       //increase iterator indicating how many samples have been summed to average intermediatevalues
      for (int sampleiterator = 0; sampleiterator < SAMPLESPERCYCLE; sampleiterator++) {                                                                                                   //go through each reading of current window

        if (tempFilt1 < filteredValueArray1[sampleiterator] || tempFilt2 < filteredValueArray2[sampleiterator]) {  //if either probe has a absolute maximum
          tempFilt1 = filteredValueArray1[sampleiterator];                                                         //update each activation level
          tempFilt2 = filteredValueArray2[sampleiterator];
          tempBPFilt1 = BPFilteredValueArray1[sampleiterator];
          tempBPFilt2 = BPFilteredValueArray2[sampleiterator];
          tempIntermediateFilt1 = 0;  //reset intermediate value
          tempIntermediateFilt2 = 0;
          tempIntermediateBPFilt1 = 0;
          tempIntermediateBPFilt2 = 0;
          numberOfIntermediateSamples = 0;  //reset number of values in intermediate value
        }
      }
      if (numberOfIntermediateSamples > 0) {  //starting on frame after maximum, start summing local maximums into appropriate variables
        tempIntermediateFilt1 += tempFilt1;   //sum up values as
        tempIntermediateFilt2 += tempFilt2;
        tempIntermediateBPFilt1 += tempBPFilt1;
        tempIntermediateBPFilt2 += tempBPFilt2;
      }
      LPStartThreshold1[positioniterator] = tempFilt1;  //store absolute value of activation
      LPStartThreshold2[positioniterator] = tempFilt2;
      BPStartThreshold1[positioniterator] = tempBPFilt1;
      BPStartThreshold2[positioniterator] = tempBPFilt2;
      LPIntermediateThreshold1[positioniterator] = tempIntermediateFilt1 / numberOfIntermediateSamples;  //store held value by averaging local maximums of each frame after activation
      LPIntermediateThreshold2[positioniterator] = tempIntermediateFilt2 / numberOfIntermediateSamples;
      BPIntermediateThreshold1[positioniterator] = tempIntermediateBPFilt1 / numberOfIntermediateSamples;
      BPIntermediateThreshold2[positioniterator] = tempIntermediateBPFilt2 / numberOfIntermediateSamples;

      //acquire
      //check if a local maximum is reached from any input
      //save it as new input
      //continue until finished
    }
  }

  if (true) {
    for (int positioniterator = 0; positioniterator < NUMBEROFSTATES; positioniterator++) {
      Serial.print("LPStartThreshold1[");
      Serial.print(positioniterator);
      Serial.print("]: ");
      Serial.println(LPStartThreshold1[positioniterator]);

      Serial.print("LPStartThreshold2[");
      Serial.print(positioniterator);
      Serial.print("]: ");
      Serial.println(LPStartThreshold2[positioniterator]);

      Serial.print("BPStartThreshold1[");
      Serial.print(positioniterator);
      Serial.print("]: ");
      Serial.println(BPStartThreshold1[positioniterator]);

      Serial.print("BPStartThreshold2[");
      Serial.print(positioniterator);
      Serial.print("]: ");
      Serial.println(BPStartThreshold2[positioniterator]);

      Serial.print("LPIntermediateThreshold1[");
      Serial.print(positioniterator);
      Serial.print("]: ");
      Serial.println(LPIntermediateThreshold1[positioniterator]);

      Serial.print("LPIntermediateThreshold2[");
      Serial.print(positioniterator);
      Serial.print("]: ");
      Serial.println(LPIntermediateThreshold2[positioniterator]);

      Serial.print("BPIntermediateThreshold1[");
      Serial.print(positioniterator);
      Serial.print("]: ");
      Serial.println(BPIntermediateThreshold1[positioniterator]);

      Serial.print("BPIntermediateThreshold2[");
      Serial.print(positioniterator);
      Serial.print("]: ");
      Serial.println(BPIntermediateThreshold2[positioniterator]);
    }
  }
}




void loop() {
  while (true) {
    ThresholdInit();
    ExecuteFingerPositions(0b11110);  //open up hand
      delay(10000000);
  }
  while (true) {



    emgSetup();  // Setup for raw input array according to appropriate samples per cycle

    peakDetector();  // Serial print cycle peak detection
    checkPos();      // checks & updates position, state and activate

    delay(25);  // Delay for imaging capture, this will also change the cycle refresh for recordPeak
  }
}
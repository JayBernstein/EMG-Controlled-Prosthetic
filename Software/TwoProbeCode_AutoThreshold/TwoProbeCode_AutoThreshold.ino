#include <ADC.h>
ADC *adc = new ADC();    // adc object
const int emgPin1 = A4;  // Analog input pin for the sEMG signal
const int emgPin2 = A3;  // Analog input pin for the sEMG signal
const int dcOffsetPin = A2;
float dcOffset = 1.622;  // DC voltage offset
const int readingsInWindow = 10;
float totalOfWindow1 = 0;
float totalOfWindow2 = 0;
float Window1[readingsInWindow];
float Window2[readingsInWindow];

const int samplesPerCycle = 1000;  // Array samples
int PWM1 = 0;                      // the PWM pin the LED is attached to
int PWM2 = 1;                      // the PWM pin the LED is attached to
int PWM3 = 2;                      // the PWM pin the LED is attached to
int PWM4 = 3;                      // the PWM pin the LED is attached to
int PWM5 = 4;
int OutputPWMPins[5] = { PWM1, PWM2, PWM3, PWM4, PWM5 };

//Array For raw and manipulated analog readings

float normalizedValueArray1[samplesPerCycle];
float filteredValueArray1[samplesPerCycle];

float normalizedValueArray2[samplesPerCycle];
float filteredValueArray2[samplesPerCycle];

int rawValueArray1[samplesPerCycle];
int rawValueArray2[samplesPerCycle];

float rollingAverageFilteredArray1[samplesPerCycle];
float rollingAverageFilteredArray2[samplesPerCycle];


//
// Threshold Arrays
float LPInactiveThreshold1 = 2 ^ 31;
float LPInactiveThreshold2 = 2 ^ 31;
float LPStartThreshold1[5] = { 0, 0, 0, 0, 0 };  //Starting from index 0 corresponds to Thumb, index, middle, ring, pinky
float LPStartThreshold2[5] = { 0, 0, 0, 0, 0 };
float LPActiveThreshold1[5] = { 0, 0, 0, 0, 0 };
float LPActiveThreshold2[5] = { 0, 0, 0, 0, 0 };


//


int cycle = 0;  // Number cycles elapsed

float recordPeak1 = 0.0;  // Highest recorded peak value
float recordPeak2 = 0.0;

float averageProbe = 0.0;

//
//Filter Init

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

FilterChLp1 chebyshevFilter;   // First order chebyshev LPF 40kHz at a 1500Hz low corner cut off
FilterChLp1 chebyshevFilter2;  // First order chebyshev LPF 40kHz at a 1500Hz low corner cut off

//

void setup() {

  adc->adc0->setAveraging(4);                                       // average 4 together
  adc->adc0->setResolution(10);                                     // 10 bits resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);  // Reference chart for speed. 4.5us per sample in this case
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_MED_SPEED);

  pinMode(emgPin1, INPUT);
  pinMode(emgPin2, INPUT);
  pinMode(dcOffsetPin, INPUT);
  Serial.begin(115000);  // Initialize serial communication buad rate

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW


  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(PWM4, OUTPUT);
  pinMode(PWM5, OUTPUT);
  digitalWrite(PWM1, HIGH);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(PWM2, HIGH);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(PWM3, HIGH);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(PWM4, HIGH);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(PWM5, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);               // wait for a second
  digitalWrite(PWM1, LOW);   // turn the LED on (LOW is the voltage level)
  digitalWrite(PWM2, LOW);   // turn the LED on (LOW is the voltage level)
  digitalWrite(PWM3, LOW);   // turn the LED on (LOW is the voltage level)
  digitalWrite(PWM4, LOW);   // turn the LED on (LOW is the voltage level)
  digitalWrite(PWM5, LOW);   // turn the LED on (LOW is the voltage level)

  if (true) {
    dcOffset = analogRead(dcOffsetPin) * 3.3 / 1023;  //only applicable in 10 bit mode
    Serial.println(dcOffset);
  }
}




void emgSetup() {

  for (int i = 0; i < samplesPerCycle; i++) {
    rawValueArray1[i] = analogRead(emgPin1);  // Value is 0-1023
    rawValueArray2[i] = analogRead(emgPin2);  // Value is 0-1023
    delayMicroseconds(21);                    // FILTER NEEDS TO MATCH sample rate. 25uS = 40kHz currently. Anything slower will result in artifacting and distortion
    //set to 21 becuase sampling rate is 2.04us, and there are two inputs. 25 us total for each cycle. Sampling Frequency Caluclated to be 39808.91719745223 Hz
  }
}

void filterState() {
  for (int tempiterator = 0; tempiterator < readingsInWindow; tempiterator++) {
    Window1[tempiterator] = 0;
    Window2[tempiterator] = 0;
  }
  int index = 0;
  totalOfWindow1 = 0;
  totalOfWindow2 = 0;

  for (int i = 0; i < samplesPerCycle; i++) {

    normalizedValueArray1[i] = rawValueArray1[i] / 1023.0;  // This is the normalized raw value array from 0.0 to 1.0
    normalizedValueArray2[i] = rawValueArray2[i] / 1023.0;

    // filteredValueArray1[i] = abs((chebyshevFilter.step(normalizedValueArray1[i]) * 3.3) - 2);   // Apply LPF to voltage minus the offset. Then rectify values. e^3.3x - 1. Arbitrary values
    // filteredValueArray2[i] = abs((chebyshevFilter2.step(normalizedValueArray2[i]) * 3.3) - 2);  // Second probe, will possibly require much less gain.
    filteredValueArray1[i] = pow(2.71828, 3.3 * abs((chebyshevFilter.step(normalizedValueArray1[i]) * 3.3) - dcOffset)) - 1;   // Apply LPF to voltage minus the offset. Then rectify values. e^3.3x - 1. Arbitrary values
    filteredValueArray2[i] = pow(2.71828, 3.3 * abs((chebyshevFilter2.step(normalizedValueArray2[i]) * 3.3) - dcOffset)) - 1;  // Second probe, will possibly require much less gain.

    totalOfWindow1 -= Window1[index];  // Subtract the oldest reading from the total
    totalOfWindow2 -= Window2[index];
    // Serial.print(totalOfWindow2);
    // Serial.print("\t");

    Window1[index] = filteredValueArray1[i];  // Store the new reading in the array
    Window2[index] = filteredValueArray2[i];
    totalOfWindow1 += filteredValueArray1[i];  // Add the new reading to the total
    totalOfWindow2 += filteredValueArray2[i];
    index = (index + 1) % readingsInWindow;
    if (index == 0)
      // Serial.println();

      // Move to the next index
      rollingAverageFilteredArray1[i] = totalOfWindow1 / readingsInWindow;  // Calculate the moving average of rawValue
    rollingAverageFilteredArray2[i] = totalOfWindow2 / readingsInWindow;
  }
}

void peakDetector() {

  float filtPeak1 = 0.0;
  float filtPeak2 = 0.0;

  float tempFilt1 = 0.0;
  float tempFilt2 = 0.0;
  for (int i = 0; i < samplesPerCycle; i++) {
    if (tempFilt1 < rollingAverageFilteredArray1[i]) { tempFilt1 = rollingAverageFilteredArray1[i]; }
    if (tempFilt2 < rollingAverageFilteredArray2[i]) { tempFilt2 = rollingAverageFilteredArray2[i]; }
  }
  Serial.print(tempFilt1);
  Serial.print("\t");
  Serial.println(tempFilt2);

  float distance[6] = {
    sqrt(pow((LPStartThreshold1[0] - tempFilt1), 2) + pow((LPStartThreshold2[0] - tempFilt2), 2)),
    sqrt(pow((LPStartThreshold1[1] - tempFilt1), 2) + pow((LPStartThreshold2[1] - tempFilt2), 2)),
    sqrt(pow((LPStartThreshold1[2] - tempFilt1), 2) + pow((LPStartThreshold2[2] - tempFilt2), 2)),
    sqrt(pow((LPStartThreshold1[3] - tempFilt1), 2) + pow((LPStartThreshold2[3] - tempFilt2), 2)),
    sqrt(pow((LPStartThreshold1[4] - tempFilt1), 2) + pow((LPStartThreshold2[4] - tempFilt2), 2)),
    sqrt(pow((LPInactiveThreshold1 - tempFilt1), 2) + pow((LPInactiveThreshold2 - tempFilt2), 2))
  };
  for (int it = 0; it < 5; it++) {
  distance[it] =sqrt(pow((LPStartThreshold1[it] - tempFilt1), 2) + pow((LPStartThreshold2[it] - tempFilt2), 2));
  Serial.print(distance[it]);
  Serial.print("\t");
  Serial.print(tempFilt1);
  Serial.print(" ");
  Serial.print(LPStartThreshold1[it]);
  Serial.print("\t");
  Serial.print(tempFilt2);
  Serial.print(" ");
  Serial.print(LPStartThreshold2[it]);
  Serial.print("\n");
  }
  
  int smallestindex = 6;
  float smallest = pow(2, 31);
  for (int i = 0; i < 6; i++) {
    if (distance[i] < smallest) {
      smallestindex = i;
      smallest = distance[i];
    }
  }
      Serial.println(smallestindex);

  if (smallestindex == 5) {
    for (int i = 0; i < 5; i++) {
      digitalWrite(OutputPWMPins[i], LOW);  // turn the LED on (HIGH is the voltage level)
    }
  } else {
    for (int i = 0; i < 5; i++) {
      digitalWrite(OutputPWMPins[i], LOW);  // turn the LED on (HIGH is the voltage level)
    }
    digitalWrite(OutputPWMPins[smallestindex], HIGH);  // turn the LED on (HIGH is the voltage level)
  }


  filtPeak1 = tempFilt1;  // filtPeak1 will hold the peak, we can use these to activate leds VIA threshold
  tempFilt1 = 0.0;        // Re-initialize tempFilt

  filtPeak2 = tempFilt2;  // filtPeak2 will hold the peak, we can use these to activate leds VIA threshold
  tempFilt2 = 0.0;        // Re-initialize tempFilt



  if (cycle < 10) {  // 20*25 = 500 = 0.5 second cycle reset, count starts from 0

    if (recordPeak1 < filtPeak1) { recordPeak1 = filtPeak1; }
    if (recordPeak2 < filtPeak2) { recordPeak2 = filtPeak2; }

    cycle = cycle + 1;  // increment cycle
  }
  averageProbe = (recordPeak1 + recordPeak2) / 2;


  if (false) {
    Serial.print(filtPeak1, 1);
    Serial.print(" ");
    Serial.print(filtPeak2, 1);
    Serial.print(" ");
    Serial.print(recordPeak1, 1);  // Print highest recorded peak with the 5 second refresh

    Serial.print(" ");
    Serial.print(recordPeak2, 1);  // Print highest recorded peak with the 5 second refresh


    Serial.print(" ");
    Serial.println(averageProbe, 1);  // Print highest recorded peak with the 5 second refresh
  }


  if (cycle == 10) {
    cycle = 0;
    recordPeak1 = 0.0;
    recordPeak2 = 0.0;
    // for (int i = 0; i < 6; i++) {
    //   Serial.print(distance[i]);
    //   Serial.print("\t");
    // }
    //   Serial.print("\n");

  }  // 20*25 = 500 = 0.5 second cycle reset, for both emg sets

  // set thresholds here to activate led
}


bool ThresholdInit() {

  Serial.print("Keep arm and hand still and unflexed\n Pay attention for further instructions\n An LED will light up when new instructions are available\n");
  delay(1000);
  for (int tempiterator = 0; tempiterator < 3 * 40000 / samplesPerCycle; tempiterator++) {
    emgSetup();     // Setup for raw input array according to appropriate samples per cycle
    filterState();  // Rectified Filtered and Unfiltered voltage.
    for (int i = 0; i < samplesPerCycle; i++) {
      // if (LPInactiveThreshold1 > filteredValueArray1[i]) { LPInactiveThreshold1 = filteredValueArray1[i]; }
      LPInactiveThreshold1 = (LPInactiveThreshold1 + filteredValueArray1[i]) / 2;
      LPInactiveThreshold2 = (LPInactiveThreshold2 + filteredValueArray2[i]) / 2;
      // if (LPInactiveThreshold2 > filteredValueArray2[i]) { LPInactiveThreshold2 = filteredValueArray2[i]; }
    }
  }
  Serial.print(LPInactiveThreshold1);
  Serial.print("\t");
  Serial.println(LPInactiveThreshold1);


  for (int i = 0; i < 5; i++) {

    Serial.print("When LED turns on, Flex and hold finger\t");
    Serial.println(i);
    delay(1000);


    for (int tempiterator = 0; tempiterator < 3 * 40000 / samplesPerCycle; tempiterator++) {
    digitalWrite(OutputPWMPins[i], HIGH);  // turn the LED on (HIGH is the voltage level)

      emgSetup();     // Setup for raw input array according to appropriate samples per cycle
digitalWrite(OutputPWMPins[i], LOW); 

      filterState();  // Rectified Filtered and Unfiltered voltage.
      bool triggerAffirm = false;
      for (int j = 0; j < samplesPerCycle; j++) {
        if (LPStartThreshold1[i] < rollingAverageFilteredArray1[j]) { LPStartThreshold1[i] = rollingAverageFilteredArray1[j]; }
        if (LPStartThreshold2[i] < rollingAverageFilteredArray2[j]) { LPStartThreshold2[i] = rollingAverageFilteredArray2[j]; }
        // if (LPStartThreshold1[i] < filteredValueArray1[j] || LPStartThreshold2[i] < filteredValueArray2[j]) {
        // LPStartThreshold1[i] = filteredValueArray1[j];  //not sure if I should split the max readings or have them initialize together
        // LPStartThreshold2[i] = filteredValueArray2[j];
        // triggerAffirm = true;
        // } else if (triggerAffirm) {
        //   LPActiveThreshold1[i] = filteredValueArray1[j];
        //   LPActiveThreshold2[i] = filteredValueArray2[j];
        //   // triggerAffirm = false;
        // }
      }
    }
    Serial.print(LPStartThreshold1[i]);
    Serial.print("\t");
    Serial.println(LPStartThreshold2[i]);
    digitalWrite(OutputPWMPins[i], LOW);  // turn the LED on (HIGH is the voltage level)
    Serial.print("Please Relax Hand\n");
    delay(1000);
  }

  /*
  Serial.print("Testing will begin in 5 seconds. Please flex corresponding finger when prompted\n");
  int issues = 0;
  for (int fingertest = 0; fingertest < 5; fingertest++) {
    emgSetup();     // Setup for raw input array according to appropriate samples per cycle
    filterState();  // Rectified Filtered and Unfiltered voltage.
    float tempFilt1 = 0.0;
    float tempFilt2 = 0.0;
    for (int i = 0; i < samplesPerCycle; i++) {
      if (tempFilt1 < filteredValueArray1[i]) { tempFilt1 = filteredValueArray1[i]; }
      if (tempFilt2 < filteredValueArray2[i]) { tempFilt2 = filteredValueArray2[i]; }
    }
    float distance[6] = {
      sqrt((tempFilt1 - LPStartThreshold1[0]) * (tempFilt1 - LPStartThreshold1[0]) + (tempFilt2 - LPStartThreshold2[0]) * (tempFilt2 - LPStartThreshold2[0])),
      sqrt((tempFilt1 - LPStartThreshold1[1]) * (tempFilt1 - LPStartThreshold1[1]) + (tempFilt2 - LPStartThreshold2[1]) * (tempFilt2 - LPStartThreshold2[1])),
      sqrt((tempFilt1 - LPStartThreshold1[2]) * (tempFilt1 - LPStartThreshold1[2]) + (tempFilt2 - LPStartThreshold2[2]) * (tempFilt2 - LPStartThreshold2[2])),
      sqrt((tempFilt1 - LPStartThreshold1[3]) * (tempFilt1 - LPStartThreshold1[3]) + (tempFilt2 - LPStartThreshold2[3]) * (tempFilt2 - LPStartThreshold2[3])),
      sqrt((tempFilt1 - LPStartThreshold1[4]) * (tempFilt1 - LPStartThreshold1[4]) + (tempFilt2 - LPStartThreshold2[4]) * (tempFilt2 - LPStartThreshold2[4])),
      sqrt((tempFilt1 - LPInactiveThreshold1) * (tempFilt1 - LPInactiveThreshold1) + (tempFilt2 - LPInactiveThreshold2) * (tempFilt2 - LPInactiveThreshold2)),
    };

    int smallestindex = 2 ^ 31;
    float smallest = 2 ^ 31;
    for (int i = 0; i < 6; i++) {
      if (distance[i] < smallest) {
        smallestindex = i;
        smallest = distance[i];
      }
    }
    if (smallestindex == 5) {
      for (int i = 0; i < 5; i++) {
        digitalWrite(OutputPWMPins[i], LOW);  // turn the LED on (HIGH is the voltage level)
      }
    } else {
      for (int i = 0; i < 5; i++) {
        digitalWrite(OutputPWMPins[i], LOW);  // turn the LED on (HIGH is the voltage level)
      }
      digitalWrite(OutputPWMPins[smallestindex], HIGH);  // turn the LED on (HIGH is the voltage level)
    }
  }

*/
  return true;
}


bool flipflop = true;
void loop() {
  while (!ThresholdInit())
    ;
  while (true) {

    emgSetup();      // Setup for raw input array according to appropriate samples per cycle
    filterState();   // Rectified Filtered and Unfiltered voltage.
    peakDetector();  // Serial print cycle peak detection



    flipflop = !flipflop;
    digitalWrite(LED_BUILTIN, flipflop);  // turn the LED on (HIGH is the voltage level)

    delay(750);  // Delay for imaging capture, this will also change the cycle refresh for recordPeak
  }
}
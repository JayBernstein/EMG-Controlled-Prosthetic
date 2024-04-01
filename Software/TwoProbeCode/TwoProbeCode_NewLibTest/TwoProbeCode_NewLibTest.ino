#include <ADC.h>
#include <ADC_util.h>
ADC *adc = new ADC();              // adc object
const int emgPin1 = A4;            // Analog input pin for the sEMG signal
const int emgPin2 = A3;            // Analog input pin for the sEMG signal
const int samplesPerCycle = 1000;  // Array samples
const int readingsInWindow = 100;
float totalOfWindow1 = 0;
float totalOfWindow2 = 0;
float Window1[readingsInWindow];
float Window2[readingsInWindow];


float normalizedValueArray1[samplesPerCycle];
float filteredValueArray1[samplesPerCycle];

float normalizedValueArray2[samplesPerCycle];
float filteredValueArray2[samplesPerCycle];

int rawValueArray1[samplesPerCycle];
int rawValueArray2[samplesPerCycle];

float rollingAverageFilteredArray1[samplesPerCycle];
float rollingAverageFilteredArray2[samplesPerCycle];

int cycle = 0;  // Number cycles elapsed

float dcOffset = 2.005;   // DC voltage offset
float recordPeak1 = 0.0;  // Highest recorded peak value
float recordPeak2 = 0.0;

float averageProbe = 0.0;

void setup() {
  Serial.begin(115000);                                             // Initialize serial communication buad rate
  adc->adc0->setAveraging(4);                                       // no averaging
  adc->adc0->setResolution(10);                                     // 10 bits resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);  // Reference chart for speed. 4.5us per sample in this case
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_MED_SPEED);
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


FilterChLp1 chebyshevFilter;   // First order chebyshev LPF 40kHz at a 1500Hz low corner cut off
FilterChLp1 chebyshevFilter2;  // First order chebyshev LPF 40kHz at a 1500Hz low corner cut off

void emgSetup() {

  for (int i = 0; i < samplesPerCycle; i++) {
    rawValueArray1[i] = analogRead(emgPin1);  // Value is 0-1023
    rawValueArray2[i] = analogRead(emgPin2);  // Value is 0-1023
    delayMicroseconds(16);                    // FILTER NEEDS TO MATCH sample rate. 25uS = 40kHz currently. Anything slower will result in artifacting and distortion
  }
}

void filterState() {
  int index = 0;
  for (int tempiterator = 0; tempiterator < readingsInWindow; tempiterator++) {
     Window1[tempiterator] = 0;
     Window2[tempiterator] = 0;
  }
  for (int i = 0; i < samplesPerCycle; i++) {

    normalizedValueArray1[i] = rawValueArray1[i] / 1023.0;  // This is the normalized raw value array from 0.0 to 1.0
    normalizedValueArray2[i] = rawValueArray2[i] / 1023.0;

    filteredValueArray1[i] = pow(2.71828, 3.3 * abs((chebyshevFilter.step(normalizedValueArray1[i]) * 3.3) - dcOffset)) - 1;   // Apply LPF to voltage minus the offset. Then rectify values. e^3.3x - 1. Arbitrary values
    filteredValueArray2[i] = pow(2.71828, 3.3 * abs((chebyshevFilter2.step(normalizedValueArray2[i]) * 3.3) - dcOffset)) - 1;  // Second probe, will possibly require much less gain.

    totalOfWindow1 -= Window1[index];  // Subtract the oldest reading from the total
    totalOfWindow2 -= Window2[index];
    Window1[index] = filteredValueArray1[i];  // Store the new reading in the array
    Window2[index] = filteredValueArray2[i];
    totalOfWindow1 += filteredValueArray1[i];  // Add the new reading to the total
    totalOfWindow2 += filteredValueArray2[i];
    index = (index + 1) % readingsInWindow;                               // Move to the next index
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

    if (tempFilt1 < filteredValueArray1[i]) { tempFilt1 = filteredValueArray1[i]; }
    if (tempFilt2 < filteredValueArray2[i]) { tempFilt2 = filteredValueArray2[i]; }
  }



  filtPeak1 = tempFilt1;  // filtPeak1 will hold the peak, we can use these to activate leds VIA threshold
  tempFilt1 = 0.0;        // Re-initialize tempFilt
  Serial.print(filtPeak1, 1);
  Serial.print(" ");
  filtPeak2 = tempFilt2;  // filtPeak2 will hold the peak, we can use these to activate leds VIA threshold
  tempFilt2 = 0.0;        // Re-initialize tempFilt
  Serial.print(filtPeak2, 1);


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
  Serial.print(" ");
  Serial.println(averageProbe, 1);  // Print highest recorded peak with the 5 second refresh



  if (cycle == 10) {
    cycle = 0;
    recordPeak1 = 0.0;
    recordPeak2 = 0.0;
  }  // 20*25 = 500 = 0.5 second cycle reset, for both emg sets

  // set thresholds here to activate led
}


void loop() {

  while (true) {

    emgSetup();     // Setup for raw input array according to appropriate samples per cycle
    filterState();  // Rectified Filtered and Unfiltered voltage.

    peakDetector();  // Serial print cycle peak detection

    delay(25);  // Delay for imaging capture, this will also change the cycle refresh for recordPeak
  }
}
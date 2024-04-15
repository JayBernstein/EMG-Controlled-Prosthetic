// #include <Servo.h>
#include <ADC.h>
#include <PWMServo.h>
ADC *adc = new ADC();  // adc object

PWMServo ID1;  // Define all PWMservo motors for hand
PWMServo ID2;
PWMServo ID3;
PWMServo ID4;
PWMServo ID5;
#define distanceIndexMax 5
float distance[distanceIndexMax] = { 0, 0, 0, 0, 0};

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

const int emgPin1 = A4;            // Analog input pin 18 for the sEMG signal
const int emgPin2 = A3;            // Analog input 17 pin for the sEMG signal
const int samplesPerCycle = 3000;  // Array samples

float filteredValueArray1[samplesPerCycle];

float filteredValueArray2[samplesPerCycle];

float rawValueArray1[samplesPerCycle];
float rawValueArray2[samplesPerCycle];

float LPInactiveThreshold1 = 2 ^ 31;
float LPInactiveThreshold2 = 2 ^ 31;
float LPStartThreshold1[5] = { 0, 0, 0, 0, 0 };  //Starting from index 0 corresponds to Thumb, index, middle, ring, pinky
float LPStartThreshold2[5] = { 0, 0, 0, 0, 0 };

float recordPeak1 = 0.0;  // Highest recorded peak value
float recordPeak2 = 0.0;

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

  ID1.attach(0);  //23
  ID2.attach(1);
  ID3.attach(2);
  ID4.attach(3);
  ID5.attach(4);
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

void filterState() {

  for (int i = 0; i < samplesPerCycle; i++) {


    filteredValueArray1[i] = pow(2.71828, expGain * abs((chebyshevFilter1.step(rawValueArray1[i] / 1023.0) * 3.3) - dcOffset)) - 1;  // Apply LPF to voltage minus the offset. Then rectify values. e^3.3x - 1. Arbitrary values
    filteredValueArray2[i] = pow(2.71828, expGain * abs((chebyshevFilter2.step(rawValueArray2[i] / 1023.0) * 3.3) - dcOffset)) - 1;  // Second probe, will possibly require much less gain.
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
  // Serial.print(filtPeak1, 1);
  // Serial.print(" ");
  filtPeak2 = tempFilt2;  // filtPeak2 will hold the peak, we can use these to activate leds VIA threshold
  tempFilt2 = 0.0;        // Re-initialize tempFilt
  // Serial.print(filtPeak2, 1);


  if (cycle < 10) {  // 20*25 = 500 = 0.5 second cycle reset, count starts from 0

    if (recordPeak1 < filtPeak1) { recordPeak1 = filtPeak1; }
    if (recordPeak2 < filtPeak2) { recordPeak2 = filtPeak2; }

    cycle = cycle + 1;  // increment cycle
  }

  Serial.print(" ");
  Serial.print(recordPeak1, 1);  // Print highest recorded peak with the 5 second refresh


  Serial.print(" ");
  Serial.print(recordPeak2, 1);  // Print highest recorded peak with the 5 second refresh
  distance[0] = sqrt(pow((0 - recordPeak1), 2) + pow((0 - recordPeak2), 2));
  distance[1] = sqrt(pow((7.5 - recordPeak1), 2) + pow((7.5 - recordPeak2), 2));
  distance[2] = sqrt(pow((100 - recordPeak1), 2) + pow((100 - recordPeak2), 2));
  distance[3] = sqrt(pow((3 - recordPeak1), 2) + pow((3 - recordPeak2), 2));
  distance[4] = sqrt(pow((50 - recordPeak1), 2) + pow((50 - recordPeak2), 2));
  if (false) {
    Serial.print(" ");
    Serial.print(distance[0], 1);  // Print highest recorded peak with the 5 second refresh
    Serial.print(" ");
    Serial.print(distance[1], 1);  // Print highest recorded peak with the 5 second refresh
    Serial.print(" ");
    Serial.print(distance[2], 1);  // Print highest recorded peak with the 5 second refresh
  }
  averageProbe = (recordPeak1 + recordPeak2) / 2;
  // Serial.print(" ");
  // Serial.println(averageProbe, 1);  // Print highest recorded peak with the 5 second refresh

  if (cycle == 10) {
    cycle = 0;
    recordPeak1 = 0.0;
    recordPeak2 = 0.0;
  }  // 20*25 = 500 = 0.5 second cycle reset, for both emg sets
     // Serial.print("stuff \t");  // Print highest recorded peak with the 5 second refresh

  // Serial.print(handControl[2][1], 1);    // Print highest recorded peak with the 5 second refresh
  // Serial.print(handControl[2][2], 1);    // Print highest recorded peak with the 5 second refresh
  // Serial.print(handControl[2][3], 1);  // Print highest recorded peak with the 5 second refresh
  Serial.print("\n");
}

void handOpen() {

  for (int j = 10; j < 170; j++)  // goes from 10 degrees to 170 degrees
  {                               // in steps of 1 degree
    ID1.write(170 - j);           // tell servo to go to open position
    ID2.write(j);                 // tell servo to go to open position
    ID3.write(j);                 // tell servo to go to open position
    ID4.write(j);                 // tell servo to go to open position
    ID5.write(j);                 // tell servo to go to open position
    delay(4);                     // waits 15ms for the servo to reach the position
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
    delay(4);            // waits 15ms for the servo to reach the position
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


  int smallestindex = distanceIndexMax;
  float smallest = pow(2, 31);
  for (int i = 0; i < distanceIndexMax; i++) {
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
      Serial.println("1");
      handControl[2][0] = 0;
      handControl[2][1] = 0;
      handControl[2][2] = 0;
      handControl[2][3] = 0;
      handControl[2][4] = 0;
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

void activatePos() {

  for (int fingeriteration = 0; fingeriteration < 5; fingeriteration++) {
    if (handControl[2][fingeriteration]) {
      if (handControl[0][fingeriteration] == 0) {
        fingerOpen(fingeriteration);
        handControl[0][fingeriteration] = 1;
      }
    } else {
      if (handControl[0][fingeriteration] == 1) {
        fingerClose(fingeriteration);
        handControl[0][fingeriteration] = 0;
      }
    }
  }
}

void fingerOpen(int fingerActive) {

  switch (fingerActive) {

    case 0:
      for (int j = 10; j < 170; j+=10) {
        ID1.write(170 - j);
        delay(5);
      }
      break;

    case 1:
      for (int j = 10; j < 170; j+=10) {
        ID2.write(j);
        delay(5);
      }
      break;

    case 2:
      for (int j = 10; j < 170; j+=10) {
        ID3.write(j);
        delay(5);
      }
      break;

    case 3:
      for (int j = 10; j < 170; j+=10) {
        ID4.write(j);
        delay(5);
      }
      break;

    case 4:
      for (int j = 10; j < 170; j+=10) {
        ID5.write(j);
        delay(5);
      }
      break;
  }
}

void fingerClose(int fingerActive) {

  switch (fingerActive) {

    case 0:
      for (int j = 160; j >= 1; j-=10) {
        ID1.write(180 - j);
        delay(5);
      }
      break;

    case 1:
      for (int j = 160; j >= 1; j-=10) {
        ID2.write(j);
        delay(5);
      }
      break;

    case 2:
      for (int j = 160; j >= 1; j-=10) {
        ID3.write(j);
        delay(5);
      }
      break;

    case 3:
      for (int j = 160; j >= 1; j-=10) {
        ID4.write(j);
        delay(5);
      }
      break;

    case 4:
      for (int j = 160; j >= 1; j-=10) {
        ID5.write(j);
        delay(5);
      }
      break;
  }
}


void loop() {

  while (true) {

    activatePos();  // this function avoids the need for setup, already initialized


    emgSetup();     // Setup for raw input array according to appropriate samples per cycle
    filterState();  // Rectified Filtered and Unfiltered voltage.

    peakDetector();  // Serial print cycle peak detection
    checkPos();      // checks & updates position, state and activate

    delay(25);  // Delay for imaging capture, this will also change the cycle refresh for recordPeak
  }
}

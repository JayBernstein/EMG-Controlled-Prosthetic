const int emgPin = A4;                  // Analog input pin for the sEMG signal
const int samplesPerCycle = 100;        // Array samples

float normalizedValueArray[samplesPerCycle];
float filteredValueArray[samplesPerCycle];
float unfilteredValueArray[samplesPerCycle];

int rawValueArray[samplesPerCycle];
int cycle = 0;                          // Number cycles elapsed

float dcOffset = 1.622;                 // DC voltage offset
float recordPeak = 0.0;                 // Highest recorded peak value

void setup() {
  Serial.begin(9600);                   // Initialize serial communication buad rate
}

//Low pass chebyshev filter order=1 alpha1=0.034090909090909 

class  FilterChLp1
{
	public:
		FilterChLp1()
		{
			v[0]=v[1]=0.0;
		}
	private:
		float v[2];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = (9.728285932334473918e-2 * x)
				 + (0.80543428135331052165 * v[0]);
			return 
				 (v[0] + v[1]);
		}
};


FilterChLp1 chebyshevFilter;                         // First order chebyshev LPF 44kHz at a 1500Hz low corner cut off

void emgSetup() {
  
    for (int i = 0; i < samplesPerCycle; i++) {
    rawValueArray[i] = analogRead(emgPin);           // Value is 0-1023
    delayMicroseconds(25);                           // FILTER NEEDS TO MATCH sample rate. 25uS = 40kHz currently. Anything slower will result in artifacting and distortion
    
  } 
  
}

void filterState() {

    for (int i = 0; i < samplesPerCycle; i++) {
    
    normalizedValueArray[i] = rawValueArray[i] / 1023.0;                                                                    // This is the normalized raw value array from 0.0 to 1.0
    filteredValueArray[i] = pow(2.71828, 3.3 * abs((chebyshevFilter.step(normalizedValueArray[i]) * 3.3) - dcOffset)) - 1;  // Apply LPF to voltage minus the offset. Then rectify values. e^3.3x - 1. Arbitrary values
    unfilteredValueArray[i] = pow(2.71828, 3.3 * abs((normalizedValueArray[i] * 3.3) - dcOffset)) - 1;                      // Unfiltered voltage minus the offset. Then rectify values. e^3.3x - 1. Arbitrary values
    
  }

}

void peakDetector(){
  
  float filtPeak = 0.0;
  float unfiltPeak = 0.0;
  
  float tempFilt = 0.0;
  float tempUnfilt = 0.0;
  
 for (int i = 0; i < samplesPerCycle; i++) {
  
  if(tempFilt < filteredValueArray[i]){ tempFilt = filteredValueArray[i]; }
  
  if(tempUnfilt < unfilteredValueArray[i]){ tempUnfilt = unfilteredValueArray[i]; }
  
 }
 
  filtPeak = tempFilt;          // filtPeak will hold the peak, we can use these to activate leds VIA threshold
  tempFilt = 0.0;               // Re-initialize tempFilt
  Serial.println(filtPeak);
  
  Serial.print(" ");
  
  unfiltPeak = tempUnfilt;      // unfiltPeak will hold the peak, we can use these to activate leds VIA threshold
  tempUnfilt = 0.0;             // Re-initialize tempUnfilt
  Serial.print(unfiltPeak);
  
  if(cycle < 20){    // 20*250 = 5000 = 5 second cycle reset, count starts from 0
    
    if(recordPeak < filtPeak){ recordPeak = filtPeak};
    
    cycle = cycle++; // increment cycle
  }
  
  Serial.print(" ");
  Serial.print(recordPeak);      // Print highest recorded peak with the 5 second refresh

  if(cycle == 20){ cycle = 0; }  // 20*250 = 5000 = 5 second cycle reset
  
  // set thresholds here to activate led
  
}

void serialOutput() {
 for (int i = 0; i < samplesPerCycle; i++) {
  
  Serial.print(0.0);
  Serial.print(" ");
  Serial.print(3.3);
  Serial.print(" ");
  Serial.print(unfilteredValueArray[i], 2);
  Serial.print(" ");
  Serial.print(filteredValueArray[i], 2);

 }
}

void loop() {
  
  while (true) {
    
    emgSetup();               // Setup for raw input array according to appropriate samples per cycle
    filterState();            // Rectified Filtered and Unfiltered voltage.
    
    //serialOutput();           // Serial outputs for monitor
    peakDetector();           // Serial print cycle peak detection

    delay(250);              // Delay for imaging capture, this will also change the cycle refresh for recordPeak
  }
}
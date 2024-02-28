const int samplesPerCycle = 500;  //Keep in mind 32 bits * 5000 = 20kB
int rawValueArray[samplesPerCycle];
float normalizedValueArray[samplesPerCycle];
float filteredValueArray[samplesPerCycle];
float movingAverageResultArray[samplesPerCycle];

const int emgPin = A4;       // Analog input pin for the sEMG signal
const int gain = 1;          // Set the amplifier gain
const int numReadings = 10;  // Number of readings to average

int readings[numReadings];  // Array to store readings
int index1 = 0;             // Index for storing new readings
int total = 0;              // Total of the readings

float filtVolt = 0.0;    // Initialize variable
float unfiltVolt = 0.0;  // Initialize variable
float avgVolt = 0.0;     // Initialize variable

void setup() {
  Serial.begin(115200);  // Initialize serial communication buad rate
}


//Band pass chebyshev filter order=2 alpha1=0.0025 alpha2=0.05 
class  FilterChBp2
{
	public:
		FilterChBp2()
		{
			for(int i=0; i <= 4; i++)
				v[i]=0.0;
		}
	private:
		float v[5];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = v[3];
			v[3] = v[4];
			v[4] = (1.184515651623030581e-2 * x)
				 + (-0.82576824486942723702 * v[0])
				 + (3.41129610559370632927 * v[1])
				 + (-5.34444378593883495654 * v[2])
				 + (3.75889373385177583131 * v[3]);
			return 
				 (v[0] + v[4])
				- 2 * v[2];
		}
    
    void reInitialize()
    {
      for(int i=0; i <= 4; i++)
				v[i]=0.0;
    }
    

};


FilterChBp2 chebyshevFilter;
int rawValue;

void filterState() {
  //chebyshevFilter.reInitialize();
  for (int i = 0; i < samplesPerCycle; i++) {
    normalizedValueArray[i] = rawValueArray[i];                             // Normalized signal from 0 to 1
    filteredValueArray[i] = chebyshevFilter.step(normalizedValueArray[i]);  // Apply LPF and convert back to voltage. Always an option to multiply by 3.3 to get true voltage reading
  }
  /* commented because the mapped value isn't needed, its just a 3.3x gain of normalized signal
  filtVolt = chebyshevFilter.step(signalNormal) * 3.3;  // Apply LPF and convert back to voltage
  unfiltVolt = signalNormal * 3.3;                      // Unfiltered value and converted back to voltage
  */
}

void movingAverage() {
  //int total = 0;  //reinitialize
  //int index1=0;
  for (int i = 0; i < samplesPerCycle; i++) {
    total -= readings[index1];                         // Subtract the oldest reading from the total
    readings[index1] = rawValueArray[i];               // Store the new reading in the array
    total += rawValueArray[i];                                    // Add the new reading to the total
    index1 = (index1 + 1) % 10;               // Move to the next index
    int average = total / 10;                 // Calculate the moving average of rawValue
    movingAverageResultArray[i] = (average / 4096.0);  // Converted to voltage
  }
}

void serialOutput() {
 for (int i = 0; i < samplesPerCycle; i++) {
  Serial.print(0.5);
  Serial.print(" ");
  Serial.print(0.7);
  Serial.print(" ");
    Serial.print(rawValueArray[i], 5);
  Serial.print(" ");
  Serial.print(filteredValueArray[i], 5);
  Serial.print(" ");
  Serial.print(normalizedValueArray[i], 5);
  Serial.print(" ");
  Serial.println(movingAverageResultArray[i], 5);
 }
}

void loop() {
  while (true) {
    for (int i = 0; i < samplesPerCycle; i++) {
      rawValueArray[i] = analogRead(emgPin);  //value is 0-1023
      delayMicroseconds(25);                  //FILTER NEEDS TO MATCH sample rate. 25uS=40kHz currently. Anything slower will result in artifacting and distortion
    }

    //chebyshevFilter.reInitialize();  //get filtered output
    for (int i = 0; i < samplesPerCycle; i++) {
      normalizedValueArray[i] = rawValueArray[i]/4096.0; 
      filteredValueArray[i] = chebyshevFilter.step(normalizedValueArray[i])+0.122;  // Apply LPF and convert back to voltage. Always an option to multiply by 3.3 to get true voltage reading
    }

    movingAverage();  // Averaging filter for capture
    serialOutput();   // Serial outputs for monitor

    delay(3000);
  }
}
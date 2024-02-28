const int emgPin = A4;           // Analog input pin for the sEMG signal
const int gain = 1;              // Set the amplifier gain
const int numReadings = 10;      // Number of readings to average

int readings[numReadings];       // Array to store readings
int index1 = 0;                   // Index for storing new readings
int total = 0;                   // Total of the readings

float filtVolt = 0.0;            // Initialize variable
float unfiltVolt = 0.0;          // Initialize variable
float avgVolt = 0.0;             // Initialize variable

void setup() {
  Serial.begin(9600);            // Initialize serial communication buad rate
}

class  FilterChLp2               // Low pass chebyshev filter order=2 alpha1=0.0090702947845805 
{
	public:
		FilterChLp2()
		{
			v[0]=0.0;
			v[1]=0.0;
			v[2]=0.0;
		}
	private:
		float v[3];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = (7.318635633972170318e-4 * x)
				 + (-0.94881443361775485990 * v[0])
				 + (1.94588697936416599177 * v[1]);
			return 
				 (v[0] + v[2])
				+2 * v[1];
		}
};

FilterChLp2 chebyshevFilter;
int rawValue; 

void filterState(){

  rawValue = analogRead(emgPin);                                // Read the raw sEMG signal 0 to 1023
  float signalNormal = (rawValue / 1023.0);                         // Normalized signal from 0 to 1
 
  filtVolt = gain * chebyshevFilter.step(signalNormal) * 3.3;       // Apply LPF and convert back to voltage
  unfiltVolt = gain * signalNormal * 3.3;                           // Unfiltered value and converted back to voltage

}

void movingAverage(int input){
  
  total -= readings[index1];                                         // Subtract the oldest reading from the total
  readings[index1] = input;                                          // Store the new reading in the array
  total += input;                                                   // Add the new reading to the total
  index1 = (index1 + 1) % numReadings;                                // Move to the next index
  
  int average = total / numReadings;                                // Calculate the moving average of rawValue
  
  avgVolt = gain * (average / 1023.0) * 3.3;                        // Converted to voltage

}

void serialOutput(){
  
    Serial.print(0);
    Serial.print(" ");
    Serial.print(3.3);
    Serial.print(" ");
    Serial.print(filtVolt, 4);
    Serial.print(" ");
    Serial.print(unfiltVolt, 4);
    Serial.print(" ");
    Serial.println(avgVolt, 4);

}

void loop() {
  while (true) {
    
    filterState();                                                  // Conversions for filtered value
    movingAverage(rawValue);                                        // Averaging filter for capture
    serialOutput();                                                 // Serial outputs for monitor
    
    //delay(50);                                                    // This will recover a 20Hz signal or 1 / 50 mS, but will introduce aliasing with higher freq components
    delayMicroseconds(22.68);                                       // Ideally at 44100Hz the sampling rate should be 22.68 muS or 1 / sampled rate. Anything slower will result in artifacting and distortion
    
  }
}
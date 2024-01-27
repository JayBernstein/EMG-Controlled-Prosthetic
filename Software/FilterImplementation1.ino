const int emgPin = A4;           // Analog input pin for the sEMG signal
const int amplifierGain = 1000;  // Set the amplifier gain

void setup() {
  Serial.begin(9600);  // Initialize serial communication
}

//Low pass chebyshev filter order=2 alpha1=0.0090702947845805 
class  FilterChLp2
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

void filterState(){

  int rawValue = analogRead(emgPin);// Read the raw sEMG signal
  int gain = 1; // Adjust this value to increase gain
  //int filterControl[5] = {0, 0, 0, 0, 0}; // Range from 0 to 1023 from EMG input.
  float filterValue = 0.0; // Used for normalization

  float rawValueConvert = rawValue / 1023.0;
    
  filterValue = chebyshevFilter.step(gain * rawValueConvert); // Normalized EMG input between 0 and 1, and apply LPF
 // Serial.print("rawValue: ");
 // Serial.println(rawValueConvert, 4);
  
 // Serial.print("filterValue: ");
  Serial.print(0);
  Serial.print(" ");
  Serial.print(1);
  Serial.print(" ");
  Serial.print(filterValue, 4);
  Serial.print(" ");
  Serial.println(rawValueConvert, 4);
  delay(50);

}

void loop() {
  while (true) {
    //float voltage = (float)rawValue * (3.3 / 1023.0) * amplifierGain;  // Convert to millivolts (mV)
    filterState();
    // Send time and voltage data to the Serial Plotter
    
  }
}
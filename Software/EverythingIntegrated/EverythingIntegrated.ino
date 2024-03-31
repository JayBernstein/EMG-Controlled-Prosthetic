#include <Servo.h>

Servo ID1;  // Define all servo motors for hand
Servo ID2;  
Servo ID3;
Servo ID4;
Servo ID5;

int pin0 = 28;
int pin1 = 29;
int pin2 = 30;
int pin3 = 31;
int pin4 = 32;

bool handControl[3][5] = {
{0, 0, 0, 0, 0}, // Checks the active state of each finger (handState)
{0, 0, 0, 0, 0}, // Checks previous position of finger (handPosition)
{1, 1, 1, 1, 1}  // Checks whether the finger should move based on active state & previous position (handActivate)
};

const int emgPin = A4;                  // Analog input pin for the sEMG signal
const int samplesPerCycle = 1000;        // Array samples

float normalizedValueArray[samplesPerCycle];
float filteredValueArray[samplesPerCycle];
float unfilteredValueArray[samplesPerCycle];

int rawValueArray[samplesPerCycle];
int cycle = 0;                          // Number cycles elapsed

float dcOffset = 1.622;                 // DC voltage offset
float recordPeak = 0.0;                 // Highest recorded peak value

void setup() {
  Serial.begin(115000);                   // Initialize serial communication buad rate

  ID1.attach(23); 
  ID2.attach(22);  
  ID3.attach(21);  
  ID4.attach(20);  
  ID5.attach(19);
}

//Low pass chebyshev filter order=1 alpha1=0.0375 
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
			v[1] = (1.060566927263208309e-1 * x)
				 + (0.78788661454735831047 * v[0]);
			return 
				 (v[0] + v[1]);
		}
};


FilterChLp1 chebyshevFilter;                         // First order chebyshev LPF 40kHz at a 1500Hz low corner cut off

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
  
  if(cycle < 15){    // 20*250 = 5000 = 5 second cycle reset, count starts from 0
    
    if(recordPeak < filtPeak){ recordPeak = filtPeak;}
    
    cycle = cycle +1; // increment cycle
  }
  
  Serial.print(" ");
  Serial.print(recordPeak, 1);      // Print highest recorded peak with the 5 second refresh

  if(cycle == 15){ cycle = 0; recordPeak = 0.0;}  // 20*250 = 5000 = 5 second cycle reset
  
  
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

void checkPos(){

  //filterState();

  for(int i = 0; i < 5; i++){

    if(handControl[0][i] == handControl[1][i]){

      handControl[2][i] = 0;

    }

    else if(handControl[0] != handControl[1]){

      handControl[2][i] = 1;

      handControl[1][i] = handControl[0][i];      
    }

  }

}

void activatePos(){

  for(int i = 0; i < 5; i++){

    if(handControl[2][i] == true){

      switch(handControl[1][i]){

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

void fingerOpen(int fingerActive){

  switch(fingerActive){

    case 0:
      for(int j = 10; j < 170; j++){                                  
        ID1.write(170 - j);              
        delay(4);                       
      } 
      break;
    
    case 1:
      for(int j = 10; j < 170; j++){                                  
        ID2.write(j);              
        delay(4);                       
      } 
      break;
    
    case 2:
      for(int j = 10; j < 170; j++){                                  
        ID3.write(j);              
        delay(4);                       
      } 
      break;

    case 3:
      for(int j = 10; j < 170; j++){                                  
        ID4.write(j);              
        delay(4);                       
      } 
      break;

    case 4:
      for(int j = 10; j < 170; j++){                                  
        ID5.write(j);              
        delay(4);                       
      } 
      break;
  }

}

void fingerClose(int fingerActive){

  switch(fingerActive){

    case 0:
      for(int j = 180; j >= 1; j--){                                  
        ID1.write(180 - j);              
        delay(4);                       
      } 
      break;
    
    case 1:
      for(int j = 180; j >= 1; j--){                                  
        ID2.write(j);              
        delay(4);                       
      } 
      break;
    
    case 2:
      for(int j = 180; j >= 1; j--){                                  
        ID3.write(j);              
        delay(4);                       
      } 
      break;

    case 3:
      for(int j = 180; j >= 1; j--){                                  
        ID4.write(j);              
        delay(4);                       
      } 
      break;

    case 4:
      for(int j = 180; j >= 1; j--){                                  
        ID5.write(j);              
        delay(4);                       
      } 
      break;
  }


}

void engageFinger(){

  

    if(recordPeak > 160 && recordPeak < 250){

      handControl[0][2] = 1;

    }

    else if (recordPeak < 10){

      handControl[0][2] = 0;


    }

    if(recordPeak > 30 && recordPeak < 130){

      handControl[0][3] = 1;

    }

    else if (recordPeak < 10){

      handControl[0][3] = 0;


    }


 



}

void loop() {
  
  while (true) {
    
    activatePos(); //this function avoids the need for setup, already initialized
    checkPos(); //checks & updates position, state and activate

    emgSetup();               // Setup for raw input array according to appropriate samples per cycle
    filterState();            // Rectified Filtered and Unfiltered voltage.
    
    //serialOutput();           // Serial outputs for monitor
    peakDetector();           // Serial print cycle peak detection

    //engageFinger();

    delay(25);              // Delay for imaging capture, this will also change the cycle refresh for recordPeak
  }
}
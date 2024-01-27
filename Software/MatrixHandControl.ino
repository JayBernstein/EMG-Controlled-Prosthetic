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
int pin4 =32;

bool handControl[3][5] = {
{0, 0, 0, 0, 0}, // Checks the active state of each finger (handState)
{0, 0, 0, 0, 0}, // Checks previous position of finger (handPosition)
{1, 1, 1, 1, 1}  // Checks whether the finger should move based on active state & previous position (handActivate)
};

void setup() {
  Serial.begin(9600);

  ID1.attach(23); 
  ID2.attach(22);  
  ID3.attach(21);  
  ID4.attach(20);  
  ID5.attach(19);

  pinMode(pin0, INPUT);
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(pin4, INPUT);

}

void checkPos(){

  filterState();

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

void filterState(){

  handControl[0][0] = digitalRead(pin0); //This is for dipswitch configuration & testing
  handControl[0][1] = digitalRead(pin1); //EMG reading will call seperate filter function to set hand states
  handControl[0][2] = digitalRead(pin2);
  handControl[0][3] = digitalRead(pin3);
  handControl[0][4] = digitalRead(pin4);

}

void loop() {

  activatePos(); //this function avoids the need for setup, already initialized
  checkPos(); //checks & updates position, state and activate

}

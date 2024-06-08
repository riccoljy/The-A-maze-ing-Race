// Give names to the pins of Arduino to be used and 
// define their values as integer 
int Ena = 7;	// IO port 7 will be connected to Pin 9 (Enable) of L293D
int Mot1 = 5;	// IO port 5 will be connected to Pin 15 of L293D
int Mot2 = 6; 	// IO port 6 will be connected to Pin 10 of L293D
int tp;		// Define a variable for period of PWM
int tON;	// Define a variable for ON time
int tOFF;	// Define a variable for OFF time
int ActiveMot = Mot1;
long counter;


// Following section will be run once at the beginning. 
void setup()
{
	pinMode(Ena, OUTPUT);	// OUTPUT from Arduino
	pinMode(Mot1, OUTPUT);	// OUTPUT from Arduino
	pinMode(Mot2, OUTPUT);	// OUTPUT from Arduino

	digitalWrite(Ena, HIGH); 	//Enable pin is set to logic HIGH
	digitalWrite(Mot1, LOW);	// Both control pins are initialized  
	digitalWrite(Mot2, LOW);	// to LOW so motor won't spin for now

	tp = 50; 	// Period is 500 microseconds; must be >= tON
	tON = 50;	// ON for 300 microseconds
	tOFF = tp-tON;	// OFF for remaining time of the period 
  float tp_in_sec = ((float) tp)/1000000;
  counter = (int) 10/tp_in_sec; // 10 seconds divide by tp = number of times to run this
  
}

// Following section toggles pin 15 of L293D between ON and OFF states 
// according to the duty cycle.  It will continue looping indefinitely.
void loop() 
{

 	digitalWrite(ActiveMot, HIGH); //port 5 (pin 15 of L293) is set to HIGH
	delayMicroseconds(tON);	  //wait for tON microseconds
	digitalWrite(ActiveMot, LOW);  //port 5 is set to LOW
	delayMicroseconds(tOFF);  //wait for tOFF microseconds
  counter--;

  
  if (counter == 0){
    counter = 200000;
    digitalWrite(ActiveMot, LOW);

    if (ActiveMot == Mot1) ActiveMot = Mot2;
    else {ActiveMot = Mot1;}
  }

}
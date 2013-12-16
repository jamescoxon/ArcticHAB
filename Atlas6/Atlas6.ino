// Code for Atlas6 - High altitude balloon
// Based on Teensy 3.0
// RTTY using Interrupts for timing (IntervalTimer lib which is built into the Teensyduino code)
//
// jacoxon@googlemail.com


int txPin = 13;
volatile uint32_t timer0count;
volatile char c;

IntervalTimer timer0;

// RTTY Functions - from RJHARRISON's AVR Code
void rtty_txstring (char * string)
{

	/* Simple function to sent a char at a time to 
	** rtty_txbyte function. 
	** NB Each char is one byte (8 Bits)
	*/
	c = *string++;
	while ( c != '\0')
	{
          timer0count = 0;
          timer0.begin(timerCallback0, 19500); // 19500 microseconds
          while(timer0count < 11){
          }
          timer0.end();
          c = *string++;
	}
}

void rtty_txbit (int bit)
{
  		if (bit)
		{
		  // high
                  digitalWrite(txPin, HIGH);
		}
		else
		{
		  // low
                  digitalWrite(txPin, LOW);
		}       
                
}

void timerCallback0() {
  
  // start, data, data, data, data, data, data, data, data, stop, stop
  if(timer0count == 0){
    rtty_txbit (0); // Start bit
  }
  else if(timer0count >= 1 && timer0count <= 8){
    if (c & 1) rtty_txbit(1); 
        else rtty_txbit(0);	
    c = c >> 1;
  }
  if(timer0count == 9 || timer0count == 10){
    rtty_txbit (1); // Stop bit
  }
  timer0count++;
  
}

void setup()
{
  pinMode(txPin, OUTPUT);
}

void loop()
{
  rtty_txstring("$$ATLAS,0,00:00:00,00.0000,00.0000,0000*0000\n");
  delay(1000);
}

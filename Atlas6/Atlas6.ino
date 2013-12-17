// Code for Atlas6 - High altitude balloon
// Based on Teensy 3.0
// RTTY using Interrupts for timing (IntervalTimer lib which is built into the Teensyduino code)
//
// jacoxon@googlemail.com

int txPin = 13;
volatile uint32_t timer0count;
volatile char c;

//Variables
int32_t lat = 514981000, lon = -530000, alt = 0;
uint8_t hour = 0, minute = 0, second = 0, month = 0, day = 0, lock = 0, sats = 0, navmode = 0, n = 1, GPSerror = 0;
char superbuffer [80]; //Telem string buffer
int count = 0;

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
          timer0.begin(timerCallback0, 19925); // 19925 (21000 - 18850 microseconds
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
  int n;
  count++;
  
  n=sprintf (superbuffer, "$$ATLAS,%d,%02d:%02d:%02d,%ld,%ld,%ld,%d,%d,%d", count, hour, minute, second, lat, lon, alt, sats, lock, navmode);
  n = sprintf (superbuffer, "%s*%04X\n", superbuffer, gps_CRC16_checksum(superbuffer));
  
  rtty_txstring("$$");
  rtty_txstring(superbuffer);
  delay(1000);
}

//************Other Functions*****************

uint16_t gps_CRC16_checksum (char *string)
{
	size_t i;
	uint16_t crc;
	uint8_t c;
 
	crc = 0xFFFF;
 
	// Calculate checksum ignoring the first two $s
	for (i = 2; i < strlen(string); i++)
	{
		c = string[i];
		crc = crc_xmodem_update (crc, c);
	}
 
	return crc;
}

uint16_t crc_xmodem_update (uint16_t crc, uint8_t data)
    {
        int i;

        crc = crc ^ ((uint16_t)data << 8);
        for (i=0; i<8; i++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }

        return crc;
    }
